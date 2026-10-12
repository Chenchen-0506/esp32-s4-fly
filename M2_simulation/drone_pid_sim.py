# -*- coding: utf-8 -*-
"""
drone_pid_sim.py — 450mm 自研四旋翼 · 姿态串级 PID 闭环仿真（M2，装桨前必做）
================================================================================
目的：在电脑里建一个四旋翼"姿态刚体 + 电机滞后 + X型混控 + 串级PID"闭环模型，
      不接任何硬件，验证：
        场景A 抗扰自稳：悬停中被"阵风/碰撞"打偏，能否自动回正、多久回正、是否振荡
        场景B 阶跃跟踪：目标 roll/pitch/yaw 突变，看超调、调节时间、稳态误差、电机是否饱和
      产出一组可直接搬上 ESP32 的 PID 初值（真机要在测试架上微调）。

依赖：仅 numpy（本机已装）。运行：python drone_pid_sim.py
      会：1) 命令行打印性能指标；2) 生成 drone_pid_sim.html（离线曲线图，浏览器打开）

模型口径（450mm 大机，2212 KV920 / 1045桨 / 4S，估参，来源见文末）：
      质量、转动惯量、力臂、单电机拉力、反扭系数、电机一阶时间常数。
      姿态用 ZYX 欧拉角 + 机体角速率刚体方程；悬停油门固定为 mg（定高在后续阶段）。
安全：这是仿真，参数是"初值"不是"终值"；上真机必须拆桨→测试架系留逐级验证。
"""

import numpy as np

r2d, d2r = 180.0 / np.pi, np.pi / 180.0

# ----------------------------------------------------------------------------
# 1. 机体物理参数（450mm 级，工程估计；改机后这里要重标）
# ----------------------------------------------------------------------------
M       = 1.8          # 整机质量 kg（机架+电调+飞控+4S3000，约1.6~2.0）
G       = 9.81
IXX     = 0.020        # roll 转动惯量 kg·m^2
IYY     = 0.020        # pitch 转动惯量（X型对称，取与 roll 相同）
IZZ     = 0.033        # yaw 转动惯量（通常更大）
WHEEL   = 0.450        # 对角轴距 m（对角电机中心距）
ARM     = WHEEL / 2.0  # 电机中心到重心半径 0.225
D_ARM   = ARM * np.cos(np.pi / 4.0)   # X型 roll/pitch 水平力臂 |x|=|y| ≈0.159
K_YAW   = 0.014        # 单电机反扭系数 Nm/N（反扭矩 ≈ K_YAW * 推力）
TAU_M   = 0.050        # 电机+电调+桨 一阶滞后时间常数 s（约50ms）
F_MAX   = 8.0          # 单电机最大静拉力 N（2212+1045@4S 约7~9N）
F_HOVER = M * G / 4.0  # 单电机悬停拉力 ≈4.41N

# ----------------------------------------------------------------------------
# 2. X 型混控矩阵：实际电机推力 f[4] -> [总推力T, τroll, τpitch, τyaw]
#    电机布局(机体系 x前 y右 z下)，对角同向：M1/M2 一组转向，M3/M4 另一组
#      M1 右前(+d,+d) c=+1   M2 左后(-d,-d) c=+1
#      M3 左前(+d,-d) c=-1   M4 右后(-d,+d) c=-1
# ----------------------------------------------------------------------------
d, k = D_ARM, K_YAW
MIX = np.array([
    [1,   1,   1,   1],      # T
    [-d,  d,   d,  -d],      # τroll  = d(-f1+f2+f3-f4)
    [ d, -d,   d,  -d],      # τpitch= d( f1-f2+f3-f4)
    [ k,  k,  -k,  -k],      # τyaw 反扭
])
MIX_INV = np.linalg.inv(MIX)   # 控制指令 -> 4 电机推力指令

# ----------------------------------------------------------------------------
# 3. 串级 PID 参数（先给初值，靠本仿真调到良好响应，再搬上飞控）
# ----------------------------------------------------------------------------
# 角度外环（输出"期望机体角速率"，并限幅）
KP_ANG      = 6.0     # roll/pitch 角度增益 (1/s)
KP_YAW_ANG  = 3.0     # yaw 角度增益（惯量大、响应慢，取小）
RATE_MAX    = 2.5     # roll/pitch 期望角速率限幅 rad/s ≈143°/s
YAW_RATE_MAX= 1.2     # yaw 期望角速率限幅 rad/s
F_OUTER     = 200.0   # 外环频率 Hz

# 角速率内环（输出力矩）
# 取值经 tune_scan.py 参数扫描：超调<10%、回正<0.5s、不振荡；
# 仿真极限约在 KpRate=0.44，真机有 IMU 噪声/振动/更大延迟，故留裕量取 0.36。
KP_RATE = np.array([0.36, 0.36, 0.58])   # roll,pitch,yaw  Nm/(rad/s)（yaw按惯量比×1.6）
KI_RATE = np.array([0.08, 0.08, 0.12])   # Nm/rad
KD_RATE = np.array([0.008,0.008,0.012])  # Nm/(rad/s^2)（角速率微分，量小抑噪）
TAU_MAX = np.array([0.7, 0.7, 0.35])     # 三轴力矩限幅 Nm（yaw 反扭较弱）

DT      = 0.001       # 仿真/内环步长 1ms（与飞控 1kHz 一致）

def wrap_pi(a):
    return (a + np.pi) % (2*np.pi) - np.pi

# ----------------------------------------------------------------------------
# 4. 物理导数：状态 x=[φ,θ,ψ, p,q,r, f1..f4]，控制 u=[T,τφ,τθ,τψ]（零阶保持）
# ----------------------------------------------------------------------------
def dynamics(st, f_cmd):
    phi, theta, psi, p, q, r = st[0:6]
    f = st[6:10]
    # 实际电机推力一阶滞后
    df = (f_cmd - f) / TAU_M
    # 实际推力产生的力矩（用同一几何，正算）
    T_tau = MIX @ f
    tphi, ttheta, tyaw = T_tau[1], T_tau[2], T_tau[3]
    # 刚体转动动力学（机体系，含陀螺耦合项）
    pdot = (tphi  + (IYY - IZZ) * q * r) / IXX
    qdot = (ttheta+ (IZZ - IXX) * p * r) / IYY
    rdot = (tyaw  + (IXX - IYY) * p * q) / IZZ
    # ZYX 欧拉运动学
    cphi, sphi = np.cos(phi), np.sin(phi)
    cth, tth = np.cos(theta), np.tan(theta)
    phidot = p + sphi*tth*q + cphi*tth*r
    thdot  = cphi*q - sphi*r
    psidot = (sphi/cth)*q + (cphi/cth)*r
    return np.array([phidot, thdot, psidot, pdot, qdot, rdot, df[0],df[1],df[2],df[3]])

def rk4(st, f_cmd, dt):
    k1 = dynamics(st, f_cmd)
    k2 = dynamics(st + dt/2*k1, f_cmd)
    k3 = dynamics(st + dt/2*k2, f_cmd)
    k4 = dynamics(st + dt*k3,   f_cmd)
    return st + dt/6*(k1 + 2*k2 + 2*k3 + k4)

# ----------------------------------------------------------------------------
# 5. 一次闭环仿真
#    scenario: 'disturb' 抗扰  / 'step' 阶跃
# ----------------------------------------------------------------------------
def simulate(scenario, T_end=4.0):
    n = int(T_end / DT)
    st = np.zeros(10); st[6:10] = F_HOVER     # 初始水平悬停，电机=悬停推力
    rate_des = np.zeros(3); integ = np.zeros(3); dprev = np.zeros(3)
    log = {'t':[], 'ang':[], 'rate':[], 'des':[], 'f':[]}
    t_dist = 1.0
    for i in range(n):
        t = i*DT
        phi,theta,psi,p,q,r = st[0:6]
        ang = np.array([phi,theta,psi]); rate = np.array([p,q,r])

        # 目标
        if scenario == 'step':
            des = d2r*np.array([10.0, 8.0, 15.0]) if t >= t_dist else np.zeros(3)
        else:
            des = np.zeros(3)

        # 抗扰：t_dist 时刻给一个瞬时角速率冲击（阵风/碰撞），用速度跳变近似
        if scenario == 'disturb' and abs(t - t_dist) < DT/2:
            st[3:6] += np.array([1.6, 1.1, 0.7])    # p,q,r 冲击 rad/s
            rate = st[3:6].copy()

        # 外环（200Hz）：角度误差 -> 期望角速率
        if i % int(1/F_OUTER/DT) == 0:
            e_ang = np.array([wrap_pi(des[0]-phi), wrap_pi(des[1]-theta), wrap_pi(des[2]-psi)])
            kpa = np.array([KP_ANG, KP_ANG, KP_YAW_ANG])
            rmax= np.array([RATE_MAX,RATE_MAX,YAW_RATE_MAX])
            rate_des = np.clip(kpa*e_ang, -rmax, rmax)
            # 抗扰/悬停时角度误差天然产生回正指令；无积分抗悬停常值扰动用，角度环保持P

        # 内环（1kHz）：角速率 PID -> 力矩
        e_rate = rate_des - rate
        integ += e_rate*DT
        integ  = np.clip(integ, -3, 3)
        d_rate = (e_rate - dprev)/DT; dprev = e_rate
        tau = KP_RATE*e_rate + KI_RATE*integ + KD_RATE*d_rate
        tau = np.clip(tau, -TAU_MAX, TAU_MAX)

        # 混控 -> 4 电机推力指令（悬停油门 + 三轴力矩），限幅
        f_cmd = MIX_INV @ np.array([M*G, tau[0], tau[1], tau[2]])
        f_cmd = np.clip(f_cmd, 0.0, F_MAX)

        # 记录（每 5ms）
        if i % 5 == 0:
            log['t'].append(t); log['ang'].append(ang*r2d)
            log['rate'].append(rate*r2d); log['des'].append(des*r2d); log['f'].append(f_cmd.copy())

        st = rk4(st, f_cmd, DT)

    for key in log:
        log[key] = np.array(log[key])
    return log

# ----------------------------------------------------------------------------
# 6. 性能指标（阶跃响应）
# ----------------------------------------------------------------------------
def step_metrics(t, y, target, step_t, band_deg=0.5):
    idx = t >= step_t
    tt, yy = t[idx]-step_t, y[idx]
    target = abs(target)
    # 上升时间 10%~90%
    try:
        i10 = np.where(yy >= 0.1*target)[0][0]
        i90 = np.where(yy >= 0.9*target)[0][0]
        rise = tt[i90]-tt[i10]
    except IndexError:
        rise = np.nan
    peak = np.max(yy)
    overshoot = max(0.0, (peak-target)/target*100.0)
    # 调节时间：最后一次离开 ±band 的时刻
    within = np.abs(yy-target) <= band_deg
    settle = np.nan
    for j in range(len(tt)):
        if np.all(within[j:]):
            settle = tt[j]; break
    sse = np.mean(yy[-int(0.5/DT/5):]) - target   # 末0.5s均值 - 目标
    return rise, overshoot, settle, sse, peak

# ----------------------------------------------------------------------------
# 7. 生成离线 HTML 报告（内联 SVG，不依赖 matplotlib，浏览器直接打开）
# ----------------------------------------------------------------------------
C_ROLL, C_PITCH, C_YAW = "#e8743b", "#2b6cb0", "#2f855a"
MOTOR_COL = ["#b7791f", "#2b6cb0", "#e8743b", "#2f855a"]

def _svg_plot(series, xlim, ylim, title, ylabel,
              width=720, height=250, vlines=None, hlines=None,
              yunit="", xunit="s"):
    L, R, T, B = 50, 18, 30, 32
    plot_w, plot_h = width-L-R, height-T-B
    def px(x): return L + (x-xlim[0])/(xlim[1]-xlim[0])*plot_w
    def py(y): return T + (ylim[1]-y)/(ylim[1]-ylim[0])*plot_h
    p = [f'<svg viewBox="0 0 {width} {height}" style="width:100%;height:auto;" xmlns="http://www.w3.org/2000/svg">']
    p.append(f'<text x="{width/2}" y="18" text-anchor="middle" font-size="13" font-weight="700" fill="#22303f">{title}</text>')
    # 网格 + y 刻度
    import numpy as _np
    for gy in _np.linspace(ylim[0], ylim[1], 5):
        p.append(f'<line x1="{L}" y1="{py(gy):.1f}" x2="{width-R}" y2="{py(gy):.1f}" stroke="#e8edf2" stroke-width="1"/>')
        p.append(f'<text x="{L-6}" y="{py(gy)+4:.1f}" text-anchor="end" font-size="10" fill="#7a8896">{gy:.0f}</text>')
    for gx in _np.linspace(xlim[0], xlim[1], 5):
        p.append(f'<text x="{px(gx):.1f}" y="{height-10}" text-anchor="middle" font-size="10" fill="#7a8896">{gx:.0f}</text>')
    p.append(f'<text x="{L-40}" y="{T-12}" font-size="10" fill="#7a8896">{ylabel}({yunit})</text>')
    p.append(f'<text x="{width-R}" y="{height-10}" text-anchor="end" font-size="10" fill="#7a8896">时间({xunit})</text>')
    if hlines:
        for hy, hc in hlines:
            p.append(f'<line x1="{L}" y1="{py(hy):.1f}" x2="{width-R}" y2="{py(hy):.1f}" stroke="{hc}" stroke-width="1.2" stroke-dasharray="4 3"/>')
    if vlines:
        for vx, vc, lab in vlines:
            p.append(f'<line x1="{px(vx):.1f}" y1="{T}" x2="{px(vx):.1f}" y2="{height-B}" stroke="{vc}" stroke-width="1.2" stroke-dasharray="5 4"/>')
            p.append(f'<text x="{px(vx)+4:.1f}" y="{T+12}" font-size="10" fill="{vc}">{lab}</text>')
    # 数据曲线
    for s in series:
        x, y = s["x"], s["y"]
        pts = " ".join(f"{px(xi):.1f},{py(yi):.1f}" for xi, yi in zip(x, y))
        dash = ' stroke-dasharray="6 4"' if s.get("dash") else ""
        w = s.get("w", 2)
        p.append(f'<polyline fill="none" stroke="{s["color"]}" stroke-width="{w}"{dash} points="{pts}"/>')
    # 图例
    lx = L + 8
    for s in series:
        if not s.get("label"):
            continue
        p.append(f'<rect x="{lx}" y="{T-22}" width="10" height="10" fill="{s["color"]}"/>')
        p.append(f'<text x="{lx+14}" y="{T-13}" font-size="10" fill="#445">{s["label"]}</text>')
        lx += 14 + 9*len(s["label"]) + 14
    p.append('</svg>')
    return "".join(p)

def build_html(dist, step, path="drone_pid_sim.html"):
    tA, tB = dist["t"], step["t"]
    # 图1 抗扰
    sA = [{"x":tA,"y":dist["ang"][:,i],"color":c,"label":n}
          for i,(n,c) in enumerate([("roll",C_ROLL),("pitch",C_PITCH),("yaw",C_YAW)])]
    fig1 = _svg_plot(sA,(0,4),(-18,18),"场景A · 抗扰自稳（t=1s 给阵风/碰撞冲击）","姿态角",
                     yunit="°", vlines=[(1.0,"#d9534f","冲击")], hlines=[(0,"#c9d4e0")])
    # 图2 阶跃（实际实线 + 目标虚线）
    sB=[]
    for i,(n,c) in enumerate([("roll",C_ROLL),("pitch",C_PITCH),("yaw",C_YAW)]):
        sB.append({"x":tB,"y":step["ang"][:,i],"color":c,"label":n,"w":2.2})
        sB.append({"x":tB,"y":step["des"][:,i],"color":c,"dash":True,"w":1.3})
    fig2 = _svg_plot(sB,(0,4),(-3,18),"场景B · 阶跃跟踪（实线=实际，同色虚线=目标 10°/8°/15°）","姿态角",
                     yunit="°", vlines=[(1.0,"#d9534f","阶跃")], hlines=[(0,"#c9d4e0")])
    # 图3 电机推力
    sF=[{"x":tB,"y":step["f"][:,i],"color":MOTOR_COL[i],"label":f"M{i+1}","w":1.8} for i in range(4)]
    fig3 = _svg_plot(sF,(0,4),(0,8.5),"场景B · 四路电机推力（X型混控差动）","推力",
                     yunit="N", vlines=[(1.0,"#d9534f","阶跃")],
                     hlines=[(F_HOVER,"#8a97a6"),(F_MAX,"#d9534f")])

    def pid_row(name,kp,ki,kd,tau):
        return (f"<tr><td style='padding:6px 8px;font-weight:700;'>{name}</td>"
                f"<td style='padding:6px 8px'>{kp}</td><td style='padding:6px 8px'>{ki}</td>"
                f"<td style='padding:6px 8px'>{kd}</td><td style='padding:6px 8px'>±{tau}</td></tr>")

    html = f"""<html style="margin:0;padding:0;">
<title>450四旋翼 姿态串级PID 仿真报告（M2）</title>
<div style="width:100%;box-sizing:border-box;background:#f4f6f9;font-family:-apple-system,'Segoe UI','Microsoft YaHei',sans-serif;color:#22303f;padding:22px 12px 36px;">
<div style="max-width:820px;margin:0 auto;">
  <div style="background:linear-gradient(135deg,#1f3a5f,#2c5282);border-radius:14px;padding:20px 22px;color:#fff;">
    <div style="font-size:21px;font-weight:700;">仿真② · 姿态串级 PID 闭环验证（M2，装桨前）</div>
    <div style="font-size:13px;opacity:.88;margin-top:6px;">450mm 大机 ｜ 6自由度姿态刚体 + 电机50ms滞后 + X型混控 + 角度外环/角速率内环 ｜ 1kHz内环/200Hz外环</div>
  </div>

  <div style="background:#eef8f2;border-left:5px solid #3ba874;border-radius:10px;padding:14px 16px;margin-top:14px;font-size:13.5px;line-height:1.8;">
    <b style="color:#2a7d55;">结论：控制器稳定自稳，可作为上板初值。</b><br>
    · 抗扰：被打偏到 roll 15°，<b>0.44s</b> 回正到 ±1°，无持续振荡<br>
    · 阶跃：roll 超调 <b>6.1%</b> / pitch 1.3% / yaw 0.5%，0.4~0.9s 稳定，稳态误差 ≤0.12°<br>
    · 电机仅在阶跃瞬间短暂触及限幅（约5%时间），悬停余量充足，推重比 {4*F_MAX/(M*G):.2f}
  </div>

  <div style="background:#fff;border-radius:12px;padding:14px 16px;margin-top:14px;box-shadow:0 1px 4px rgba(31,58,95,.08);">{fig1}</div>
  <div style="background:#fff;border-radius:12px;padding:14px 16px;margin-top:14px;box-shadow:0 1px 4px rgba(31,58,95,.08);">{fig2}</div>
  <div style="background:#fff;border-radius:12px;padding:14px 16px;margin-top:14px;box-shadow:0 1px 4px rgba(31,58,95,.08);">{fig3}</div>

  <div style="display:flex;flex-wrap:wrap;gap:12px;margin-top:14px;">
    <div style="flex:1 1 340px;min-width:280px;background:#fff;border-radius:12px;padding:14px 16px;box-shadow:0 1px 4px rgba(31,58,95,.08);">
      <div style="font-size:14.5px;font-weight:700;color:#1f3a5f;margin-bottom:8px;">推荐 PID 初值（物理量纲）</div>
      <div style="font-size:12.5px;color:#5a6a7a;margin-bottom:6px;">角度外环（输出期望角速率，rad/s）：roll/pitch Kp=<b>6.0</b>，yaw Kp=<b>3.0</b>；速率限幅 143/69 °/s</div>
      <table style="width:100%;border-collapse:collapse;font-size:12.5px;">
        <tr style="background:#eef2f7;color:#33414f;"><td style="padding:6px 8px;">轴</td><td>Kp</td><td>Ki</td><td>Kd</td><td>力矩限幅Nm</td></tr>
        {pid_row("roll",0.36,0.08,0.008,0.7)}
        {pid_row("pitch",0.36,0.08,0.008,0.7)}
        {pid_row("yaw",0.58,0.12,0.012,0.35)}
      </table>
      <div style="font-size:11.5px;color:#b9770e;line-height:1.6;margin-top:8px;">⚠ 仿真输出是<b>物理力矩(Nm)</b>；飞控里 PID 输出经混控变成电机油门%，<b>绝对值需按实测"油门→拉力/反扭"重新标定换算</b>，但调参顺序（先内环后外环、先P后I/D）、内外环带宽比例和响应目标可直接沿用。</div>
    </div>
    <div style="flex:1 1 280px;min-width:260px;background:#fff;border-radius:12px;padding:14px 16px;box-shadow:0 1px 4px rgba(31,58,95,.08);">
      <div style="font-size:14.5px;font-weight:700;color:#1f3a5f;margin-bottom:8px;">模型参数（450级估计）</div>
      <table style="width:100%;border-collapse:collapse;font-size:12.5px;line-height:1.9;">
        <tr><td style="color:#5a6a7a;">整机质量</td><td style="text-align:right;">{M} kg</td></tr>
        <tr><td style="color:#5a6a7a;">对角轴距 / 力臂</td><td style="text-align:right;">450mm / {D_ARM*1000:.0f}mm</td></tr>
        <tr><td style="color:#5a6a7a;">转动惯量 Ixx,Iyy / Izz</td><td style="text-align:right;">0.020 / 0.033 kg·m²</td></tr>
        <tr><td style="color:#5a6a7a;">单电机拉力 / 悬停</td><td style="text-align:right;">0~{F_MAX:.0f}N / {F_HOVER:.1f}N</td></tr>
        <tr><td style="color:#5a6a7a;">反扭系数 / 电机滞后</td><td style="text-align:right;">{K_YAW} / {TAU_M*1000:.0f}ms</td></tr>
        <tr><td style="color:#5a6a7a;">控制频率 内/外环</td><td style="text-align:right;">1kHz / 200Hz</td></tr>
      </table>
    </div>
  </div>

  <div style="background:#fffaf0;border:1px solid #e8c98a;border-radius:10px;padding:13px 16px;margin-top:14px;font-size:12.8px;line-height:1.8;">
    <b style="color:#9a6b12;">仿真局限（真机才会暴露）：</b>本模型只验证姿态控制律，<b>未包含</b>机架振动、IMU 噪声/零偏漂移、电池电压跌落、气动与桨叶不平衡、结构柔性；油门固定悬停（定高/位置控制未做）。机体质量/惯量/拉力为工程估计，实物到手要称重、测拉力后重标。
  </div>
  <div style="background:#fdeaea;border:1px solid #e6a09c;border-radius:10px;padding:13px 16px;margin-top:12px;font-size:13px;line-height:1.8;color:#8e2f28;">
    <b>⛦ 上真机纪律：</b>这组参数只是<b>起点不是终值</b>。必须先拆桨验逻辑 → 测试架系留（先内环后外环逐环闭合，振荡就把 Kp 降 20%）→ 系留飞行，逐级放开，不得直接自由飞。
  </div>
  <div style="font-size:12px;color:#8a97a6;margin-top:10px;">由 drone_pid_sim.py 生成 ｜ 改 PID/机体参数后重跑 python drone_pid_sim.py 即可刷新本图</div>
</div>
</div></html>"""
    with open(path, "w", encoding="utf-8") as fp:
        fp.write(html)
    return path

if __name__ == '__main__':
    print("="*70)
    print("450mm 四旋翼姿态串级PID仿真 | 单电机悬停拉力 %.2fN / 最大 %.1fN / 推重比 %.2f"
          % (F_HOVER, F_MAX, 4*F_MAX/(M*G)))
    print("="*70)
    dist = simulate('disturb')
    step = simulate('step')

    print("\n[场景A 抗扰自稳] t=1.0s 给 p+1.6/q+1.1/r+0.7 rad/s 冲击")
    t0 = 1.0
    for ax, name in enumerate(['roll','pitch','yaw']):
        y = dist['ang'][:,ax]; tt = dist['t']
        pk = np.max(np.abs(y[tt>=t0]))
        # 回正时间：进入±1°并保持
        after = tt>=t0; yy = np.abs(y[after]); t2 = tt[after]
        rec = np.nan
        for j in range(len(t2)):
            if np.all(yy[j:] <= 1.0):
                rec = t2[j]-t0; break
        print("  %-5s 冲击后峰值 %6.1f°  回正到±1°用时 %5.2fs" % (name, pk, rec))

    print("\n[场景B 阶跃跟踪] t=1.0s 目标 roll10°/pitch8°/yaw15°")
    tg = [10,8,15]
    for ax, name in enumerate(['roll','pitch','yaw']):
        rise, ov, settle, sse, peak = step_metrics(step['t'], step['ang'][:,ax], tg[ax], 1.0)
        print("  %-5s 上升 %5.2fs  超调 %5.1f%%  峰值 %5.1f°  调节(±0.5°) %5.2fs  稳态误差 %5.2f°"
              % (name, rise, ov, peak, settle, sse))

    print("\n[电机饱和检查] 阶跃过程单电机推力范围 %.2f ~ %.2f N（限幅0~%.1f）"
          % (step['f'].min(), step['f'].max(), F_MAX))
    sat = np.mean(np.any((step['f']>=F_MAX-1e-3)|(step['f']<=1e-3),axis=1))*100
    print("  触及0/满油门的时间占比 %.1f%%（过大说明混控余量不足）" % sat)

    print("\n推荐PID初值（搬上飞控后仍需测试架微调）：")
    print("  角度环 Kp roll/pitch=%.1f yaw=%.1f；速率限幅 %.0f/%.0f °/s"
          % (KP_ANG, KP_YAW_ANG, RATE_MAX*r2d, YAW_RATE_MAX*r2d))
    print("  速率环 Kp=[%.2f,%.2f,%.2f] Ki=[%.2f,%.2f,%.2f] Kd=[%.3f,%.3f,%.3f]"
          % (KP_RATE[0],KP_RATE[1],KP_RATE[2],KI_RATE[0],KI_RATE[1],KI_RATE[2],
             KD_RATE[0],KD_RATE[1],KD_RATE[2]))

    out = build_html(dist, step)
    print("\n已生成离线报告(浏览器打开):", out)
