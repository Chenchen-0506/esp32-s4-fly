# -*- coding: utf-8 -*-
"""PID 参数扫描：在物理/电机滞后约束下，自动挑选超调小、回正快、不振荡的姿态环增益。"""
import numpy as np
import drone_pid_sim as S

def osc_count(t, y, target, step_t=1.0):
    """首次到达90%目标后，穿越目标线的次数（越多越振荡）。"""
    tt = t[t >= step_t] - step_t
    yy = y[t >= step_t]
    try:
        i90 = np.where(yy >= 0.9*target)[0][0]
    except IndexError:
        return 99
    seg = yy[i90:] - target
    s = np.sign(seg)
    s[s == 0] = 1
    return int(np.sum(np.abs(np.diff(s)) > 0))

def eval_set(kp_ang, kp_rate, kd_rate, ki_rate=0.08):
    S.KP_ANG = kp_ang; S.KP_YAW_ANG = kp_ang*0.5
    S.KP_RATE[:] = [kp_rate, kp_rate, kp_rate*1.6]
    S.KD_RATE[:] = [kd_rate, kd_rate, kd_rate*1.5]
    S.KI_RATE[:] = [ki_rate, ki_rate, ki_rate*1.5]
    stp = S.simulate('step')
    dst = S.simulate('disturb')
    # roll 阶跃(10°)
    rise, ov, settle, sse, peak = S.step_metrics(stp['t'], stp['ang'][:,0], 10.0, 1.0)
    osc = osc_count(stp['t'], stp['ang'][:,0], 10.0)
    # roll 抗扰回正时间
    t0=1.0; tt=dst['t']; yy=np.abs(dst['ang'][:,0]); aft=tt>=t0; t2=tt[aft]; yy=yy[aft]
    rec=np.nan
    for j in range(len(t2)):
        if np.all(yy[j:]<=1.0): rec=t2[j]-t0; break
    return ov, settle, rec, osc, sse

rows=[]
for kp_rate in [0.20,0.28,0.36,0.44]:
    for kd in [0.004,0.008,0.012]:
        for kp_ang in [4.5,6.0,7.5]:
            ov,settle,rec,osc,sse = eval_set(kp_ang,kp_rate,kd)
            rows.append((kp_rate,kd,kp_ang,ov,settle,rec,osc,sse))

print("%-8s%-7s%-7s | %-6s %-7s %-7s %-4s %-6s" %
      ("KpRate","Kd","KpAng","超调%","调节s","回正s","振荡","稳态°"))
good=[]
for r in rows:
    kp_rate,kd,kp_ang,ov,settle,rec,osc,sse=r
    flag = ""
    if (not np.isnan(settle)) and ov<10 and osc<=1 and (not np.isnan(rec)) and rec<1.0:
        flag=" OK"; good.append(r)
    print("%-8.2f%-7.3f%-7.1f | %-6.1f %-7.2f %-7.2f %-4d %-6.2f%s" %
          (kp_rate,kd,kp_ang,ov,settle,rec,osc,sse,flag))

print("\n满足(超调<10%、振荡<=1、回正<1s)的组合 %d 组，按 调节时间+0.3*回正 排序：" % len(good))
good.sort(key=lambda r:(r[4] if not np.isnan(r[4]) else 9)+0.3*(r[5] if not np.isnan(r[5]) else 9))
for r in good[:6]:
    print("  KpRate=%.2f Kd=%.3f KpAng=%.1f -> 超调%.1f%% 调节%.2fs 回正%.2fs 振荡%d"
          % (r[0],r[1],r[2],r[3],r[4],r[5],r[6]))
