%% mahony_sim.m — Mahony 姿态解算仿真验证（M3 前置）
% 用途：在上板之前，用仿真 IMU 数据验证 Mahony 算法的收敛性与精度。
% 原理：
%   1) 构造真实姿态轨迹（欧拉角函数）→ 数值微分得到欧拉角速率
%   2) 由欧拉角速率转换出机体角速度（陀螺仪测量），由重力矢量投影
%      得到机体加速度计测量，均叠加高斯噪声
%   3) 用 Mahony 6 轴算法估计姿态，与真值对比
% 输出：
%   figure 1：roll/pitch/yaw 真值 vs 估计（三子图）
%   figure 2：姿态误差（度）
%   命令行打印最大/均方根误差
% 依赖：无工具箱，纯基础函数，R2018a+ 可直接运行。
%
% 注意：6 轴 Mahony 只用陀螺+加速度计，yaw 方向没有外部修正，
%       会出现缓慢漂移，这是正常现象（需要磁力计/GPS 航向修正）。

clear; clc; close all;
rng(0);                     % 固定随机种子，结果可复现

%% 1. 参数与时间轴
dt   = 0.001;               % 采样周期 1ms（1kHz）
T    = 10;                  % 仿真时长 10s
t    = 0:dt:T;
N    = length(t);

%% 2. 真实姿态轨迹（欧拉角，ZYX 顺序：roll->pitch->yaw）
phi   = deg2rad(15*sin(2*pi*0.20*t) +  5*sin(2*pi*0.80*t));
theta = deg2rad(10*sin(2*pi*0.25*t) +  3*sin(2*pi*0.70*t));
psi   = deg2rad( 6*sin(2*pi*0.05*t));   % yaw 小幅缓慢摆动

%% 3. 欧拉角速率（中心差分）
dphi   = [0, diff(phi)/dt];
dtheta = [0, diff(theta)/dt];
dpsi   = [0, diff(psi)/dt];

%% 4. 机体角速度（陀螺仪理想测量）
%  ZYX 欧拉角速率 -> 机体角速度：
%  wx = dphi - dpsi*sin(theta)
%  wy = dtheta*cos(phi) + dpsi*cos(theta)*sin(phi)
%  wz = -dtheta*sin(phi) + dpsi*cos(theta)*cos(phi)
wx = dphi - dpsi.*sin(theta);
wy = dtheta.*cos(phi) + dpsi.*cos(theta).*sin(phi);
wz = -dtheta.*sin(phi) + dpsi.*cos(theta).*cos(phi);

%% 5. 加速度计理想测量（机体系，单位 g）
%  重力在机体系投影：a_body = R' * [0;0;-1]
ax = -sin(theta);
ay =  cos(theta).*sin(phi);
az =  cos(theta).*cos(phi);

%% 6. 叠加传感器噪声
gyro_noise = 0.01;    % rad/s  (约 0.6 deg/s 1σ)
acc_noise  = 0.02;    % g
gx = wx + randn(1,N)*gyro_noise;
gy = wy + randn(1,N)*gyro_noise;
gz = wz + randn(1,N)*gyro_noise;
ax = ax + randn(1,N)*acc_noise;
ay = ay + randn(1,N)*acc_noise;
az = az + randn(1,N)*acc_noise;

%% 7. Mahony 6 轴算法
Kp = 1.2;   Ki = 0.05;
q = [1; 0; 0; 0];           % 四元数 (w,x,y,z)，初始水平
e_int = zeros(3,1);         % 积分误差

est = zeros(3, N);          % 存放估计欧拉角
for k = 1:N
    % 7.1 归一化加速度
    n = norm([ax(k); ay(k); az(k)]);
    if n > 1e-6
        a = [ax(k); ay(k); az(k)] / n;
    else
        a = [0; 0; 1];
    end

    % 7.2 由四元数估计重力方向（机体系）
    q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
    v = [2*(q1*q3 - q0*q2);
         2*(q0*q1 + q2*q3);
         q0*q0 - q1*q1 - q2*q2 + q3*q3];

    % 7.3 叉积误差
    e = cross(a, v);

    % 7.4 误差积分 + 比例补偿（gyro 单位 rad/s）
    e_int = e_int + e * (Ki * dt);
    w = [gx(k); gy(k); gz(k)] + Kp*e + e_int;

    % 7.5 四元数微分更新
    Omega = 0.5 * [ 0,   -w(1), -w(2), -w(3);
                    w(1),  0,    w(3), -w(2);
                    w(2), -w(3),  0,    w(1);
                    w(3),  w(2), -w(1),  0  ];
    q = q + Omega * q * dt;
    q = q / norm(q);        % 归一化

    % 7.6 用更新后的四元数 -> 欧拉角（ZYX）
    q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
    est(1,k) = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2));
    est(2,k) = asin(max(-1, min(1, 2*(q0*q2 - q3*q1))));
    est(3,k) = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2));
end

%% 8. 误差统计（角度制）
true_e = [phi; theta; psi];
err = (est - true_e) * 180/pi;
names = {'roll', 'pitch', 'yaw'};
for i = 1:3
    fprintf('%s: 最大误差 %6.2f°,  RMS 误差 %6.3f°\n', ...
        names{i}, max(abs(err(i,:))), sqrt(mean(err(i,:).^2)));
end

%% 9. 绘图
figure('Name', 'Mahony 姿态解算仿真', 'Color', 'w');
for i = 1:3
    subplot(3,1,i);
    plot(t, true_e(i,:)*180/pi, 'k-', 'LineWidth', 1.5); hold on;
    plot(t, est(i,:)*180/pi, '--', 'LineWidth', 1.2, 'Color', [0.18 0.43 0.55]);
    ylabel([names{i} ' (deg)']);
    xlabel('时间 (s)');
    legend('真值', 'Mahony 估计', 'Location', 'best');
    grid on; xlim([0 T]);
end
sgtitle('Mahony 6 轴姿态解算：真值 vs 估计（含传感器噪声）');

figure('Name', '姿态误差', 'Color', 'w');
plot(t, err(1,:), 'LineWidth', 1.2); hold on;
plot(t, err(2,:), 'LineWidth', 1.2);
plot(t, err(3,:), 'LineWidth', 1.2);
legend('roll 误差', 'pitch 误差', 'yaw 误差', 'Location', 'best');
xlabel('时间 (s)'); ylabel('误差 (deg)'); grid on; xlim([0 T]);
title('姿态估计误差（注意 yaw 缓慢漂移属 6 轴正常现象）');

%% 结论阅读提示
%  - roll/pitch 应快速收敛且误差小（加速度计持续修正）
%  - yaw 会缓慢漂移：6 轴无磁力计航向修正，误差随积分累积。
%    上板若要稳定航向，需加入磁力计(9 轴 Mahony)或 GPS 航向。
%  - 调参：Kp 越大修正越快但越敏感，Ki 消除稳态漂移；按实际传感器噪声调整。
