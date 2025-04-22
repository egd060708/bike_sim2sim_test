clc
g = 9.8;
h = 0.088;
v = 0.634;
w = 0.167;
b = 0.055;
% 重力加速度
% 自行车质心的高度
% 自行车的行驶速度
% 前后轮与地面接触点之间的距离
% 后轮的着地点与重心在地面上的投影间的距离
A_21 = g/h;
A_23 = -v^2/(w*h);
B_21 = -(b*v)/(w*h);
A = [0 1 0; A_21 0 A_23; 0 0 0];
B = [0; B_21; 1];
C = [1 0 0; 0 0 1];
D = 0;
Ts = 0.020;
% 采样间隔
t = 0:Ts:4;
u = zeros(size(t));
[G,H] = c2d(A,B,Ts);
x0 = [0.0873; 0; 0];
% 将连续系统变为离散系统
% 设定系统的初始状态
Tc = ctrb(G,H);
if (rank(Tc)==3)
    fprintf('此系统是可控的！\n');
    Q = [300 0 0; ...
         0 0 0; ...
         0 0 300];
    R = 1;
    K = dlqr(G,H,Q,R);
    disp(K)
    % Q 矩阵
    % R 矩阵
    % 计算状态反馈系数（自行查看计算后的数值）
    G2 = G-H*K;
    y = dlsim(G2,H,C,D,u,x0);
    subplot(2,1,1)
    plot(t,y(:,1),'b.-','LineWidth',1.5);
    xlabel('Time(s)');
    ylabel('\phi(rad)');
    grid on
    subplot(2,1,2)
    plot(t,y(:,2),'b.-','LineWidth',1.5);
    xlabel('Time(s)');
    ylabel('\delta(rad)');
    grid on
end