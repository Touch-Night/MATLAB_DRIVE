clc;
clear;
close all;
clear L;
%% 使用Robotics Toolbox进行的仿真
%% 参数定义
a1 = 150; alpha1 = -pi/2; d1 = 0; theta1 = 0;
a2 = 760; alpha2 = 0; d2 = 0; theta2 = -pi/2;
a3 = 220; alpha3 = -pi/2; d3 = 0; theta3 = 0;
a4 = 0; alpha4 = pi/2; d4 = 970; theta4 = 0;
a5 = 0; alpha5 = -pi/2; d5 = 0; theta5 = 0;
a6 = 0; alpha6 = 0; d6 = 0; theta6 = 0;
%% 创建模型
L(1) = Link([theta1, d1, a1, alpha1, 0]);
L(2) = Link([theta2, d2, a2, alpha2, 0]);
L(3) = Link([theta3, d3, a3, alpha3, 0]);
L(4) = Link([theta4, d4, a4, alpha4, 0]);
L(5) = Link([theta5, d5, a5, alpha5, 0]);
L(6) = Link([theta6, d6, a6, alpha6, 0]);
%% 定义关节范围
L(1).qlim = [-180/180*pi, 180/180*pi];
L(2).qlim = [-110/180*pi, 155/180*pi];
L(3).qlim = [-165/180*pi, 220/180*pi];
L(4).qlim = [-150/180*pi, 150/180*pi];
L(5).qlim = [-45/180*pi, 180/180*pi];
L(6).qlim = [-200/180*pi, 200/180*pi];
%% 显示机械臂
Motoman = SerialLink(L, 'name', 'Motoman');
theta = [0 -pi/2 0 0 0 0]; % 初始姿态
figure(1);
Motoman.plot(theta);
%Motoman.teach;
%% 正向和逆向运动学
forward_theta = [0 -pi/3 0 pi/4 pi/4 pi/3]; % 用于正向运动学计算的姿态
forward = Motoman.fkine(forward_theta);
inverse = ikine(Motoman, forward);
Motoman.plot(inverse);
%% 轨迹规划
% 给定末端执行器的初始位姿
p1 =[
-1 0 0 -500;
0 1 0 -800;
0 0 1 0;
0 0	0 1
];
% 给定末端执行器的终止位姿
p2 = [
0 0 1 800;
0 -1 0 400;
1 0 0 400;
0 0	0 1  
];
% 求解初始关节角和终止关节角
p1_ang = Motoman.ikine(p1);
p2_ang = Motoman.ikine(p2);
% 五次多项式计算轨迹
step = 100;
[q, qd, qdd] = jtraj(p1_ang, p2_ang, step);
% 显示机器人姿态随时间的变化
figure(1);
Motoman.plot(q);
hold on;
grid on;
title('末端执行器轨迹');
for i=1:step
position = Motoman.fkine(q(i,:));
plot3(position.t(1),position.t(2),position.t(3),'b.','MarkerSize',5);
end

%显示机器人关节运动状态
figure(2);
subplot(3,2,1);
i=1:6;
plot(rad2deg(q(:,i)));
title('关节角度');
grid on;
subplot(3,2,2);
i=1:6;
plot(rad2deg(qd(:,i)));
title('关节角速度');
grid on;
subplot(3,2,3);
i=1:6;
plot(rad2deg(qdd(:,i)));
title('关节角加速度');
grid on;

%显示末端执行器的位置
subplot(3,2,4);
hold on
grid on
title('末端执行器轨迹');
for i=1:step
position = Motoman.fkine(q(i,:));
plot3(position.t(1),position.t(2),position.t(3),'b.','MarkerSize',5);
end
subplot(3,2,5);
hold on
grid on
title('末端执行器速度');
vel = zeros(step,6);
vel_velocity = zeros(step,1);
vel_angular_velocity = zeros(step,1);
for i=1:step
vel(i,:) = Motoman.jacob0(q(i,:))*qd(i,:)';
vel_velocity(i) = sqrt(vel(i,1)^2+vel(i,2)^2+vel(i,3)^2);
vel_angular_velocity(i) = sqrt(vel(i,4)^2+vel(i,5)^2+vel(i,3)^6);
end
x = linspace(1,step,step);
plot(x,vel_velocity);

subplot(3,2,6);
hold on
grid on
title('末端执行器角速度');
x = linspace(1,step,step);
plot(x,vel_angular_velocity);