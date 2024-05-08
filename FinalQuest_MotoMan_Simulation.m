clc;
clear;
close all;

% 仿真部分
%syms t;

% 参数设置
a1 = 200; a2 = 600; a3 = 115; d4 = 770;
nx = 0; ny = 0; nz = 1;
ox = 0; oy = -1; oz = 0;
ax = 1; ay = 0; az = 0;

% 轨迹设置（五角星塔）
t = 0:0.001*pi:5*pi;
px = 100 + (100-25.*sin(5.*t)).*cos(t).*sqrt(1-(t/(5.*pi)).^2);
py = 35 + (100-25.*sin(5.*t)).*sin(t).*sqrt(1-(t/(5.*pi)).^2);
pz = t;

% 计算得到的反解结果
theta1 = atan2(py, px);
s1 = sin(theta1); c1 = cos(theta1);

k = ((sqrt(px.^2+py.^2)-a1).^2+pz.^2-d4.^2-a2.^2-a3.^2)/(2.*a2);
theta3p = atan2(a3, d4)-atan2(k, real(sqrt(a3.^2+d4.^2-k.^2)));
theta3n = atan2(a3, d4)-atan2(k, -real(sqrt(a3.^2+d4.^2-k.^2)));
s3p = sin(theta3p); c3p = cos(theta3p); fprintf("theta3p done\n");
s3n = sin(theta3n); c3n = cos(theta3n); fprintf("theta3n done\n");

theta2p = atan2((a2.*py.*s3p-d4.*py).*s1+a1.*d4-a3.*pz-a2.*c3p.*pz-a1.*a2.*s3p-c1.*d4.*px+a2.*c1.*px.*s3p, ...
    (a3.*py+a2.*c3p.*py).*s1+a3.*c1.*px-d4.*pz-a1.*a2.*c3p-a1.*a3+a2.*pz.*s3p+a2.*c1.*c3p.*px)-theta3p;
theta2n = atan2((a2.*py.*s3n-d4.*py).*s1+a1.*d4-a3.*pz-a2.*c3n.*pz-a1.*a2.*s3n-c1.*d4.*px+a2.*c1.*px.*s3n, ...
    (a3.*py+a2.*c3n.*py).*s1+a3.*c1.*px-d4.*pz-a1.*a2.*c3n-a1.*a3+a2.*pz.*s3n+a2.*c1.*c3n.*px)-theta3n;
s2p = sin(theta2p); c2p = cos(theta2p); fprintf("theta2p done\n");
s2n = sin(theta2n); c2n = cos(theta2n); fprintf("theta2n done\n");
s23p = sin(theta2p+theta3p); c23p = cos(theta2p+theta3p); fprintf("theta23p done\n");
s23n = sin(theta2n+theta3n); c23n = cos(theta2n+theta3n); fprintf("theta23n done\n");

theta4p = atan2(-ax.*s1+ay.*c1, -ax.*c1.*c23p-ay.*s1.*c23p+az.*s23p);
theta4n = atan2(-ax.*s1+ay.*c1, -ax.*c1.*c23n-ay.*s1.*c23n+az.*s23n);
s4p = sin(theta4p); c4p = cos(theta4p); fprintf("theta4p done\n");
s4n = sin(theta4n); c4n = cos(theta4n); fprintf("theta4n done\n");

theta5p = atan2(-(ax.*(s1.*s4p+c1.*c4p.*c23p)+ay.*(s1.*c4p.*c23p-c1.*s4p)-az.*(c4p.*s23p)), ...
    -ax.*c1.*s23p-ay.*s1.*s23p-az.*c23p);
theta5n = atan2(-(ax.*(s1.*s4n+c1.*c4n.*c23n)+ay.*(s1.*c4n.*c23n-c1.*s4n)-az.*(c4n.*s23n)), ...
    -ax.*c1.*s23n-ay.*s1.*s23n-az.*c23n);
s5p = sin(theta5p); c5p = cos(theta5p); fprintf("theta5p done\n");
s5n = sin(theta5n); c5n = cos(theta5n); fprintf("theta5n done\n");

theta6p = atan2(-nx.*(c1.*s4p.*c23p-s1.*c4p)-ny.*(c1.*c4p+s1.*s4p.*c23p)+nz.*s4p.*s23p, ...
    nx.*(s1.*s4p.*c5p-c1.*s5p.*s23p+c1.*c4p.*c5p.*c23p)+ny.*(s1.*c4p.*c5p.*c23p-s1.*s5p.*s23p)-nz.*(s5p.*c23p+c4p.*c5p.*s23p));
theta6n = atan2(-nx.*(c1.*s4n.*c23n-s1.*c4n)-ny.*(c1.*c4n+s1.*s4n.*c23n)+nz.*s4n.*s23n, ...
    nx.*(s1.*s4n.*c5n-c1.*s5n.*s23n+c1.*c4n.*c5n.*c23n)+ny.*(s1.*c4n.*c5n.*c23n-s1.*s5n.*s23n)-nz.*(s5n.*c23n+c4n.*c5n.*s23n));
s6p = sin(theta6p); c6p = cos(theta6p); fprintf("theta6p done\n");
s6n = sin(theta6n); c6n = cos(theta6n); fprintf("theta6n done\n");

% 绘制关节变量图像
% 轨迹三维图
figure(1);
plot3(px, py, pz); grid on;
title("末端执行器轨迹");

% 关节变量图
figure(2);
subplot(3,2,1);
plot(t, rad2deg(theta1)); grid on;
xlabel('t'), ylabel('\theta_{1}');
title('\theta_{1}');

subplot(3,2,2);
plot(t, rad2deg(theta2p), '-b'); grid on;
hold on;
plot(t, rad2deg(theta2n), '-r'); grid on;
xlabel('t'), ylabel('\theta_{2}');
title('\theta_{2}');

subplot(3,2,3);
plot(t, rad2deg(theta3p), '-b'); grid on;
hold on;
plot(t, rad2deg(theta3n), '-r'); grid on;
xlabel('t'), ylabel('\theta_{3}');
title('\theta_{3}');

subplot(3,2,4);
plot(t, rad2deg(theta4p), '-b'); grid on;
hold on;
plot(t, rad2deg(theta4n), '-r'); grid on;
xlabel('t'), ylabel('\theta_{4}');
title('\theta_{4}');

subplot(3,2,5);
plot(t, rad2deg(theta5p), '-b'); grid on;
hold on;
plot(t, rad2deg(theta5n), '-r'); grid on;
xlabel('t'), ylabel('\theta_{5}');
title('\theta_{5}');

subplot(3,2,6);
plot(t, rad2deg(theta6p), '-b'); grid on;
hold on;
plot(t, rad2deg(theta6n), '-r'); grid on;
xlabel('t'), ylabel('\theta_{6}');
title('\theta_{6}');