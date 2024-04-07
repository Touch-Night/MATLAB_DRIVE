%杆2长度
l=10;
%杆3长度
s=9;
%圆心坐标
circlecenter=[35,0].';
%圆半径
r=8;
%末端执行器角度
syms alpha;
%通过目标确定的末端执行器在基坐标系的位姿
TAO=[[-cos(alpha) sin(alpha);-sin(alpha) -cos(alpha)] circlecenter;0 0 1]
TEA=[1 0 -r;0 1 0;0 0 1]
T3E=[1 0 -s;0 1 0;0 0 1]
T3O=TAO*TEA*T3E
%计算杆二伸出长度d、杆一旋转角度theta1、杆三旋转角度theta3
d=sqrt(T3O(1,3).^2+T3O(2,3).^2)-l
theta1=atan2(T3O(2,3),T3O(1,3))
theta3=atan2(T3O(2,1),T3O(1,1))
fplot(d,[0,2*pi],"-r");
hold on;
fplot(theta1,[0,2*pi],"-g");
hold on;
fplot(theta3,[0,2*pi],"-b");