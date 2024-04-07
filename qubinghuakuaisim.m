syms alpha;
a=10;
b=30;
x=a*cos(alpha)+sqrt(b*b-a*a*sin(alpha)*sin(alpha));
xdot=diff(x);
subplot(1,2,1),fplot(x);
subplot(1,2,2),fplot(xdot);