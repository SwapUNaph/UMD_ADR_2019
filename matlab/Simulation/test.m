
n = n3;

x2 = x3 - n*cos(hdg3);
y2 = y3 - n*sin(hdg3);

ds = (-2:0.01:2);

x = x0*(1-ds).^3+3*ds.*(1-ds).^2*x1+3*ds.^2.*(1-ds)*x2+ds.^3*x3;
y = y0*(1-ds).^3+3*ds.*(1-ds).^2*y1+3*ds.^2.*(1-ds)*y2+ds.^3*y3;

plot(x,y,'--')
hold on
plot(x0,y0,'xk')
plot(x3,y3,'xk')