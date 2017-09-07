function xnew = Servoffunc(x,u,dt)
q = x(1);
ys = x(2);
kd = x(3);
wc = x(4);
wp = x(5);
sigma = x(6);
deltax = [wp*(ys+sigma)-kd*q;wc*u-wc*ys;0;0;0;0];
xnew = x + deltax*dt;
end
