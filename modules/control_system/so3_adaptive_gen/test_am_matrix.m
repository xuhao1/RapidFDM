function test_am_matrix(p,epi)
b2 = 32;
pd = 2*epi*sqrt(p/b2)
Am = [0 1;-p*b2 -pd*b2];
b = [0;b2];
figure
[t1,x1]=ode23(@(t,x) Dynamics2nd(t,x,Am,b),...
    [0,4],[0;0]);
pd = 1;
Am = [0 1;-p*b2 -pd*b2];
[t2,x2]=ode23(@(t,x) Dynamics2nd(t,x,Am,b),...
    [0,4],[0;0]);
plot(t1,x1(:,1),t1,x1(:,2),t2,x2(:,1),t2,x2(:,2))
legend('x1','xdot1','x2','xdot2')
H = tf(b2,[1,pd*b2,p*b2]);
figure
bode(H)
grid on


function  dydt = Dynamics2nd(t,x,Am,b)
dydt = [0;0];
dydt(1:2) = Am*x(1:2) + b;