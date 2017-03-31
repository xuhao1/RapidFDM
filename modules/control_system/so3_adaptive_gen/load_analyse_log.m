function load_analyse_log(filename)
S = load(strcat('/Users/xuhao/Develop/FixedwingProj/RapidFDM/build/release/log/' , filename));
names = fieldnames(S);
[count , ~ ]= size(names);
data = getfield(S,names{1});
for i=2:count
    names{i};
    data = vertcat(data,getfield(S,names{i}));
end
size(data)
t = data(:,1);
x_pre = data(:,2)*180/pi;
xdot_pre = data(:,3)*180/pi;

omega_pre = data(:,4);
theta0_pre = data(:,5);
theta1_pre = data(:,6);

sigma_pre = data(:,7);

err0 = data(:,8)*180/pi;
err1 = data(:,9)*180/pi;

uarr = data(:,10);
etaarr = data(:,11);
r = data(:,12)*180/pi;
fw = data(:,13)*180/pi;

figure
ax1 = subplot(6,1,1);
plot(ax1,t,x_pre,t,xdot_pre)
legend(ax1,'x','xdot_(pre)')
title(ax1,'x pre state')

ax2 = subplot(6,1,2);
plot(ax2,t,err0,t,err1)
legend(ax2,'err[0]','err[1]')
title(ax2,'err state')

ax3 = subplot(6,1,3);
plot(ax3,t,omega_pre,t,theta0_pre,t,theta1_pre,t,sigma_pre)
legend(ax3,'omega','theta[0]','theta[1]','sigma')
title(ax3,'Parameter state')

ax4 = subplot(6,1,4);
plot(ax4,t,uarr,t,etaarr)
legend(ax4,'u','eta')
title(ax4,'Output')

ax5 = subplot(6,1,5);
plot(ax5,t,x_pre-err0,t,xdot_pre - err1,t,r,t,fw)
legend(ax5,'x','xdot','r','feedforward')
title(ax5,'x state&r')

ax6 = subplot(6,1,6);
plot(ax6,t,fw,t,[diff(x_pre)*200;0])
legend(ax6,'feedforward','x pre diff')
title(ax6,'diff')

pitchctrl = L1ControllerUpdateParams(7,0.655,32,7,1000);
errarr = [err0,err1];
P = pitchctrl.P;
b = pitchctrl.b;
Gamma = pitchctrl.Gamma;
xarr = [x_pre,xdot_pre];
grid on;
[len,~] = size(errarr);
for i=1:len
    err = errarr(i,:)';
    x = xarr(i,:)';
    u = uarr(i);
    dparam(i,1) = - Gamma*err'*P*b*u*0.005;
    dparam(i,2:3) = - (Gamma*err'*P*b*x)'*0.005;
    dparam(i,4) = - Gamma*err'*P*b*0.005;
end
ax7 = subplot(7,1,7);
plot(ax7,t,dparam(:,1),t,dparam(:,2),t,dparam(:,3),t,dparam(:,4))
legend(ax7,'omega','th0','th1','sigma')
title(ax7,'diff of param')

end