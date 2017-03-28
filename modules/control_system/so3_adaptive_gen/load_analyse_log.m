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

ax1 = subplot(4,1,1);
plot(ax1,t,x_pre,t,xdot_pre,t,r)
legend(ax1,'x','xdot','r')
title(ax1,'x state&r')

ax2 = subplot(4,1,2);
plot(ax2,t,err0,t,err1)
legend(ax2,'err[0]','err[1]')
title(ax2,'err state')

ax3 = subplot(4,1,3);
plot(ax3,t,omega_pre,t,theta0_pre,t,theta1_pre,t,sigma_pre)
legend(ax3,'omega','theta[0]','theta[1]','sigma')
title(ax3,'Parameter state')

ax4 = subplot(4,1,4);
plot(ax4,t,uarr,t,etaarr)
legend(ax4,'u','eta')
title(ax4,'Output')
end