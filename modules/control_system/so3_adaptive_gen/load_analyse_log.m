function load_analyse_log(filename)
S = load(strcat('/var/log/rapidfdm/' , filename));
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
sigma_1nd_pre = data(:,8);

err0 = data(:,9)*180/pi;
err1 = data(:,10)*180/pi;

uarr = data(:,11);
etaarr = data(:,12);
rdot = data(:,13)*180/pi;
fw = data(:,14)*180/pi;

quat = data(:,15:18);
quat_sp = data(:,19:22);

time_used = data(:,23);
outarr = data(:,24);
xdot_ctrl_use = data(:,25)*180/pi;
uact = data(:,26);
uest = data(:,27);
xdot_noise = data(:,28)*180/pi;
xdot_real = data(:,29)*180/pi;
x = x_pre-err0;

roll =  data(:,30)*180/pi;
roll_sp = data(:,31)*180/pi;

P_act_est_ys = data(:,32);

figure
subplot_size_x = 7;
ax = subplot(subplot_size_x,1,1);
plot(ax,t,x_pre,t,xdot_pre,t,xdot_ctrl_use)
legend(ax,'xpre','xdot_(pre)','xdotuse')
title(ax,'x pre state')

ax = subplot(subplot_size_x,1,2);
plot(ax,t,err0,t,err1)
legend(ax,'err[0]','err[1]')
title(ax,'err state')

ax = subplot(subplot_size_x,1,3);
plot(ax,t,omega_pre,t,sigma_1nd_pre,t,sigma_pre)
legend(ax,'omega','sigma_1nd','sigma')
title(ax,'Parameter state')

ax = subplot(subplot_size_x,1,4);
u_by_x = theta0_pre.*x/180*pi./omega_pre;
u_by_x = arrayfun(@(x) float_constrain(x,-2,2),u_by_x);
u_by_xdot = theta1_pre.*xdot_ctrl_use./omega_pre/180*pi;
u_by_xdot = arrayfun(@(x) float_constrain(x,-2,2),u_by_xdot);
plot(ax,t,uarr,t,etaarr)
legend(ax,'u','eta')
title(ax,'Output')

ax = subplot(subplot_size_x,1,5);
plot(ax,t,x,t,xdot_ctrl_use,t,rdot,t,xdot_real)
legend(ax,'x','xdotctrl','rdot','xdotreal')
title(ax,'x state&r')
%t,[diff(x_pre)*200;0]


ax = subplot(subplot_size_x,1,6);
plot(ax,t,theta0_pre,t,theta0_pre./omega_pre,t,ones(size(t))*2.0)
legend(ax,'the0','the0/omg','Km0')

ax = subplot(subplot_size_x,1,7);
plot(ax,t,theta1_pre,t,ones(size(t))*0.869)
legend(ax,'the1','Km1')

figure
plot(t,uact,t,uest)
legend('ACT','EST')
end