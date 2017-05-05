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
r = data(:,13)*180/pi;
fw = data(:,14)*180/pi;

quat = data(:,15:18);
quat_sp = data(:,19:22);

time_used = data(:,23);
outarr = data(:,24);
xdot = data(:,25)*180/pi;
uact = data(:,26);
uest = data(:,27);
x = x_pre-err0;

figure
subplot_size_x = 8;
ax1 = subplot(subplot_size_x,1,1);
plot(ax1,t,x_pre,t,xdot_pre)
legend(ax1,'x','xdot_(pre)')
title(ax1,'x pre state')

ax2 = subplot(subplot_size_x,1,2);
plot(ax2,t,err0,t,err1)
legend(ax2,'err[0]','err[1]')
title(ax2,'err state')

ax3 = subplot(subplot_size_x,1,3);
plot(ax3,t,omega_pre,t,theta0_pre,t,theta1_pre,t,sigma_pre,t,sigma_1nd_pre)
legend(ax3,'omega','theta[0]','theta[1]','sigma','sigma_1nd')
title(ax3,'Parameter state')

ax4 = subplot(subplot_size_x,1,4);
plot(ax4,t,uarr,t,etaarr)
legend(ax4,'u','eta')
title(ax4,'Output')

ax5 = subplot(subplot_size_x,1,5);
plot(ax5,t,x,t,r)
legend(ax5,'x','r')
title(ax5,'x state&r')
%t,[diff(x_pre)*200;0]
ax6 = subplot(subplot_size_x,1,6);
fw_tmp = arrayfun(@(x) float_constrain(x,-20,40),fw);
xdot_tmp = arrayfun(@(x) float_constrain(x,-40,20),xdot);
plot(ax6,t,-fw_tmp,t,xdot)
legend(ax6,'feedforward','xdot')
title(ax6,'diff')

pitchctrl = init_adaptive_controller(0,1); 
pitchctrl = L1ControllerUpdateParams(pitchctrl,7,0.8,32,1000,3,2.5,3);
errarr = [err0,err1];
P = pitchctrl.P;
b = pitchctrl.b;
Gamma = pitchctrl.Gamma;
xarr = [x_pre,xdot_pre];
grid on;
[len,~] = size(errarr);

ax7 = subplot(7,1,7);
fw_u = (pitchctrl.kg_rate * - fw) ./ omega_pre;
pdu = (pitchctrl.Am(2,2) / pitchctrl.b(2) * xdot)./omega_pre;
plot(ax7,t,-fw_u,t,pdu)
legend(ax7,'fwu','fu')
title(ax7,'Feedforward on Axis')
grid on

%figure
%plot(t,time_used)
%legend('time used us')
%title('Time us of Attitude Control')
%grid on
%testServoEKF(xdot,uarr,t)
testServoEKF(xdot,outarr,t,uact,uest)

end