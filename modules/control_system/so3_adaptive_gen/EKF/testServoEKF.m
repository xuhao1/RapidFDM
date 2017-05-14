function testServoEKF(parr_noise,uarr,tarr,uact,uest,preal)
%pnoise = awgn(parr/180*pi,10);
[n,~] = size(parr_noise);
obj = InitActuatorEstimator();
ysarr = zeros(n,1);
kdarr = zeros(n,1);
wcarr = zeros(n,1);
wparr = zeros(n,1);
pmarr = zeros(n,1);
zarr = zeros(n,1);
yresarr = zeros(n,1);
Pwc = zeros(n,1);
Pkd = zeros(n,1);
fdarr = zeros(n,1);
sigmarr = zeros(n,1);

lag_filter = make_lag_obj(10,20);
lag = zeros(n,1);
for i=1:n
    [obj,ysarr(i)] = IterActuatorEst(obj,uarr(i),0.005);
    obj =  EKFUpdate(obj,parr_noise(i)/180*pi,@Servohfunc);
    pmarr(i) = obj.x(1)*180/pi;
    [lag(i),lag_filter] = IterTransform(parr_noise(i),lag_filter);
    %ysarr(i) = obj.x(2);
    kdarr(i) = obj.x(3);
    wcarr(i) = obj.x(4);
    wparr(i) = obj.x(5);
    yresarr(i) = obj.yres;
    Pwc(i) = obj.P(4,4);
    Pkd(i) = obj.P(3,3);
    Psigma(i) = obj.P(6,6);
    sigmarr(i) = obj.x(6);
    zarr(i) = obj.z;
    wp = wparr(i);
    wc = wcarr(i);
    kq = kdarr(i);
    wn = sqrt(wp*wc);
    eta = (wc + kq)/(2*wn);
    fdarr(i) = wn*sqrt(1-eta*eta)/(2*pi);
end
figure
ps = 7;
ax1 = subplot(ps,1,1);
plot(ax1,tarr,preal,tarr,pmarr,tarr,lag);
legend(ax1,'real','est','filter')
title(ax1,'angular velocity');

ax2 = subplot(ps,1,2);
plot(ax2,tarr,uarr,tarr,ysarr,tarr,uact,tarr,uest);
grid on
legend(ax2,'SP','EST','ACT','ESTCON');
title(ax2,'Servo Value');

ax = subplot(ps,1,3);
%tarr,(uact-uarr).*(uact-uarr)
plot(ax,tarr,(uact-ysarr).*(uact-ysarr),tarr,(uact-uest).*(uact-uest))
legend(ax,'EstErr','EstErrCon');
title(ax,'ServoERR');

ax = subplot(ps,1,4);
plot(ax,tarr,kdarr./wparr);
legend(ax,'Angular Damp Theta');

ax = subplot(ps,1,5);
plot(ax,tarr,wcarr/(2*pi),tarr,fdarr);
title(ax,'Freq');
legend(ax,'fc','fnatur');
grid on

ax = subplot(ps,1,6);
plot(ax,tarr,yresarr,tarr,sigmarr);
legend(ax,'yres','Sigma');

ax = subplot(ps,1,7);
plot(ax,tarr,wparr/42)
legend(ax,'WP');
%plot(ax,tarr,(preal-pmarr).*(preal-pmarr),tarr,(preal-lag).*(preal-lag));
%legend(ax,'EKF','LAG');
end