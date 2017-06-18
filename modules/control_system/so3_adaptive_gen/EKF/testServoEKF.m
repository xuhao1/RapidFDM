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

lag = zeros(n,1);
for i=1:n
    [obj,ysarr(i)] = IterActuatorEst(obj,parr_noise(i)/180*pi,uarr(i),0.005);
    pmarr(i) = obj.x(1)*180/pi;
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
ps = 3;
ax1 = subplot(ps,1,1);
plot(ax1,tarr,pmarr,tarr,parr_noise);
legend(ax1,'est','noise')
title(ax1,'angular velocity');

ax2 = subplot(ps,1,2);
plot(ax2,tarr,uarr,tarr,uact,tarr,uest,tarr,(uact-uest).*(uact-uest));
grid on
legend(ax2,'SP','ACT','EST','ERR');
title(ax2,'Servo Value');

% 
% ax = subplot(ps,1,3);
% %tarr,(uact-uarr).*(uact-uarr)
% plot(ax,tarr,(uact-ysarr).*(uact-ysarr),tarr,(uact-uest).*(uact-uest))
% legend(ax,'EstErr','EstErrCon');
% title(ax,'ServoERR');

ax = subplot(ps,1,3);
plot(ax,tarr,kdarr);
legend(ax,'Angular Damp Theta');
return
% ax = subplot(ps,1,5);
% plot(ax,tarr,wcarr/(2*pi),tarr,fdarr);
% title(ax,'Freq');
% legend(ax,'fc','fnatur');
% grid on
%return;
ax = subplot(ps,1,4);
plot(ax,tarr,yresarr,tarr,sigmarr);
legend(ax,'yres','Sigma');

ax = subplot(ps,1,5);
plot(ax,tarr,wparr,tarr,wcarr)
legend(ax,'WP','WC');
%plot(ax,tarr,(preal-pmarr).*(preal-pmarr),tarr,(preal-lag).*(preal-lag));
%legend(ax,'EKF','LAG');
disp(['EstErrSum   ',num2str(norm(uact-ysarr)),'  EstErrConSum   ',num2str(norm(uact-uest))])
end