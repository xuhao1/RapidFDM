function test_filter
fc = 5;
fs = 200;

[b,a] = butter(1,fc/(fs/2));
filter_obj = make_filter_obj(5);
lead_obj = make_lead_obj(1,10);
lag_obj = make_lag_obj(1,10);

% ax1 = subplot(2,1,1);
% H = tf(lead_obj.B,lead_obj.A,0.005);
% bode(ax1,H)
% grid on
% title(ax1,'Lead')
% ax2 = subplot(2,1,2);
% H = tf(lag_obj.B,lag_obj.A,0.005);
% bode(ax2,H)
% grid on
% title('Lag')
% 
t = linspace(0,100,1000);
x = sin(t*0.1) + 0*rand(size(t));
y_matlab = filter(lead_obj.B,lead_obj.A,x);
y_user = t;

for i = 1:1000
    %[ylead(i),lead_obj] = IterTransform1st(x(i),lead_obj);
    %[ylag(i),lag_obj] = IterTransform1st(x(i),lag_obj);
    [y_user(i),filter_obj] = IterTransform1st(x(i),filter_obj);
end

plot(t,x,t,y_matlab,t,y_user,t,y_matlab-y_user)
legend('Input','MatlabFilter','ManualFilter','FilterDelta')
% figure
% plot(t,x,t,ylead,t,ylag)
% legend('Input Data','lead','lag')
% title('test lead')
% figure
% plot(t,x,t,yy)
% title('test lead2')
% legend('Input Data','lead')
end