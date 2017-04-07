function test_filter
fs = 200;


[b,a] = lowpass_filter_2fc(7,20);

tf12 = tf(b,a);
dtf12 = c2d(tf12,0.005,'tustin')

filter_obj = make_iter_trans(2);
bb = [0.02368,0.04736,0.02368];
aa = [1,-1.324,0.4185];
filter_obj.B(1:3) = bb;
filter_obj.A(1:3) = aa;

t = linspace(0,100,1000);
x = sin(t*0.1) + 2*rand(size(t));
y_matlab = filter(bb,aa,x);
y_user = t;

for i = 1:1000
    %[ylead(i),lead_obj] = IterTransform1st(x(i),lead_obj);
    %[ylag(i),lag_obj] = IterTransform1st(x(i),lag_obj);
    [y_user(i),filter_obj] = IterTransform(x(i),filter_obj);
end

plot(t,x,t,y_matlab,t,y_user)
legend('Input','Matlab','ManualFilter')
% figure
% plot(t,x,t,ylead,t,ylag)
% legend('Input Data','lead','lag')
% title('test lead')
% figure
% plot(t,x,t,yy)
% title('test lead2')
% legend('Input Data','lead')
end