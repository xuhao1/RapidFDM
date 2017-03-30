function test_filter
fc = 5;
fs = 200;

[b,a] = butter(1,fc/(fs/2));
filter_obj = make_filter_obj(200,5);

t = linspace(-pi,pi,1000);
x = sin(t) + 0.25*rand(size(t));
y = filter(b,a,x);

%plot(t,x)
%hold on
%plot(t,y)
%hold on
ynew = t;
for i = 1:1000
    [ynew(i),filter_obj] = IterTransform1st(x(i),filter_obj);
end

%plot(t,ynew-y)
%legend('Input Data','Filtered Data','User Filter')
%title('First Row')

fc = 5;
H = tf([1],[1/(2*pi*fc) 1]);
Hd = c2d(H,1,'tustin')

[b,a] = butter(1,fc/(fs/2))
[b,a] = butter1st_200hz(fc)

end
