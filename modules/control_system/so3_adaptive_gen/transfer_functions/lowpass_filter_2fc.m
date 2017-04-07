function obj = lowpass_filter_2fc(fc1,fc2)
obj = make_iter_trans(2);
%tf 1/(s/wc1+1) * 1/(s/wc2+1)
b = [1];
a =  [1/(4*pi*pi*fc1*fc2),(1/(2*pi*fc1)+1/(2*pi*fc2)),1];
end