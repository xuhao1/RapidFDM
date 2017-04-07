function obj = make_filter_obj(fc)
obj = make_iter_trans(1);
[b,a] = butter1st_200hz(fc);
obj.A(1,1:2) = a;
obj.B(1,1:2) = b;
end