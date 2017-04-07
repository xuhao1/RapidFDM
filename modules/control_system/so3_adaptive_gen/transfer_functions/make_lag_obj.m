function obj = make_lag_obj(f1,a)
%a>1 lead
%a<1 lag
obj = make_iter_trans(1);
wz = f1 * 2*pi*a;
wp = 2*pi*f1;
b =  [1,-((400.*wp - wp*wz)/(400.*wp + wp*wz))]*wp*(400. + wz)/((400. + wp)*wz);
a = [1,-((400.*wz - wp*wz)/(400.*wz + wp*wz))];
obj.A(1,1:2) = a;
obj.B(1,1:2) = b;
end