function obj = make_lead_obj(f1,a)
obj = make_iter_trans(1);
wz = f1 * 2*pi;
wp = f1 * 2*pi*a;
b =  [1,-((400.*wp - wp*wz)/(400.*wp + wp*wz))]*wp*(400. + wz)/((400. + wp)*wz);
a = [1,-((400.*wz - wp*wz)/(400.*wz + wp*wz))];
obj.A = a;
obj.B = b;
end