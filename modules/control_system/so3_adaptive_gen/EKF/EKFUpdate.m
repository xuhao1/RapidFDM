function obj = EKFUpdate(obj,z,h)
    [n,~] = size(obj.x);
    y = z - h(obj.x);
    S = obj.H*obj.P*obj.H' + obj.R;
    K = obj.P*obj.H'/S;
    obj.x = obj.x + K * y;
    obj.P = (eye(n) - K*obj.H)*obj.P;
    obj.yres = y;
    obj.z = z;
end