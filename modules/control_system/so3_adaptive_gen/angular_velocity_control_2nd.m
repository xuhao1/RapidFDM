function [obj,u] = angular_velocity_control_2nd(obj,dt,sys,r)
%we control roll
xd = sys.angular_rate_dot(1);
%[xd,obj.g_filter] = IterTransform1st(sys.angular_rate_dot(1),obj.g_filter);
x_real = [sys.angular_rate(1);xd];
[obj,u] = L1AdaptiveControl2nd(dt,obj,x_real,r);
end