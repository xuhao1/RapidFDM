function yawrate = get_yaw_rate(sys_state)
[phi,theta,~] = get_sys_euler(sys_state);
p = sys_state.angular_rate(1);
q = sys_state.angular_rate(2);
r = sys_state.angular_rate(3);
yawrate = (q*sin(phi)+r*cos(phi)) / cos(theta);
%yawrate = p+tan(theta)*(q*sin(phi)+r*cos(phi));
end