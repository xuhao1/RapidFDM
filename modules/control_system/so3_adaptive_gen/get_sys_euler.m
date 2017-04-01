function [phi,theta,psi] = get_sys_euler(sys_state)
eul = quat2eul(sys_state.quat);
phi = eul(3);
theta = eul(2);
psi = eul(1);
end