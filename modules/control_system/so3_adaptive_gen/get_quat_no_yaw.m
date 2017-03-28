function ans_quat = get_quat_no_yaw(quat)
eul = quat2eul(quat);
YawAngle = eul(1);
%[YawAngle,~,~] = quat2angle(quat,'ZYX');
invYawquat = eul2quat([-YawAngle 0 0]);
ans_quat = quatmultiply(invYawquat,quat);
end