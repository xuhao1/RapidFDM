function err_rov = quat_err_rov(quat_sp,quat)
quat_err = quatmultiply( quatinv(quat_sp),quat);
err_rov = quat2rov(quat_err);
end