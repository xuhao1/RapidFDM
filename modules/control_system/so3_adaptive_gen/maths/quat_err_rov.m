function err_rov = quat_err_rov(quat_sp,quat)
quat_err = quatmultiply( quatinv(quat_sp),quat);
err_rov = quat2rov(quat_err);
return;
err_rov = [0,0,0];
%quat = quatnormalize(quat);
%quat_sp = quatnormalize(quat_sp);
ql0 = quat(1);
ql1 = quat(2);
ql2 = quat(3);
ql3 = quat(4);

qsp0 = quat_sp(1);
qsp1 = quat_sp(2);
qsp2 = quat_sp(3);
qsp3 = quat_sp(4);
if (1 - Power(ql0*qsp0 + ql1*qsp1 + ql2*qsp2 + ql3*qsp3,2)) < 1e-6
    return
end
angle = warp2pi(2*acos(ql0*qsp0 + ql1*qsp1 + ql2*qsp2 + ql3*qsp3)/sqrt(1 - Power(ql0*qsp0 + ql1*qsp1 + ql2*qsp2 + ql3*qsp3,2)));

err_rov(1) = (ql1*qsp0 - ql0*qsp1 - ql3*qsp2 + ql2*qsp3);
err_rov(2) = (ql2*qsp0 + ql3*qsp1 - ql0*qsp2 - ql1*qsp3);
err_rov(3) = (ql3*qsp0 - ql2*qsp1 + ql1*qsp2 - ql0*qsp3);

err_rov = err_rov/norm(err_rov)*angle;

end