function test_quaternion

qspl = eul2quat([8/pi 6/pi+0.001 0]);
qsp = eul2quat([8/pi 6/pi-0.003 -0.001]);
ql = eul2quat([8/pi 6/pi 7/pi]);
quat_err_rov(qsp,ql) - quat_err_rov(qspl,ql)
qeq = quatmultiply( quatinv(qsp),ql);
qe = quat_err_rov(qsp,qspl)
quatrotate(qeq,qe)
quatrotate(ql,qe)
quatrotate(qsp,qe)