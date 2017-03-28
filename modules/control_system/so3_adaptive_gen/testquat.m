function testquat
q1 = euler2quat([pi/6,0,0]);
q2 = euler2quat([pi/6,pi/6,0]);
q3 = euler2quat([0,pi/2,pi/6]);    
q4 = euler2quat([0,pi/2,0]);

rotationvector1 = quat2rov(q1);

rotationvector2 = quat2rov(q2);

quat3 = quatmultiply(quatinv(q1),q2);

rotat3 = quat2rov(quat3);

rov3 = quat2rov(q3)
rov4 = quat2rov(q4)

quat2rov( quatmultiply(quatinv(q4),q3))

end