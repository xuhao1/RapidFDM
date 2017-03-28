function testquat
q1 = euler2quat([pi/6,0,0]);
q2 = euler2quat([pi/6,pi/6,0]);
    
rotationvector1 = quat2rov(q1)

rotationvector2 = quat2rov(q2)

quat3 = quatmultiply(quatinv(q1),q2)

rotat3 = quat2rov(quat3)
end