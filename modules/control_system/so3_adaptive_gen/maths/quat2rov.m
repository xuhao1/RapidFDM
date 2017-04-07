function rov = quat2rov(q)
%coder.varsize('q', [4 1]);
axa =  quat2axang(q);
ax = axa(1:3);
angle = axa(4);
rov =  ax*angle;
end