function [gr,gp,gy] = so3_external_dynamics(psi,angular_rate)
psix = psi(1);
psiy = psi(2);
psiz = psi(3);
y = norm(psi);
if y < 0.001
    y = 0.001;
end

p = angular_rate(1);
q = angular_rate(2);
r = angular_rate(3);

gp = [0;0];
gp(1) = - 1.*Power(psix,2)*q*cot(y) + psix*(-0.5*r + p*psiy*cot(y)) + psiz*(0.5*p - 1.*psiz*q*cot(y) + psiy*r*cot(y));

gr = [0;0];
gr(1) = - 1.*p*Power(psiy,2)*cot(y) + psiy*(0.5*r + psix*q*cot(y)) + ...
    psiz*(-0.5*q - 1.*p*psiz*cot(y) + psix*r*cot(y));

gy = [0;0];

gy(1) = - 1.*Power(psix,2)*r*cot(y) + psix*(0.5*q + p*psiz*cot(y)) + ...
psiy*(-0.5*p + psiz*q*cot(y) - 1.*psiy*r*cot(y));
end