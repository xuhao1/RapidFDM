function [g,g_by_x] = roll_external_dynamics(psi,angular_rate)
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

g = [0;0];


g_by_x = [0 0;0 0];

g(1) = - 1.*p*Power(psiy,2)*cot(y) + psiy*(0.5*r + psix*q*cot(y)) + ...
    psiz*(-0.5*q - 1.*p*psiz*cot(y) + psix*r*cot(y));
end