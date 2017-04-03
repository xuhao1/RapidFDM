function [g,g_by_x] =  pitch_external_dynamics(psi,angular_rate)
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
g(1) = - 1.*Power(psix,2)*q*cot(y) + psix*(-0.5*r + p*psiy*cot(y)) + psiz*(0.5*p - 1.*psiz*q*cot(y) + psiy*r*cot(y));
g_by_x = [0 0;0 0];

end