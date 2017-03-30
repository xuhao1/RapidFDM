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
%g(1) = 0.5 * (r *psiy-q*psiz) - cot(y)*(-q*psix*psiy+p*psiy*psiy-r*psix*psiz+p*psiz*psiz);

g_by_x = [0 0;0 0];
%g_by_x(1,1) = p*csc(y)^2*psix*psiy^2/y + psiy*(q*cot(y) - q*csc(y)^2*psix^2/y) +...
%    psiz*(r*cot(y) - csc(y)^2*psix*(r*psix-p*psiz)/y);
%g_by_x(1,2) = - psiy^2 *cot(y) - psiz^2*cot(y);
end