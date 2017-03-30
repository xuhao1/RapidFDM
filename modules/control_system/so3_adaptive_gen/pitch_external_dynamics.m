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
%g =[ -q*cot(y)*psix^2+psix*(-0.5*r+p*cot(y)*psiy)+...
%    psiz*(0.5*p+r*cot(y)*psiy-q*cot(y)*psiz);0];
g_by_x = [0 0;0 0];

end