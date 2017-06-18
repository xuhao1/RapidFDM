function Yout = ode4user(odefun,tspan,y0,steps,varargin)

hi = (tspan(2) - tspan(1))/steps;

neq = length(y0);
F = zeros(neq,4);

Y = y0;
for i = 2:steps+1
  ti = tspan(i-1);
  yi = Y;
  F(:,1) = odefun(ti,yi,varargin{:});
  F(:,2) = odefun(ti+0.5*hi,yi+0.5*hi*F(:,1),varargin{:});
  F(:,3) = odefun(ti+0.5*hi,yi+0.5*hi*F(:,2),varargin{:});  
  F(:,4) = odefun(tspan(i),yi+hi*F(:,3),varargin{:});
  Y = yi + (hi/6)*(F(:,1) + 2*F(:,2) + 2*F(:,3) + F(:,4));
end
Yout = Y;
end