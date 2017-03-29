function [obj,u] = L1AdaptiveControl2nd(dt,obj,x_real,DeltaSpX)
%coder.cstructname(RollCtrl, 'AdaptiveCtrlT');
%coder.cstructname(SysState, 'AdaptiveSysT');
r = 0;
obj.r = r;
obj.x(1) = DeltaSpX + obj.x(1);
if not(obj.inited)
    %res = init_adaptive_controller();
    obj.x(1) = x_real(1);
    obj.x(2) = x_real(2);
    obj.inited = true;
    u = 0;
return
end

x0 = obj.x;
Ad = obj.Am + obj.g_by_x;

obj.err = x_real(1:2) - obj.x(1:2);

opts_1 = odeset('MaxStep',4);
[~,x]=ode23(@(t,x) L1Updater(t,x,Ad,obj.b,obj.P,obj.Gamma,obj.u,obj.g,x_real),...
    [obj.t,obj.t + dt],x0,opts_1);

obj = L1ControlLaw2nd(dt,obj,r);
obj.x = ctrl_x_constrain(x(end,1:6)');
obj.t = obj.t + dt;
u = obj.u;