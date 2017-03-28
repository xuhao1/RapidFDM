function AdaptiveCtrlRes = L1ControlLaw2nd(dt,AdaptiveCtrl,SysState,r)
%AdaptiveCtrl
AdaptiveCtrlRes = AdaptiveCtrl;
%coder.cstructname(AdaptiveCtrlRes, 'AdaptiveSysT');
omega = AdaptiveCtrl.x(3);
theta = AdaptiveCtrl.x(4:5);
sigma = AdaptiveCtrl.x(6);
kg = AdaptiveCtrl.kg;
eta = float_constrain((kg*r - sigma - theta'* AdaptiveCtrl.x(1:2))/omega,-1,1);
[u,AdaptiveCtrlRes.u_filter] = IterTransform1st(eta,AdaptiveCtrl.u_filter);
AdaptiveCtrlRes.u  = u;
AdaptiveCtrlRes.eta = eta;
end