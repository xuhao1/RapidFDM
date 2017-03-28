function ctrl = init_adaptive_controller()
filter_obj = struct('A',[0,0],'B',[0,0],'x',[0,0],'y',[0,0],'inited',false);
ctrl = struct('t',0,'x',[0; 0; 1.0; 0.0; 0.0; 0],'Gamma',0,...
'P',[0 0; 0 0],'Am',[0 0;0 0],'u',0,'b',[0;0],'kg',0,'u_filter',filter_obj,...
'inited',false,'err',[0;0],'eta',0,'r',0);
%coder.cstructname(ctrl, 'AdaptiveCtrlT');
end