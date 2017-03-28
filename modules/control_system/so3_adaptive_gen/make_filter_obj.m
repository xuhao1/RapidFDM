function obj = make_filter_obj(fs,fc)
[b,a] = butter(1,fc/(fs/2));
obj = struct('A',a,'B',b,'x',[0,0],'y',[0,0],'inited',false);
end