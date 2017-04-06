function obj = make_iter_trans(n)
obj = struct('A',zeros(1,10),'B',zeros(1,10),...
    'x',zeros(1,19),'y',zeros(1,10),'n',n,'inited',false);
end