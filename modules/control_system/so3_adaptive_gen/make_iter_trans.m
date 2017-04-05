function obj = make_iter_trans(n)
obj = struct('A',zeros(1,n+1),'B',zeros(1,n+1),...
    'x',zeros(1,n+1),'y',zeros(1,n+1),'n',n,'inited',false);
end