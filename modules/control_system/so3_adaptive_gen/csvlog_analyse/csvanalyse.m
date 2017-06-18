function csvanalyse(filename)
    M = csvread(filename);
    x = M(:,3:9);
    err = M(:,10:11);
    x_real = M(:,12:13);
    eta = M(:,14);
    u = M(:,15);
    est_act = M(:,16);
    [N,~] = size(x);
    ticks = 1:N;
    
    subplot_size_x = 5;
    ax = subplot(subplot_size_x,1,1);
    plot(ticks,x(:,1),ticks,x(:,2));
    legend(ax,'xpre','xdot_(pre)')
    
    ax = subplot(subplot_size_x,1,2);
    plot(ticks,x_real(:,1),ticks,x_real(:,2));
    legend(ax,'x','xdot')
    
    ax = subplot(subplot_size_x,1,3);
    plot(ticks,x(:,3),ticks,x(:,4),ticks,x(:,5));
    legend(ax,'omega','the0','the1')
    
    ax = subplot(subplot_size_x,1,4);
    plot(ticks,err(:,1),ticks,x(:,2));
    legend(ax,'err0','err1')
    
    ax = subplot(subplot_size_x,1,5);
    plot(ticks,eta,ticks,u,ticks,est_act);
    legend(ax,'eta','u','est_act')
end