function csvanalyse(folder)
    system(['./csvlog_analyse/process_log.sh ',folder,' ./temp_data'])
    M = csvread(strcat('./temp_data', '/log001_l1_adaptive_roll_0.csv'));
    SENSORRAW = csvread(strcat('./temp_data' , '/log001_sensor_combined_0.csv'));
    sensor_time = (SENSORRAW(:,1) - M(1,1))/1000000;
    sensor_gyro0 = SENSORRAW(:,2);
    [~,cols] = size(M);
    x = M(:,3:9);
    err = M(:,10:11);
    x_real = M(:,12:13);
    eta = M(:,14);
    u = M(:,15);
    est_act = M(:,18);
    if cols > 16
        act_est = M(:,17:22);
        est_damp = M(:,19);
    end
    est_wc = act_est(:,4);
    [N,~] = size(x);
    ticks = (M(:,1) - M(1,1))/1000000;
    %ticks = (1:N)/N;
    subplot_size_x = 4;
    figure
    ax = subplot(subplot_size_x,1,1);
    plot(ticks,x(:,1),ticks,x(:,2),ticks,x_real(:,1),ticks,x_real(:,2));
    legend(ax,'xpre','xdot_(pre)','xreal0','xreal1')
    
    ax = subplot(subplot_size_x,1,2);
    plot(ticks,x(:,3),ticks,x(:,4),ticks,x(:,5),ticks,x(:,6));
    legend(ax,'omega','the0','the1','sigma')
    
    ax = subplot(subplot_size_x,1,3);
    plot(ticks,err(:,1),ticks,err(:,2));
    legend(ax,'err0','err1')
    
    ax = subplot(subplot_size_x,1,4);
    plot(ticks,eta,ticks,u,ticks,est_act);
    legend(ax,'eta','out','est_act')
   
%     figure
%     plot(ticks,x_real(:,2),sensor_time,sensor_gyro0)
%     legend('ctrlRollRate','SensorGyro0')
    figure
    plot(ticks,x(:,3),ticks,est_damp/64./x(:,3))
    legend('omega','est_damp')
    grid on
    figure
    plot(ticks,est_wc);
    legend('wc')
    grid on
    figure 
    grid on
    plot(ticks,act_est(:,5),ticks,act_est(:,6)*100/64)
    legend('Wp','Sigma')
end