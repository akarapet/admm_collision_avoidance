function admm_visualise_osqp (r,x,N,T)


bag = rosbag('/home/antonis/4YP_Distributed_Control/dist_ws/src/dist_pkg/src/crazyflie-lib-python/examples/dist_swarm/RosBags/cf1.bag');
bagselect1 = select(bag, 'Topic', '/CF1_position');

Z = timeseries(bagselect1,'Z');
Y = timeseries(bagselect1,'Y');
X = timeseries(bagselect1,'X');

cf1x = X.data;
cf1y = Y.data;

cf1x_plot = [];
cf1y_plot = [];

[m,n] = size(cf1x);

x_1_plot = [];
y_1_plot = [];

x_2_plot =[];
y_2_plot =[];

x_3_plot =[];
y_3_plot =[];

M = 3;
nu = 2;
nx = 6;
%[m n] = size(r);
    
figure;


pause on

 
for k = 1:(N+1)
    


    for i = 1:M
        plot(r(1+(i-1)*nx),r((i-1)*nx+nu),'kx')
        hold on
    end

    hold on
    
    % initial point
   
    plot(x(1),x(2),'ro',x(7),x(8),'ro',x(13),x(14),'ro')
    
    hold on
    
    
    x_1_plot = [x_1_plot, x(1+(k-1)*M*nx)];
    y_1_plot = [y_1_plot, x(nu+(k-1)*M*nx)];
    
    x_2_plot = [x_2_plot, x(nx+1+(k-1)*M*nx)];
    y_2_plot = [y_2_plot, x(nx+nu+(k-1)*M*nx)];

    x_3_plot = [x_3_plot, x(2*nx+1+(k-1)*M*nx)];
    y_3_plot = [y_3_plot, x(2*nx+nu+(k-1)*M*nx)];
    
    % Real CF for the last 10 seconds
    cf1x_plot = [cf1x_plot,cf1x(k+m-110)];
    cf1y_plot = [cf1y_plot, cf1y(k+m-110)];
    


    p = plot(x_1_plot,y_1_plot,'g',x_2_plot,y_2_plot,'b',x_3_plot,y_3_plot,'r',cf1x_plot,cf1y_plot,'m');
    hold on
    
    
    plot(x(1+(k-1)*M*nx),x(nu+(k-1)*M*nx),'g*',x(nx+1+(k-1)*M*nx),x(nx+nu+(k-1)*M*nx),'b*',x(2*nx+1+(k-1)*M*nx),x(2*nx+nu+(k-1)*M*nx),'r*',cf1x(k+m-110),cf1y(k+m-110),'m*')
    hold off
    
    legend('off')
    %hold on
    
    %plot(value(x{1,k}(1)),value(x{1,k}(2)),'b*',value(x{2,k}(1)),value(x{2,k}(2)),'b*',value(x{3,k}(1)),value(x{3,k}(2)),'b*')

    pause(T)
end



% for i = 52:m
%     hold on
% 
%     
%     plot(x_1_plot,y_1_plot,'m');
%     hold on
%     
%     plot(x(i),y(i),'m*')
%     hold off
%     
%     pause(T)
%     hold off
% end
% Z.Time = Z.Time - bag.StartTime;
% Y.Time = Y.Time - bag.StartTime;
% X.Time = X.Time - bag.StartTime;

legend(p,'Agent 1','Agent 2','Agent 3', 'CrazyFlie 1')

end