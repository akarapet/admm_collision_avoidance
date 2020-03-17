function admm_visualise_osqp (r,x,N,T,nx,nu)


bag = rosbag('/home/antonis/4YP_Distributed_Control/dist_ws/src/dist_pkg/src/crazyflie-lib-python/examples/dist_swarm/RosBags/cf1.bag');
bagselect1 = select(bag, 'Topic', '/CF1_position');

bag = rosbag('/home/antonis/4YP_Distributed_Control/dist_ws/src/dist_pkg/src/crazyflie-lib-python/examples/dist_swarm/RosBags/cf2.bag');
bagselect2 = select(bag, 'Topic', '/CF1_position');

bag = rosbag('/home/antonis/4YP_Distributed_Control/dist_ws/src/dist_pkg/src/crazyflie-lib-python/examples/dist_swarm/RosBags/cf3.bag');
bagselect3 = select(bag, 'Topic', '/CF1_position');

Y = timeseries(bagselect1,'Y');
X = timeseries(bagselect1,'X');

Y2 = timeseries(bagselect2,'Y');
X2 = timeseries(bagselect2,'X');

Y3 = timeseries(bagselect3,'Y');
X3 = timeseries(bagselect3,'X');

cf1x = X.data;
cf1y = Y.data;

cf2x = X2.data;
cf2y = Y2.data;

cf3x = X3.data;
cf3y = Y3.data;

cf1x_plot = [];
cf1y_plot = [];

cf2x_plot = [];
cf2y_plot = [];

cf3x_plot = [];
cf3y_plot = [];

[m,n] = size(cf1x);

x_1_plot = [];
y_1_plot = [];

x_2_plot =[];
y_2_plot =[];

x_3_plot =[];
y_3_plot =[];

M = 3;

%[m n] = size(r);
    
figure;


pause on

% estimated from python code
m = 172;
 
for k = 1:(N+1)
    


    for i = 1:M
        plot(r(1+(i-1)*nx),r((i-1)*nx+nu),'kx')
        hold on
    end

    hold on
    
    % initial point
   
    plot(x(1),x(2),'ro',x(nx+1),x(nx+2),'ro',x(2*nx+1),x(2*nx+2),'ro')
    
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
    
    cf2x_plot = [cf2x_plot,cf2x(k+m-110)];
    cf2y_plot = [cf2y_plot, cf2y(k+m-110)];
    
    cf3x_plot = [cf3x_plot,cf3x(k+m-110)];
    cf3y_plot = [cf3y_plot, cf3y(k+m-110)];
    
    p = plot(x_1_plot,y_1_plot,'g',x_2_plot,y_2_plot,'b',x_3_plot,y_3_plot,'r',cf1x_plot,cf1y_plot,'m',cf2x_plot,cf2y_plot,'m',cf3x_plot,cf3y_plot,'m');
    hold on
    
    
    plot(x(1+(k-1)*M*nx),x(nu+(k-1)*M*nx),'g*',x(nx+1+(k-1)*M*nx),x(nx+nu+(k-1)*M*nx),'b*',x(2*nx+1+(k-1)*M*nx),x(2*nx+nu+(k-1)*M*nx),'r*',cf1x(k+m-110),cf1y(k+m-110),'m*',cf2x(k+m-110),cf2y(k+m-110),'m*',cf3x(k+m-110),cf3y(k+m-110),'m*')
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

legend(p([1 2 3]),'Agent 1','Agent 2','Agent 3')

end