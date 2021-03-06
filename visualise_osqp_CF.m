% *****************************************************************************
% *                                                                           *
% *		          Trajectory visualiser - OSQP	                              *
% *				Aren Karapetyan (c) 19/05/2020							      *
% *	     Plots the simulated and rosbag trajectories of three agents          *
% *                                                                           *
% *****************************************************************************
% *                                                                           *
% *   Fourth Year Project at Engineering Science, University of Oxford        *
% *        Distributed Control of Flying Quadrotors                           *
% *****************************************************************************

function visualise_osqp_CF (r,x,N,T,nx,nu)


% bag = rosbag('cf1_0.33.bag');
% bagselect1 = select(bag, 'Topic', '/CF1_position');
% 
% bag = rosbag('cf2_0.33.bag');
% bagselect2 = select(bag, 'Topic', '/CF1_position');
% 
% bag = rosbag('cf3_0.33.bag');
% bagselect3 = select(bag, 'Topic', '/CF1_position');

bag = rosbag('/Users/Karapetyan/Desktop/Oxford/Year 4 materials/4YP/Rosbags/cf1_0.45_swarm.bag');
bagselect1 = select(bag, 'Topic', '/CF1_position');
 
bag = rosbag('/Users/Karapetyan/Desktop/Oxford/Year 4 materials/4YP/Rosbags/cf2_0.45_swarm.bag');
bagselect2 = select(bag, 'Topic', '/CF1_position');
 
bag = rosbag('/Users/Karapetyan/Desktop/Oxford/Year 4 materials/4YP/Rosbags/cf3_0.45_swarm.bag');
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

    
figure;


pause on

% estimated from python code
m = 172;
 
for k = 1:(N+1)
    


    for i = 1:M
        if i ==1
            col = '#228B22';
            a ='gx';
        elseif i == 2
            col = 'blue';
            a = 'bx';
        else
            col = 'magenta';
            a = 'mx';
        end
        xlim([-0.3 1.4]) 
        ylim([-0.2 1.6])        
        px = plot(r(1+(i-1)*nx),r((i-1)*nx+nu),a,'MarkerSize',13);
        set(px(1), 'color', col);
        hold on
    end

    hold on
    
    % initial point
   
    po = plot(x(1),x(2),'ro',x(nx+1),x(nx+2),'bo',x(2*nx+1),x(2*nx+2),'ro');
    set(po(1), 'color', '#228B22');
    set(po,'MarkerSize',13);
    
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
    set(p(1), 'color', '#228B22');
    set(p,'LineWidth',2);
    hold on
    
    
    ps = plot(x(1+(k-1)*M*nx),x(nu+(k-1)*M*nx),'g*',x(nx+1+(k-1)*M*nx),x(nx+nu+(k-1)*M*nx),'b*',x(2*nx+1+(k-1)*M*nx),x(2*nx+nu+(k-1)*M*nx),'r*',cf1x(k+m-110),cf1y(k+m-110),'m*',cf2x(k+m-110),cf2y(k+m-110),'m*',cf3x(k+m-110),cf3y(k+m-110),'m*');
    set(ps(1), 'color', '#228B22');
    set(ps,'MarkerSize',10)
    hold off
    
    legend('off')

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