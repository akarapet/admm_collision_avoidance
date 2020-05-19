% *****************************************************************************
% *                                                                           *
% *		          Trajectory visualiser - YALMIP	                          *
% *				Aren Karapetyan (c) 19/05/2020							      *
% *	     Plots the simulated trajectories of three agents                     *
% *                                                                           *
% *****************************************************************************
% *                                                                           *
% *   Fourth Year Project at Engineering Science, University of Oxford        *
% *        Distributed Control of Flying Quadrotors                           *
% *****************************************************************************

function visualise (r,x,N,T)

x_1_plot = [];
y_1_plot = [];

x_2_plot =[];
y_2_plot =[];

x_3_plot =[];
y_3_plot =[];

[m n] = size(r);
    
figure;


pause on

 
for k = 1:(N+1)
    
% reference 

for i = 1:m
        if i ==1
            col = '#228B22';
            a ='gx';
        elseif i == 2
            col = 'blue';
            a = 'bx';
        else
            col = 'red';
            a = 'rx';
        end
        xlim([-0.2 1.2]) 
        ylim([0.2 1.6])
        px = plot(r(i,1),r(i,2),a,'MarkerSize',13);
        set(px(1), 'color', col);
        hold on
end


hold on

% initial point
po = plot(value(x{1,1}(1)),value(x{1,1}(2)),'o',value(x{2,1}(1)),value(x{2,1}(2)),'bo',value(x{3,1}(1)),value(x{3,1}(2)),'ro');
set(po(1), 'color', '#228B22');

hold on
    
    
    x_1_plot = [x_1_plot, value(x{1,k}(1))];
    y_1_plot = [y_1_plot, value(x{1,k}(2))];
    
    x_2_plot = [x_2_plot, value(x{2,k}(1))];
    y_2_plot = [y_2_plot, value(x{2,k}(2))];

    x_3_plot = [x_3_plot, value(x{3,k}(1))];
    y_3_plot = [y_3_plot, value(x{3,k}(2))];
    


   
    p = plot(x_1_plot,y_1_plot,x_2_plot,y_2_plot,x_3_plot,y_3_plot);
    set(p(1), 'color', '#228B22');
    set(p(2), 'color', 'blue');
    set(p(3), 'color', 'red');
    hold on

    
    ps = plot(value(x{1,k}(1)),value(x{1,k}(2)),'g*',value(x{2,k}(1)),value(x{2,k}(2)),'b*',value(x{3,k}(1)),value(x{3,k}(2)),'r*');
    set(ps(1), 'color', '#228B22');
    hold off
    
    legend('off')
    pause(T)
end

xlabel('x [m]') 
ylabel('y [m]') 
legend(p,'Agent 1','Agent 2','Agent 3')

end