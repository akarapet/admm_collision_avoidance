function admm_visualise_2drones(r,x,N,T)

x_1_plot = [];
y_1_plot = [];

x_2_plot =[];
y_2_plot =[];


[m n] = size(r);
    
figure;


pause on

 
for k = 1:(N+1)
    
% reference 

for i = 1:m
        plot(r(i,1),r(i,2),'kx')
        hold on
end

hold on

% initial point

plot(value(x{1,1}(1)),value(x{1,1}(2)),'ro',value(x{2,1}(1)),value(x{2,1}(2)),'ro')

hold on
    
    
    x_1_plot = [x_1_plot, value(x{1,k}(1))];
    y_1_plot = [y_1_plot, value(x{1,k}(2))];
    
    x_2_plot = [x_2_plot, value(x{2,k}(1))];
    y_2_plot = [y_2_plot, value(x{2,k}(2))];


    p = plot(x_1_plot,y_1_plot,'g',x_2_plot,y_2_plot,'b');
    hold on
    
    plot(value(x{1,k}(1)),value(x{1,k}(2)),'g*',value(x{2,k}(1)),value(x{2,k}(2)),'b*')
    hold off
    
    legend('off')
    %hold on
    
    %plot(value(x{1,k}(1)),value(x{1,k}(2)),'b*',value(x{2,k}(1)),value(x{2,k}(2)),'b*',value(x{3,k}(1)),value(x{3,k}(2)),'b*')

    pause(T)
end

legend(p,'Agent 1','Agent 2')

end