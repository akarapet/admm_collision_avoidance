function admm_visualise (r,x,N,T)

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
%plot(r{1}(1),r{1}(2),'kx')
%plot(r(1),r(2),'kx')

% plot(r(1,1),r(1,2),'kx')
% hold on
% plot(r(2,1),r(2,2),'kx')
% hold on
% plot(r(3,1),r(3,2),'kx')

for i = 1:m
        plot(r(i,1),r(i,2),'kx')
        hold on
end

hold on

% initial point
%plot(x_1_0(1),x_1_0(2),'ro',x_2_0(1),x_2_0(2),'ro',x_3_0(1),x_3_0(2),'ro')
plot(value(x{1,1}(1)),value(x{1,1}(2)),'ro',value(x{2,1}(1)),value(x{2,1}(2)),'ro',value(x{3,1}(1)),value(x{3,1}(2)),'ro')

hold on
    
    
    x_1_plot = [x_1_plot, value(x{1,k}(1))];
    y_1_plot = [y_1_plot, value(x{1,k}(2))];
    
    x_2_plot = [x_2_plot, value(x{2,k}(1))];
    y_2_plot = [y_2_plot, value(x{2,k}(2))];

    x_3_plot = [x_3_plot, value(x{3,k}(1))];
    y_3_plot = [y_3_plot, value(x{3,k}(2))];
    


    p = plot(x_1_plot,y_1_plot,'g',x_2_plot,y_2_plot,'b',x_3_plot,y_3_plot,'r');
    hold on
    
    %legend('Agent 1','Agent 2','Agent3')
    
    
    plot(value(x{1,k}(1)),value(x{1,k}(2)),'g*',value(x{2,k}(1)),value(x{2,k}(2)),'b*',value(x{3,k}(1)),value(x{3,k}(2)),'r*')
    hold off
    
    legend('off')
    %hold on
    
    %plot(value(x{1,k}(1)),value(x{1,k}(2)),'b*',value(x{2,k}(1)),value(x{2,k}(2)),'b*',value(x{3,k}(1)),value(x{3,k}(2)),'b*')

    pause(T)
end

legend(p,'Agent 1','Agent 2','Agent3')

end