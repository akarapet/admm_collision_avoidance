function admm_visualise_osqp (r,x,N,T)

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
    


    p = plot(x_1_plot,y_1_plot,'g',x_2_plot,y_2_plot,'b',x_3_plot,y_3_plot,'r');
    hold on
    
    
    plot(x(1+(k-1)*M*nx),x(nu+(k-1)*M*nx),'g*',x(nx+1+(k-1)*M*nx),x(nx+nu+(k-1)*M*nx),'b*',x(2*nx+1+(k-1)*M*nx),x(2*nx+nu+(k-1)*M*nx),'r*')
    hold off
    
    legend('off')
    %hold on
    
    %plot(value(x{1,k}(1)),value(x{1,k}(2)),'b*',value(x{2,k}(1)),value(x{2,k}(2)),'b*',value(x{3,k}(1)),value(x{3,k}(2)),'b*')

    pause(T)
end

legend(p,'Agent 1','Agent 2','Agent3')

end