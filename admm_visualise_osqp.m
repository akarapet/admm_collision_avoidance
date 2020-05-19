function admm_visualise_osqp (r,x,N,T,nx,nu)

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
        xlim([-0.2 1.4]) 
        ylim([-0.2 1.6])        
        px = plot(r(1+(i-1)*nx),r((i-1)*nx+nu),a,'MarkerSize',13);
        set(px(1), 'color', col);
        hold on
    end

    hold on
    
    % initial point
   
    po = plot(x(1),x(2),'o',x(nx+1),x(nx+2),'bo',x(nx*2+1),x(nx*2+2),'ro');
    set(po(1), 'color', '#228B22');
    
    hold on
    
    
    x_1_plot = [x_1_plot, x(1+(k-1)*M*nx)];
    y_1_plot = [y_1_plot, x(nu+(k-1)*M*nx)];
    
    x_2_plot = [x_2_plot, x(nx+1+(k-1)*M*nx)];
    y_2_plot = [y_2_plot, x(nx+nu+(k-1)*M*nx)];

    x_3_plot = [x_3_plot, x(2*nx+1+(k-1)*M*nx)];
    y_3_plot = [y_3_plot, x(2*nx+nu+(k-1)*M*nx)];
    

    p = plot(x_1_plot,y_1_plot,x_2_plot,y_2_plot,x_3_plot,y_3_plot);
    set(p(1), 'color', '#228B22');
    set(p(2), 'color', 'blue');
    set(p(3), 'color', 'red');
    hold on
    
    
    ps = plot(x(1+(k-1)*M*nx),x(nu+(k-1)*M*nx),'g*',x(nx+1+(k-1)*M*nx),x(nx+nu+(k-1)*M*nx),'b*',x(2*nx+1+(k-1)*M*nx),x(2*nx+nu+(k-1)*M*nx),'r*');
    set(ps(1), 'color', '#228B22');
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
xlabel('x [m]') 
ylabel('y [m]') 
legend(p,'Agent 1','Agent 2','Agent 3');

end