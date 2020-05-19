% *****************************************************************************
% *                                                                           *
% *		    Distributed Collision Avoidance with CF Dynamics - YALMIP	      *
% *				Aren Karapetyan (c) 19/05/2020							      *
% *	  Fully Decentralise ADMM Algorithm for Collision Avoidance        	      *
% *                                                                           *
% *****************************************************************************
% *                                                                           *
% *   Fourth Year Project at Engineering Science, University of Oxford        *
% *        Distributed Control of Flying Quadrotors                           *
% *****************************************************************************

%% Initialisation

yalmip('clear')
clear all

% Model data
T = 0.1; 

A = [1 0 0.09629 0 0 0.03962;
         0 1 0 0.09629 -0.03962 0;
         0 0 0.8943 0 0 0.7027;
         0 0 0 0.8943 -0.7027 0;
         0 0 0 0.1932 0.4524 0;
         0 0 -0.1932 0 0 0.4524];
     
B = [0.003709 0; 0 0.003709;0.1057 0;0 0.1057;0 -0.1932;0.1932 0];

nx = 6; % Number of states
nu = 2; % Number of inputs

% MPC data
N = 100;
Q = eye(nu)*10;

R = eye(nu)*13;

% Number of iterations
it = 1;

M = 3; % Number of agents
N_j = M-1; % Number of neighbours 
delta = 0.33; % Inter-agent distance 
rho = 2;


% position reference setpoint
% 
%   r =[0.5  0    0;
%       1  0.5  0.5;];
r(1,:) =[0.6,1.2];
r(2,:) =[0,0.6];
r(3,:) =[0,0.6];

% w and w_from_j

wc = cell(M,N);
w_from_jc = cell(M,M,N);


% lambda and lambda_from_j

lambda = cell(M,N+1);
lambda_from_j = cell(M,M,N);
lambda_to_j = cell(M,M,N);

for i = 1:M
    
    % position and acceleration
    x(i,:) = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
    a(i,:) = sdpvar(repmat(nu,1,N),repmat(1,1,N));
    w(i,:) = sdpvar(repmat(nu,1,N+1),repmat(1,1,N+1));
    
    % w_from_jc initialisation
    for j = 1:M
        w_to_j(i,j,:) = sdpvar(repmat(nu,1,N+1),repmat(1,1,N+1));
        for k =1:N
            w_from_jc{i,j,k} = [0;0];
            
            % w_to_j
            %w_to_jc{i,j,k} = [0.1;0.1];
            
            lambda_from_j{i,j,k} = [5;5]*0;
            lambda_to_j{i,j,k} = [5;5]*0;
        end
    end
    
end

for k = 1:N
    
    lambda{1,k} = [5;5]*0;
    lambda{2,k} = [5;5]*0;
    lambda{3,k} = [5;5]*0;
    
end

% initial conditions


x_0 = [0.6     1.2   1.2 ;
       0     0.612   1.5 ; 
       0       0     0 ;
       0       0     0
       0       0     0
       0       0     0];
   
ops = sdpsettings('verbose',0);

% initialise w-s
for i = 1:M
   
    objective =0;
    constraints =[];
    for k = 1:N
        
        objective = objective + (x{i,k}(1:nu)-r(i,:)')'* Q * ...
            (x{i,k}(1:nu)-r(i,:)') + a{i,k}'* R * a{i,k};
        
        constraints = [constraints, x{i,k+1} == A*x{i,k} + B*a{i,k}, x{i,1} == x_0(:,i),-1 <= a{i,k}(1) <= 1,-1 <= a{i,k}(2) <= 1];       
    end
    
    optimize(constraints,objective);
    
    for k =1:(N+1)
       wc{i,k} = value(x{i,k}(1:nu));
       ref{i,k} = value(x{i,k}(1:nu));
       
    end
    
end   

for i = 1:M
 for k = 1:N 
   for j = 1:M
      if i~=j
         w_from_jc{i,j,k} = value(x{j,k}(1:nu))*0;
         w_to_jc{i,j,k} = value(x{j,k}(1:nu))*0;
      end
   end 
 end
end

xprev=ref;

for m =1:it

sum = 0;    
    
%% Prediction

for i = 1:M
    

    constraints_1 = [];
    objective_1 = 0;
    
     for k = 1:N
         
         J = (x{i,k}(1:nu) - r(i,:)')'* Q * ...
             (x{i,k}(1:nu) - r(i,:)') + a{i,k}'* R * a{i,k};
         
         J_1 = lambda{i,k}'* (x{i,k}(1:nu) - wc{i,k});
          
         J_2 = rho * 0.5 * ((x{i,k}(1:nu) - wc{i,k})'*(x{i,k}(1:nu) - wc{i,k}));
         
         J_3 = 0;
         for j = 1:M
            if (i ~= j)
             J_3 = J_3 + lambda_from_j{i,j,k}'* (x{i,k}(1:nu) - w_from_jc{i,j,k}) ...
                 + rho * 0.5 * (x{i,k}(1:nu) - w_from_jc{i,j,k})' * (x{i,k}(1:nu) - w_from_jc{i,j,k});
            end 
         end
         
         constraints_1 = [constraints_1, x{i,k+1} == A*x{i,k} + B*a{i,k},-1 <= a{i,k}(1) <= 1,-1 <= a{i,k}(2) <= 1];%,x{i,k}(1)<=2,x{i,k}(1)>=-2,x{i,k}(2)<=2,x{i,k}(2)>=-2];
         objective_1 = objective_1 + J + J_1 + J_2 + J_3;
      end
      
      optimize([constraints_1, x{i,1} == x_0(:,i)],objective_1,ops);
    
end




%% Communication 1
% save in x global
xc = cell(M,N+1);
for i =1:M
    for k = 1:N+1
       
        xc{i,k} = value(x{i,k});
        
    end  
end


if m>1
for i =1:M
    for k = 1:N+1
       
        wc{i,k} = value(w{i,k});
        for j =1:M
            if (i~=j)
             w_to_jc{i,j,k} = value(w_to_j{i,j,k});   
            end
        end
    end  
end
end
%% Coordination


for i = 1:M
    
    constraints_2 = [];
    objective_2 = 0;
    
    for k = 1:N
      
      J_1 =  lambda{i,k}' * (xc{i,k}(1:nu) - w{i,k}) + rho*0.5*(xc{i,k}(1:nu) - w{i,k})'*(xc{i,k}(1:nu) - w{i,k});
      %objective_2 = objective_2+J_1;
      
      J_2 =0;
      for j =1:M
          if (i~=j)
              J_2 = J_2 + lambda_to_j{i,j,k}'*(xc{j,k}(1:nu)-w_to_j{i,j,k})+rho*0.5*(xc{j,k}(1:nu)-w_to_j{i,j,k})'*(xc{j,k}(1:nu)-w_to_j{i,j,k});
          
              eta_ij = (xc{i,k}(1:nu) - xc{j,k}(1:nu)) * 1/norm(xc{i,k}(1:nu) - xc{j,k}(1:nu));
              h_ij = eta_ij' * ((w{i,k} - w_to_j{i,j,k})-(xc{i,k}(1:nu) - xc{j,k}(1:nu))) - delta;
               
               constraints_2 = [constraints_2, norm(xc{i,k}(1:nu) - xc{j,k}(1:nu)) + h_ij >= 0];
          end
          
      end
      objective_2 = objective_2 + J_1+J_2;
    end
       
    optimize(constraints_2,objective_2,ops);
 
end

%% Mediation

for i =1:M
    
    for k=1:N
        lambda{i,k} = lambda{i,k} + rho * (xc{i,k}(1:nu)-value(w{i,k}));

    for j = 1:M
        if i~=j
        lambda_to_j{i,j,k} = lambda_to_j{i,j,k} + rho * (xc{j,k}(1:nu)-value(w_to_j{i,j,k}));
        end
    end
    end
end

%% Communication 2

for i = 1:M
    for k = 1:N
        for j =1:M
            if i~=j
           lambda_from_j{i,j,k} = lambda_to_j{j,i,k};
           w_from_jc{i,j,k} = value(w_to_j{j,i,k});
            end
        end
    end
end



s1 = sprintf('iteration: %d \n', m );
disp(s1);

% objective calculation
for k = 1:N
   sum = sum + abs(value(norm(x{1,k}(1:nu) - xprev{1,k}(1:nu))));
end

s2 = sprintf('value: %d \n', sum);
disp(s2);

for k =1:N
    xprev{1,k} = value(x{1,k}(1:nu));
end

end



%% Visualisation

visualise(r,x,N,T);
