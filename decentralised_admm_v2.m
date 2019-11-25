%% Initialisation

yalmip('clear')
clear all

% Model data
T = 0.1; 
A = [1 0 T 0;
     0 1 0 T;
     0 0 1 0;
     0 0 0 1];
B = [0 0;0 0;T 0;0 T];

nx = 4; % Number of states
nu = 2; % Number of inputs

% MPC data
N = 100;
Q = eye(nu)*1;
R = eye(nu)*1;

% Number of iterations
it = 25;

M = 2; % Number of agents
N_j = M-1; % Number of neighbours 
delta = 0.05; % Inter-agent distance 
rho = 0.5;


% position reference setpoint

r =[0;0];


% w and w_from_j

wc = cell(M,N);
w_from_jc = cell(M,M,N);


% lambda and lambda_from_j

lambda = cell(M,N);
lambda_from_j = cell(M,M,N);
lambda_to_j = cell(M,M,N);

for i = 1:M
    
    % position and acceleration
    x(i,:) = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
    a(i,:) = sdpvar(repmat(nu,1,N),repmat(1,1,N));
    
    
    % w_from_jc initialisation
    for j = 1:M
        for k =1:N
            w_from_jc{i,j,k} = [0;0];
            
            % w_to_j
            w_to_jc{i,j,k} = [0.1;0.1];
            
            lambda_from_j{i,j,k} = [0.01;0.01];
            lambda_to_j{i,j,k} = [0.01;0.01];
        end
    end
    
end

for k = 1:N
    
    lambda{1,k} = [0.01;0.01];
    lambda{2,k} = [0.01;0.01];
    lambda{3,k} = [0.01;0.01];
    
    wc{1,k} = [-0.01;-0.02];
    wc{2,k} = [0.02;0.01];
    wc{3,k} = [0;0];
    
end

% initial conditions


x_0 = [1 -1 -1 ;
       2 -2  2 ; 
       0  0   0 ;
       0  0   0];
ops = sdpsettings('verbose',0);
   
for m =1:it   
%% Prediction

for i = 1:M
    

    constraints_1 = [];
    objective_1 = 0;
    
     for k = 1:N
         
         J = (x{i,k}(1:nu) - r)'* Q * ...
             (x{i,k}(1:nu) - r) + a{i,k}'* R * a{i,k};
         
         J_1 = lambda{i,k}'* (x{i,k}(1:nu) - wc{i,k});
          
         J_2 = rho * 0.5 * ((x{i,k}(1:nu) - wc{i,k})'*(x{i,k}(1:nu) - wc{i,k}));
         
         J_3 = 0;
         for j = 1:M
            if (i ~= j&& k > 2 )
             J_3 = J_3 + lambda_from_j{i,j,k}'* (x{i,k}(1:nu) - w_from_jc{i,j,k}) ...
                 + rho * 0.5 * (x{i,k}(1:nu) - w_from_jc{i,j,k})' * (x{i,k}(1:nu) - w_from_jc{i,j,k});
            end 
         end
         
         constraints_1 = [constraints_1, x{i,k+1} == A*x{i,k} + B*a{i,k}];%,x{i,k}(1)<=2,x{i,k}(1)>=-2,x{i,k}(2)<=2,x{i,k}(2)>=-2];
         objective_1 = objective_1 + J + J_1 + J_2 + J_3;
      end
      
      optimize([constraints_1, x{i,1} == x_0(:,i),x{i,N}(3:4) == [0;0]],objective_1,ops);
    
end

%% Communication 1

% save in x global
xc = cell(M,N+1);
for i =1:M
    for k = 1:N+1
       
        xc{i,k} = value(x{i,k});
        
    end  
end

for i = 1:M
    w(i,:) = sdpvar(repmat(nu,1,N+1),repmat(1,1,N+1));
    for j = 1:M
        w_to_j(i,j,:) = sdpvar(repmat(nu,1,N+1),repmat(1,1,N+1));
    end
end

for i = 1:M
    
    constraints_2 = [];
    objective_2 = 0;
    
    for k = 1:N
      J_1 =  lambda{i,k}' * (xc{i,k}(1:nu) - w{i,k}) + rho*0.5*(xc{i,k}(1:nu) - w{i,k})'*(xc{i,k}(1:nu) - w{i,k});
      objective_2 = objective_2+J_1;
      J_2 =0;
      for j =1:M
          if (i~=j && k>2)
              J_2 = J_2 + lambda_to_j{i,j,k}'*(xc{j,k}(1:nu)-w_to_j{i,j,k})+rho*0.5*(xc{j,k}(1:nu)-w_to_j{i,j,k})'*(xc{j,k}(1:nu)-w_to_j{i,j,k});
          
              eta_ij = (wc{i,k} - w_to_jc{i,j,k}) * 1/norm(wc{i,k} - w_to_jc{i,j,k});
              part_g = eta_ij' * ((w{i,k} - w_to_j{i,j,k})-(wc{i,k} - w_to_jc{i,j,k})) - delta;
              constraints_2 = [constraints_2, part_g>=0];
          end
          
      end
      objective_2 = objective_2 + J_2;
    end
       
    optimize(constraints_2,objective_2);
 
end

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


for i =1:M
    
    for k=1:N
        lambda{i,k} = lambda{i,k} + rho * (xc{i,k}(1:nu)-wc{i,k});
    for j = 1:M
        if i~=j
        lambda_to_j{i,j,k} = lambda_to_j{i,j,k} + rho * (xc{j,k}(1:nu)-w_to_jc{i,j,k});
        end
    end
    end
end


for i = 1:M
    for k = 1:N
        for j =1:M
            if i~=j
           lambda_from_j{i,j,k} = lambda_to_j{j,i,k};
           w_from_jc{i,j,k} = w_to_jc{j,i,k};
            end
        end
    end
end
% %% Coordination
% 
% 
% for i = 1:M
%       
%      constraints_2 = [];
%      objective_2 = 0;
%      
%     
%      w(i,:) = sdpvar(repmat(nu,1,N+1),repmat(1,1,N+1));
%     
%     % w_to_j
%     for j = 1:M
%        w_to_j(i,j,:) = sdpvar(repmat(nu,1,N+1),repmat(1,1,N+1)); 
%     end
%      
%      for k = 1:N
% 
%          J_1 = lambda{i,k}' * (xc{i,k}(1:nu) - w{i,k});
%          
%          J_2 = rho * 0.5 * ((xc{i,k}(1:nu) - w{i,k})' * (xc{i,k}(1:nu) - w{i,k}));
%          
%          J_3 = 0;
%          
%           for j = 1:M
%             if (i~=j&& k > 2 )  
%                 
%              J_3 = J_3 + lambda_to_j{i,j,k}'* (xc{j,k}(1:nu) - w_to_j{i,j,k}) ...
%                  + rho * 0.5 * (xc{j,k}(1:nu) - w_to_j{i,j,k})' * (xc{j,k}(1:nu) - w_to_j{i,j,k}) ; 
%                          
%              eta_ij = (wc{i,k}(1:nu) - w_to_jc{i,j,k}) * 1/(norm(wc{i,k} - w_to_jc{i,j,k}));
%              
%              part_g = (w{i,k} - w_to_j{i,j,k}) - (wc{i,k} - w_to_jc{i,j,k}) - delta;
%             
%              constraints_2 = [constraints_2, eta_ij' * part_g >= 0];
%             
%             end
%           end
%                 
%           objective_2 = objective_2 + J_1 + J_2 + J_3;
%      
%      end
% 
%       optimize(constraints_2,objective_2,ops);
%     
% end
% 
% 
% for i =1:M
%     for k = 1:N
%        
%         wc{i,k} = value(w{i,k});
%         
%         for j=1:M
%             w_to_jc{i,j,k} = value(w_to_j{i,j,k});
%         end
%         
%     end  
% end
% 
% wc{2,45}
% %% Mediation
% 
%  for i = 1:M
% %     
%      for k = 1:N
%      
%      lambda{i,k} = lambda{i,k} + rho * (xc{i,k}(1:nu) - wc{i,k});
%      %value(lambda{i,k})
%           for j = 1:M
%               if (i~=j)
%                 
%              lambda_to_j{i,j,k} = lambda_to_j{i,j,k}+ rho * (xc{j,k}(1:nu) -  w_to_jc{i,j,k});
%              
%               end
%           end
%      
%      end
% %     
%  end
% 
% %% Communication 2
% 
% 
% for i = 1:M
%     
%     for j = 1:M
%         if(i~=j)
%            
%             for k = 1:N
%                 
%                lambda_from_j{i,j,k} =  lambda_to_j{j,i,k};
%                w_from_jc{i,j,k} = w_to_jc{j,i,k};
%             end
%             
%         end       
%     end
%     
% end

m
value(w{2,45})
end

%% Visualisation


admm_visualise_2(r,x,N,T);

