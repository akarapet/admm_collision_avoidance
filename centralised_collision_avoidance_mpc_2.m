%% Initialisation


yalmip('clear')
clear all

% Model data
T = 0.5;
% A = [1 0 T 0;
%      0 1 0 T;
%      0 0 1 0;
%      0 0 0 1];
% B = [0 0;0 0;T 0;0 T];

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
N = 10;
Q = eye(nu)*14;
R = eye(nu)*5;
Qf = Q; Qf(nx,nx) = 0;
[K,S,e] = dlqr(A,B,Qf,R);

M = 3; % Number of agents
delta = 0.3; % Inter-agent distance 
 
% initialize the states, reference, control and nominal states
for i = 1:M
    
    x(i,:) = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
    a(i,:) = sdpvar(repmat(nu,1,N),repmat(1,1,N));
    %r{i} = sdpvar(nu*N,1);
    %r{i} = ones(nu*N,1)*0;
    %x_nominal(i,:) = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
    
end

r(1,:) =[0.5,1];
r(2,:) =[0,0.5];
r(3,:) =[0,0.5];

% initial conditions

x_0 = [0.5   1   1 ;
       0   0.5 1.5 ; 
       0     0   0 ;
       0     0   0 
       0     0   0
       0     0   0];

ops = sdpsettings('solver','osqp','verbose',0);


constraints = [];
objective = 0;

for i = 1:M
   
    for k = 1:N

        objective = objective + (x{i,k}(1:nu)-r(i,:)')'* Q * ...
             (x{i,k}(1:nu)-r(i,:)')  + a{i,k}'* R * a{i,k};
        
        constraints = [constraints, x{i,k+1} == A*x{i,k} + B*a{i,k}];       
    end
    
    constraints = [constraints,  x{i,1} == x_0(:,i),x{i,N+1}(3:4) == [0;0],a{i,N} == [0;0]];
   
end

 optimize(constraints,objective,ops);

 
   
    
%% Definition


MPC_L = 25;

for m = 1:MPC_L
    
    tic
    
    for it = 1:2
        
        constraints = [];
        objective = 0;
        
        for i =1:M
            
            constraints = [constraints, x{i,1} == x_0(:,i),x{i,N+1}(3:4) == [0;0],a{i,N} == [0;0]];
            
            for k = 1:N
                x_nominal{i,k} = value(x{i,k}(1:nu));
                
            end
        end
        
        for k = 1:N
            
            
            
            for i = 1:M
                
                %objective = objective  + (x{i,k}(1:nu) - ref{i,k})'* Q * ...
                %(x{i,k}(1:nu)-ref{i,k}) + a{i,k}'* R * a{i,k};
                
                objective = objective  + (x{i,k}(1:nu) - r(i,:)')'* Q * ...
                    (x{i,k}(1:nu)-r(i,:)') + a{i,k}'* R * a{i,k};
                
                constraints = [constraints, x{i,k+1} == A*x{i,k} + B*a{i,k}];
                
                for j = 1:M
                    
                    if(i~=j && k>2 )
                        
                        eta_ij = (x_nominal{i,k}(1:nu) - x_nominal{j,k}(1:nu)) * 1/norm(x_nominal{i,k}(1:nu) - x_nominal{j,k}(1:nu));
                        h_ij = eta_ij'*((x{i,k}(1:nu) - x{j,k}(1:nu)) - (x_nominal{i,k}(1:nu) - x_nominal{j,k}(1:nu))) - delta ;
                        constraints = [constraints, norm(x_nominal{i,k}(1:nu) - x_nominal{j,k}(1:nu))+h_ij >= 0];
                        
                    end
                end
                
            end
            
            
        end
        
        % Terminal Cost
        for i = 1:M
            objective = objective + (x{i,N}(1:nu)-r(i,:)')'* (S(1:2,1:2)-Q)* (x{i,N}(1:nu)-r(i,:)');
        end
        
        %optimize([constraints, x{1,1} == x_1_0,x{2,1} == x_2_0,x{3,1} == x_3_0,x{1,N+1}(3:4) ==[0;0],x{2,N+1}(3:4) == [0;0],x{3,N+1}(3:4) == [0;0],a{1,N} == [0;0],a{2,N} == [0;0],a{3,N} == [0;0]],objective);
        optimize(constraints,objective,ops);
        
    end
    
    x_0(:,1) = A*x_0(:,1) + B*a{1};
    x_0(:,2) = A*x_0(:,2) + B*a{2};
    x_0(:,3) = A*x_0(:,3) + B*a{3};
    toc
    for i =1:M
        implementedX{i,m} =  x_0(:,i);
    end
    m
end


%% Visualisation

admm_visualise (r,implementedX,MPC_L-1,T);
%admm_visualise([0.5;1.3],x,N,T);



