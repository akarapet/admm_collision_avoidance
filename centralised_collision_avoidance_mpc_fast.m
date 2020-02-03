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

% A = [1 0 0.09629 0 0 0.03962;
%          0 1 0 0.09629 -0.03962 0;
%          0 0 0.8943 0 0 0.7027;
%          0 0 0 0.8943 -0.7027 0;
%          0 0 0 0.1932 0.4524 0;
%          0 0 -0.1932 0 0 0.4524];
% B = [0.003709 0; 0 0.003709;0.1057 0;0 0.1057;0 -0.1932;0.1932 0];

nx = 4; % Number of states
nu = 2; % Number of inputs

% MPC data
N = 50;
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
    x_nominal(i,:) = sdpvar(repmat(nx,1,N),repmat(1,1,N));
    
end

r(1,:) =[0.5,1];
r(2,:) =[0,0.5];
r(3,:) =[0,0.5];

% initial conditions

x_0 = [0.5   1   1 ;
       0   0.5 1.5 ; 
       0     0   0 ;
       0     0   0 ];

ops = sdpsettings('verbose',0);


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



        
constraints = [];
objective = 0;

% for i =1:M
%     
%     constraints = [constraints,x{i,N+1}(3:4) == [0;0],a{i,N} == [0;0]];
%     
%     for k = 1:N
%         x_nominal{i,k} = x{i,k};
%         
%     end
% end
        for i =1:M           
            for k = 1:N
                x_bar{i,k} = value(x{i,k});  
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
                
%                 eta_ij = (x_nominal{i,k}(1:nu) - x_nominal{j,k}(1:nu)) * 1/norm(x_nominal{i,k}(1:nu) - x_nominal{j,k}(1:nu));
%                 h_ij = eta_ij'*((x{i,k}(1:nu) - x{j,k}(1:nu)) - (x_nominal{i,k}(1:nu) - x_nominal{j,k}(1:nu))) - delta ;
%                 constraints = [constraints, norm(x_nominal{i,k}(1:nu) - x_nominal{j,k}(1:nu))+h_ij >= 0];

                eta_ij = (x_nominal{i,k}(1:nu) - x_nominal{j,k}(1:nu)) * 1/(0.5*(x_nominal{i,k}(1:nu) - x_nominal{j,k}(1:nu))'*(x_nominal{i,k}(1:nu) - x_nominal{j,k}(1:nu)));
                h_ij = eta_ij'*((x{i,k}(1:nu) - x{j,k}(1:nu)) - (x_nominal{i,k}(1:nu) - x_nominal{j,k}(1:nu))) - delta ;
                constraints = [constraints, 0.5*(x_nominal{i,k}(1:nu) - x_nominal{j,k}(1:nu))'*(x_nominal{i,k}(1:nu) - x_nominal{j,k}(1:nu))+h_ij >= 0];

%                 eta_ij = (x_nominal{i,k}(1:nu) - x_nominal{j,k}(1:nu)) * 1/(sqrtm((x_nominal{i,k}(1:nu) - x_nominal{j,k}(1:nu))'*(x_nominal{i,k}(1:nu) - x_nominal{j,k}(1:nu))));
%                 h_ij = eta_ij'*((x{i,k}(1:nu) - x{j,k}(1:nu)) - (x_nominal{i,k}(1:nu) - x_nominal{j,k}(1:nu))) - delta ;
%                 constraints = [constraints, sqrtm((x_nominal{i,k}(1:nu) - x_nominal{j,k}(1:nu))'*(x_nominal{i,k}(1:nu) - x_nominal{j,k}(1:nu)))+h_ij >= 0];

            end
        end
        
    end
    
    
end

% Terminal Cost
for i = 1:M
    constraints = [constraints,x{i,N+1}(3:4) == [0;0],a{i,N} == [0;0]];
    objective = objective + (x{i,N}(1:nu)-r(i,:)')'* (S(1:2,1:2)-Q)* (x{i,N}(1:nu)-r(i,:)');
end

parameters_in = {x{1,1},x{2,1},x{3,1},x_nominal{:}};
solutions_out = {[a{1,1}],[a{2,1}],[a{3,1}],[x{1,:}],[x{2,:}],[x{3,:}]};

%optimize([constraints, x{1,1} == x_1_0,x{2,1} == x_2_0,x{3,1} == x_3_0,x{1,N+1}(3:4) ==[0;0],x{2,N+1}(3:4) == [0;0],x{3,N+1}(3:4) == [0;0],a{1,N} == [0;0],a{2,N} == [0;0],a{3,N} == [0;0]],objective);
%optimize(constraints,objective,ops);
options = sdpsettings('solver','mosek','verbose',2);
controller = optimizer(constraints, objective,options,parameters_in,solutions_out);

%x_bar= cell(3,50);

MPC_L = 10;

for m = 1:MPC_L
    

    for it = 1:3
        % optimize
        
        inputs = {x_0(:,1),x_0(:,2),x_0(:,3),x_bar{:}};
        [solutions,diagnostics] = controller{inputs};
        
        for i =1:M           
            for k = 1:N
                %x_bar{i,k} = value(x{i,k}); 
                x_bar{i,k} = solutions{i+3}(:,k);
            end
        end 
        %x_bar = solutions{4};
    end
    
    x_0(:,1) = A*x_0(:,1) + B*solutions{1};
    x_0(:,2) = A*x_0(:,2) + B*solutions{2};
    x_0(:,3) = A*x_0(:,3) + B*solutions{3};
    
    for i =1:M
        implementedX{i,m} =  x_0(:,i);
    end
    m
end


%% Visualisation

admm_visualise (r,implementedX,MPC_L-1,T);
%admm_visualise(r,x,N,T);


