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
Q = eye(nu)*10;
R = eye(nu)*0.1;


M = 2; % Number of agents
delta = 0.01; % Inter-agent distance 
 
% initialize the states, reference, control and nominal states
for i = 1:M
    
    x(i,:) = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
    a(i,:) = sdpvar(repmat(nu,1,N),repmat(1,1,N));
    %r{i} = sdpvar(nu*N,1);
    %r{i} = ones(nu*N,1)*0;
    %x_nominal(i,:) = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
    
end

r(1,:) =[0,4];
r(2,:) =[0,4];


%  populate nominal values
% for k = 1:N
% 
% %     x_nominal{1,k} = ones(nx,1)*0.05;
% %     x_nominal{2,k} = -0.01 * ones(nx,1);
% %     x_nominal{3,k} = 0 * ones(nx,1);
% 
%     %x_nominal{1,k} = [4;k];
%     %x_nominal{2,k} = [k;3];
%    
% end

% initial conditions

x_1_0 = [8;4;0;0];
x_2_0 = [2;1;0;0];


x_3_0 = [-0.8;-1;0;0];

x1 = linspace(x_1_0(1),r(1),N);
y1 = linspace(x_1_0(2),r(3),N);

x2 = linspace(x_2_0(1),r(1),N);
y2 = linspace(x_2_0(2),r(4),N);

constraints = [];
objective = 0;

for i = 1:M
   

    for k = 1:N
        
        objective = objective + (x{i,k}(1:nu) - r(i,:)')'* Q * ...
            (x{i,k}(1:nu) - r(i,:)') + a{i,k}'* R * a{i,k};
        
        constraints = [constraints, x{i,k+1} == A*x{i,k} + B*a{i,k}, x{1,1} == x_1_0,x{2,1} == x_2_0, x{1,N+1}(3:4) == [0;0],x{2,N+1}(3:4) == [0;0],a{1,N} == [0;0],a{2,N} == [0;0]];       
    end
   
end

 optimize(constraints,objective);

%% Definition
for m = 1:25
constraints = [];
objective = 0;
sum =0;
    for i =1:M
    for k = 1:N
        %sum = sum +abs(x_nominal{i,k}(1:nu)-value(x{i,k}(1:nu)));
        x_nominal{i,k} = value(x{i,k}(1:nu));
        %sum = sum +abs(x_nominal{i,k}(1:nu)-value(x{i,k}(1:nu)));
    end
    end
 

for k = 1:N
   

   
    for i = 1:M
        
        objective = objective + (x{i,k}(1:nu) - r(i,:)')'* Q * ...
            (x{i,k}(1:nu) - r(i,:)') + a{i,k}'* R * a{i,k};
        
        constraints = [constraints, x{i,k+1} == A*x{i,k} + B*a{i,k}];
        
        for j = 1:M
        
            if(i~=j && k>2 )
                
              eta_ij = (x_nominal{i,k}(1:nu) - x_nominal{j,k}(1:nu)) * 1/norm(x_nominal{i,k}(1:nu) - x_nominal{j,k}(1:nu));
              h_ij = eta_ij'*((x{i,k}(1:nu) - x{j,k}(1:nu)) - (x_nominal{i,k}(1:nu) - x_nominal{j,k}(1:nu))) - delta ;  
              constraints = [constraints,  h_ij  >= 0];
              
            end
        end
            
    end


end


    %optimize([constraints, x{1,1} == x_1_0,x{2,1} == x_2_0,x{3,1} == x_3_0 ],objective);
    optimize([constraints, x{1,1} == x_1_0,x{2,1} == x_2_0, x{1,N+1}(3:4) == [0;0],x{2,N+1}(3:4) == [0;0],a{1,N} == [0;0],a{2,N} == [0;0]],objective);
    %x = solve(x_nominal);
    sum =0;

    m
       
end



%% Visualisation

%admm_visualise (r,x,N,T);
admm_visualise_2([0;4],x,N,T);



