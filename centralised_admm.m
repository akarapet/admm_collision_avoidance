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
Q = eye(nu)*2;
R = eye(nu)*1;


M = 3; % Number of agents
delta = 0.05; % Inter-agent distance 
 
% initialize the states, reference, control and nominal states
for i = 1:M
    
    x(i,:) = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
    a(i,:) = sdpvar(repmat(nu,1,N),repmat(1,1,N));
    %r{i} = sdpvar(nu*N,1);
    r{i} = ones(nu*N,1)*1;
    x_nominal(i,:) = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
    
end


%  populate nominal values
for k = 1:N

    x_nominal{1,k} = ones(nx,1)*0.05;
    x_nominal{2,k} = -0.01 * ones(nx,1);
    x_nominal{3,k} = 0 * ones(nx,1);

end

% initial conditions

x_1_0 = [1;2;0;0];
x_2_0 = [-1;-2;0;0];
x_3_0 = [-0.8;-1;0;0];


%% Definition

constraints = [];
objective = 0;

for m = 1:3

for k = 1:N
   
    
    x_nominal{i,k}
    for i = 1:M
        
        objective = objective + (x{i,k}(1:nu) - r{i}((k-1)*nu+1:(k-1)*nu+nu))'* Q * ...
            (x{i,k}(1:nu) - r{i}((k-1)*nu+1:(k-1)*nu+nu)) + a{i,k}'* R * a{i,k};
        
        constraints = [constraints, x{i,k+1} == A*x{i,k} + B*a{i,k}];
        
        for j = 1:M
        
            if(i~=j && k > 2 )
                
              eta_ij = (x_nominal{i,k}(1:nu) - x_nominal{j,k}(1:nu)) * 1/norm(x_nominal{i,k}(1:nu) - x_nominal{j,k}(1:nu));
              part_g = (x{i,k}(1:nu) - x{j,k}(1:nu)) - (x_nominal{i,k}(1:nu) - x_nominal{j,k}(1:nu)) - delta;  
              constraints = [constraints, eta_ij' * part_g >= 0];
              
            end
        end
            
    end
 %objective = objective + norm(Q*x{k},1) + norm(R*u{k},1);
 %constraints = [constraints, x{k+1} == A*x{k} + B*u{k}];
 %constraints = [constraints, -1 <= u{k}<= 1, -5<=x{k+1}<=5];

end


%solve = optimizer([constraints, x{1} == [1;1;0;0],x{11} == [-1;-1;0;0] ],objective,[],x_nominal,x);


    optimize([constraints, x{1,1} == x_1_0,x{2,1} == x_2_0,x{3,1} == x_3_0 ],objective);
    %x = solve(x_nominal);
    for k = 1:N*M
        x_nominal{k}(1:nu) = x{k}(1:nu);
    end
    m
end



%% Visualisation

admm_visualise (r,x,N,T);




