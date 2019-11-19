
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
N_j = M-1; % Number of neighbours 
delta = 0.05; % Inter-agent distance 
rho = 0.5;


% initialize the states, reference, control and nominal states for each
% agent
for i = 1:M
    
    % position and acceleration
    x(i,:) = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
    a(i,:) = sdpvar(repmat(nu,1,N),repmat(1,1,N));
    
    % position reference setpoint
    r{i} = sdpvar(nu*N,1);
    r{i} = ones(nu*N,1)*0;
    
    % nominal position 
    x_nominal(i,:) = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
    

    % w and w_to_j
    w(i,:) = sdpvar(repmat(nu,1,N+1),repmat(1,1,N+1));
    for j = 1:(M-1)
        w_to_j((i-1)*(M-1)+j,:) = sdpvar(repmat(nu,1,N+1),repmat(1,1,N+1));
    end
end

    % w_from_j
    w_from_j = zeros(M*(M-1)*nu,N+1);

    % lambda, lambda_to_j and lambda_from_j
    lambda = ones(M*nu,1) * 0.1;
    lambda_to_j = ones(M*nu,M-1) * 0.1;
    lambda_from_j = ones(M*nu,M-1) * 0.1;
    


for k = 1:N
    
    %  populate nominal values
    x_nominal{1,k} = ones(nx,1)*0.05;
    x_nominal{2,k} = -0.01 * ones(nx,1);
    x_nominal{3,k} = 0 * ones(nx,1);
    
    % initialise w
    
    w{1,k} = ones(nu,1)*0.1;
    w{2,k} = -0.1 * ones(nu,1);
    w{3,k} = 0 * ones(nu,1);

end

% initial conditions


x_0 = [1 -1 -0.8 ;
       2 -2 -1.6 ; 
       0  0    0 ;
       0  0    0];

%% Prediction



for i = 1:M
    
    w_from_j_to_i = w_from_j((i-1)*N_j*nu+1:(i-1)*N_j*nu+nu*N_j,:);
    
    constraints_1 = [];
    objective_1 = 0;
    
    for k = 1:N
        
        J = (x{i,k}(1:nu) - r{i}((k-1)*nu+1:(k-1)*nu+nu))'* Q * ...
            (x{i,k}(1:nu) - r{i}((k-1)*nu+1:(k-1)*nu+nu)) + a{i,k}'* R * a{i,k};
        
        J_1 = lambda((i-1)*(nu)+1:(i-1)*(nu)+nu)'* (x{i,k}(1:nu) - w{i,k});
        
        J_2 = rho * 0.5 * ((x{i,k}(1:nu) - w{i,k})'*(x{i,k}(1:nu) - w{i,k}));
       
        J_3 = 0;
        for j = 1:N_j
           
            J_3 = J_3 + lambda_from_j((i-1)*(nu)+1:(i-1)*(nu)+nu,j)'* (x{i,k}(1:nu) - w_from_j_to_i((j-1)*(nu)+1:(j-1)*(nu)+nu,k)) ...
                + rho * 0.5 * (x{i,k}(1:nu) - w_from_j_to_i((j-1)*(nu)+1:(j-1)*(nu)+nu,k))' * (x{i,k}(1:nu) - w_from_j_to_i((j-1)*(nu)+1:(j-1)*(nu)+nu,k));
            
        end
        
        constraints_1 = [constraints_1, x{i,k+1} == A*x{i,k} + B*a{i,k}];
        objective_1 = objective_1 + J + J_1 + J_2 + J_3;
    end
    
    optimize([constraints_1, x{i,1} == x_0(:,i)],objective_1);
    
end


%% Coordination




%% Visualisation




