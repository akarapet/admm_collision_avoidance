clear, clc

% Discrete time model of a quadcopter
% Obtained from formulate_system for T = 0.1
A = [1 0 0.09629 0 0 0.03962;
    0 1 0 0.09629 -0.03962 0;
    0 0 0.8943 0 0 0.7027;
    0 0 0 0.8943 -0.7027 0;
    0 0 0 0.1932 0.4524 0;
    0 0 -0.1932 0 0 0.4524];

B = [0.003709 0; 0 0.003709;0.1057 0;0 0.1057;0 -0.1932;0.1932 0];


T = 0.1;
M = 3; % Number of agents

nx = 6;
nu = 2;

% MPC data

N = 10;

Q = eye(nu)*10;
Q = blkdiag(Q,eye(nu)*10);
R = eye(nu)*13;
Q(nx,nx) = 0;


[K,QN,e] = dlqr(A,B,Q,R);
QN = Q;
% ADMM Data

N_j = [2 3; 1 3;1 2];

rho = 40;

lambda = ones(2,1)*0;
lambda_to_ = ones(2,1)*0;
lambda_from_ = ones(2,1)*0;

w = ones(2,1)*0;
x = zeros(nx,1);
w_from_ = ones(2,1)*0;
w_to_ = ones(2,1)*0;

lambda = repmat(lambda, N+1, 3);

for i = 1:M
    lambda_to_j{i} = repmat(lambda_to_, N+1, M-1);
    lambda_from_j{i} = repmat(lambda_from_, N+1, M-1);
    w_from_j{i} = repmat(w_from_, N+1, M-1);
    w_to_j{i} = repmat(w_to_, N+1, M-1);
end
w = repmat(w, N+1, 3);
x = repmat(x, N+1, 3);


% Initial and reference states
% r = [0.6,1.2,0,0,0,0; 0,0.6,0,0,0,0; 0,0.6,0,0,0,0];
% x0 = [0.6,0,0,0,0,0;  1.2,0.608,0,0,0,0; 1.2,1.5,0,0,0,0;];
r = [1.0,1.4,0,0,0,0; 0.6,0.4,0,0,0,0; 0,0.9,0,0,0,0];
x0 = [0,0.5,0,0,0,0;  0.2,0.9,0,0,0,0; 1.0,0.3,0,0,0,0;];

delta = 0.35; % Inter-agent distance

%% Useful Matrices
posM = blkdiag(eye(2),zeros(4,4)); % Matrix to take only position from the state vector x
d = [eye(2),zeros(nu,nx-nu)]; % Matrix to take only position from the state vector x
posMN = kron(eye(N+1),d);

rhoM = kron(speye((N+1)),rho/2*eye(2)); % matrix for the quadratic objective formualation
% Transformation matrix H creation

kron_mat = -1*eye(2);
H = kron(eye(N+1),repmat(d, 2, 1));
Hw = kron(eye(N+1),repmat(eye(2), 2, 1));
for i = 1:(M-1)
    v = zeros(M-1,1);
    v(i) = 1;
    H_M = kron(eye(N+1),kron(v, -1*d));
    H_Mw = kron(eye(N+1),kron(v, -1*eye(2)));
    
    H =  [H, H_M];
    Hw = [Hw,H_Mw];
end

%H = [kron(eye(N+1),repmat(d, 2, 1)),kron(eye(N+1),kron(kron_mat,d)) ];
%Hw = [kron(eye(N+1),repmat(eye(2), 2, 1)),kron(eye(N+1),kron(kron_mat,eye(2)))];

% get the indices of non-zero values in new A_ineq
eta_part = diag(ones(N, 1), 1);
eta_part(end,:)=[];

etaM_k = kron(eye(M-1),[1 1]);
eta_M = kron(eta_part,etaM_k);

A_ineq_form = eta_M * Hw;
[row,col,v] = find(A_ineq_form);
idx = sub2ind(size(A_ineq_form), row, col);
%% The MPC Problem
% Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))

% - quadratic objective for prediction
P = blkdiag( kron(speye(N), Q ), QN, kron(speye(N), R) );
P_new = blkdiag( kron(speye(N), Q+ M*rho/2*posM ), QN+M*rho/2*posM, kron(speye(N), R) );
% - linear dynamics
Ax = kron(speye(N+1), -speye(nx)) + kron(sparse(diag(ones(N, 1), -1)), A);
Bu = kron([sparse(1, N); speye(N)], B);
Aeq = [Ax, Bu];
Aeq = [Aeq;zeros(N*nu,(N+1)*nx),eye(N*nu)];
% quadratic objective for coordination

Pc = rhoM;
% for i = 1:(M-1)
%     Pc = blkdiag(Pc,1*rhoM);
% end
Pc = kron(eye(M),rhoM);
%% Setup 

% prediction

for i = 1:M
    
    %- linear objective for prediction
    %q(:,i) = prediction_linear(lambda(:,i),lambda_from_j{i},w(:,i),w_from_j{i},rho,r(i,:)',Q,QN,N,nu,M,posMN);
    q(:,i) = [repmat(-Q*r(i,:)', N, 1); -QN*r(i,:)'; zeros(N*nu, 1)];
    
    % input constraints
    umin = ones(nu,1)*-1;
    umax = ones(nu,1)*1;
    
    min_input = repmat(umin,N,1);
    max_input = repmat(umax,N,1);
    % - linear dynamics
    
    leq(:,i) = [-x0(i,:)'; zeros(N*nx, 1)];
    ueq(:,i) = leq(:,i);
    
    l_ineq_pred(:,i) = [leq(:,i);min_input];
    u_ineq_pred(:,i) = [ueq(:,i);max_input];

    % Create  OSQP objects for prediction
    prob(i) = osqp;
    
    % Setup workspace for prediction
    prob(i).setup(P_new, q(:,i), Aeq, l_ineq_pred(:,i), u_ineq_pred(:,i),'warm_start', true,'verbose',false);
    
    % Initial solution
    res(:,i) = prob(i).solve();
    x(:,i) = res(:,i).x(1:nx*(N+1));
    %w(:,i) = res(:,i).x(1:2*(N+1));   
end

% Coordination
for i = 1:M
    % linear dynamics
    upper_inf = ones(N*nu,1)*inf;
    u_ineq = upper_inf;
    
    % Create  OSQP objects for coordination
    coord(i) = osqp;
    
    % - linear objective for coordination
    qc(:,i) = coordination_linear(i,lambda(:,i),lambda_to_j{i},rho,N_j(i,:),M,x,posMN);
    
    [A_ineq{i},l_ineq(:,i)] = communicate(i,x,N,N_j(i,:),M,H,Hw,delta,nu);
    
    % Setup workspace for coordination
    coord(i).setup(Pc,qc(:,i),A_ineq_form,l_ineq(:,i),u_ineq,'warm_start', true,'verbose',false);
    
end

%% Algorithm
tic
for nit = 1:45
    
    %% Prediction
    for i = 1:M
        
        q(:,i) = prediction_linear(lambda(:,i),lambda_from_j{i},w(:,i),w_from_j{i},rho,r(i,:)',Q,QN,N,nu,M,posMN);
        prob(i).update('q',q(:,i));
        
        res(:,i) = prob(i).solve();
        x(:,i) = res(:,i).x(1:nx*(N+1));
    end     
    %% Communication (update) and Coordination
    for i = 1:M
        
        %- linear objective for coordination
        qc(:,i) = coordination_linear(i,lambda(:,i),lambda_to_j{i},rho,N_j(i,:),M,x,posMN);
        
        % Update matrices
        [A_ineq{i},l_ineq(:,i)] = communicate(i,x,N,N_j(i,:),M,H,Hw,delta,nu);
        %coord(i).setup(Pc,qc(:,i),A_ineq{i},l_ineq(:,i),u_ineq,'warm_start', true,'verbose',true);
        coord(i).update('Ax',A_ineq{i}(idx));
        coord(i).update('q',qc(:,i),'l',l_ineq(:,i),'u',u_ineq);
        
        resc(:,i) = coord(i).solve();

        w(:,i) = resc(:,i).x(1:2*(N+1));
        
        for j = 1:(M-1)
            w_to_j{i}(:,j) = resc(:,i).x(j*2*(N+1)+1:(j+1)*2*(N+1));
        end
        
    end     
    %% Mediation
    
     for i = 1:M
         lambda(:,i) = lambda(:,i) + rho * (posMN * x(:,i)-w(:,i));
         for j = 1:(M-1)
             lambda_to_j{i}(:,j) = lambda_to_j{i}(:,j) + rho * (posMN*x(:,N_j(i,j)) - w_to_j{i}(:,j));         
         end
     end
    %% Final Communication
    
     for i = 1:M
         for j = 1:(M-1)
             lambda_from_j{i}(:,j) = lambda_to_j{N_j(i,j)}(:,(N_j(N_j(i,j),:)==i));
             w_from_j{i}(:,j) = w_to_j{N_j(i,j)}(:,(N_j(N_j(i,j),:)==i));
         end
         
     end
    
 end
toc
%% Visualise

g = r';
xconcat =[];
wconcat = [];
for k = 1:N+1
    for i = 1:M
        
        xconcat = [xconcat;x((k-1)*nx+1:k*nx,i)];
        wconcat = [wconcat;w((k-1)*2+1:k*2,i);zeros(4,1)];
    end
end
%admm_visualise_osqp (g(:),xconcat,N,T,nx,nu)

% objective calculation
sum = 0;
for i = 1:M
    finx = x(nx+1:end,i);
    finu = res(:,i).x((N+1)*nx+1:end);
    finQ = blkdiag(kron(speye(N-1), Q),QN);
    finR = kron(speye(N), R);
    finr = repmat(r(i,:)', N, 1);

    sum = sum+(finx-finr)'*finQ*(finx-finr)+finu'*finR*finu;
end


%% Functions
                                   
function [A_ineq,l_ineq] = communicate(i,x,N,N_j,M,H,Hw,delta,nu)
    
    % placeholders
    A_ineq = zeros(N*nu,(N+1)*nu*M);
    l_ineq = zeros(N*2,1);
    
    x_bar = x(:,i);

    for j = 1:(M-1)
        x_bar = [x_bar;x(:,N_j(j))];
    end

    delta_x_bar = H * x_bar;

    for k = 2:N+1
        
        eta_M_k = zeros((M-1),nu*(M-1)); % placeholder
        delta_x_k = delta_x_bar((k-1)*2*(M-1)+1:k*2*(M-1)); % get k-th delta_x

        for m = 1:(M-1)

            x_bar_norm = norm(delta_x_k((m-1)*2+1:m*2)); % get the 2 norm of x_bar
            eta_ij_k = delta_x_k((m-1)*2+1:m*2)' * 1/x_bar_norm; % formulate eta ij k
            eta_M_k(m,(m-1)*nu+1:(m-1)*nu+nu) = eta_ij_k; % populate matrix eta_M

            l_ineq((k-2)*2+m) = delta + eta_ij_k * delta_x_k((m-1)*2+1:m*2) - x_bar_norm; % fill in l_ij

        end
        
        A_ineq( (k-2)*nu+1 : (k-1)*nu , :) = eta_M_k * Hw((k-1)*(M-1)*2+1:(k)*(M-1)*2,:);
        
    end
    

end

function qc = coordination_linear(i,lambda,lambda_to_j,rho,N_j,M,x,V)


    qc = -0.5 * lambda - rho/2 * V * x(:,i);


    for j = 1:(M-1)

        qc = [qc;( -0.5 * lambda_to_j(:,j) - rho/2 * V * x(:,N_j(j)))]; 

    end

end

function q = prediction_linear(lambda,lambda_from_j,w,w_from_j,rho,r,Q,QN,N,nu,M,V)

    
    q = -(rho/2 * w' * V)' + 0.5*(lambda'*V)';

    sigma_j = 0;
    for j = 1:(M-1)
        sigma_j = sigma_j + 0.5 * (lambda_from_j(:,j)' * V)' - (rho/2 * (w_from_j(:,j))' * V)';
    end

    q=q+[repmat(-Q*r, N, 1); -QN*r] + sigma_j;
    q = [q;zeros(N*nu, 1)];

end

