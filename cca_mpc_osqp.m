% *****************************************************************************
% *                                                                           *
% *		 Centralised Collision Avoidance with CF Dynamics with MPC - OSQP	  *
% *				Aren Karapetyan (c) 19/05/2020							      *
% *	  Centralised Optimisation Problem  for Collision Avoidance        	      *
% *                                                                           *
% *****************************************************************************
% *                                                                           *
% *   Fourth Year Project at Engineering Science, University of Oxford        *
% *        Distributed Control of Flying Quadrotors                           *
% *****************************************************************************

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


% Augment the matrices for 3 agents
Ad = (blkdiag(A,A,A));
Bd = (blkdiag(B,B,B));

nx = M*6; % Number of states
nu = M*2; % Number of inputs

% MPC data
Q = eye(nu/M)*10;
Q = blkdiag(Q,eye(nu/M)*15); %15
R = eye(nu/M)*13;
Qf = Q; Qf(nx/M,nx/M) = 0;

% Get the terminal weight matrix QN by solving the discrete LQR equation
[K,QN,e] = dlqr(A,B,Qf,R);

% Initial and reference states
r  = [0.6;1.2;0;0;0;0; 0.0;0.6;0;0;0;0; 0;0.6;0;0;0;0];
x0 = [0.6;0;0;0;0;0;  1.2;0.6;0;0;0;0; 1.2;1.5;0;0;0;0;];

% r  = [1;1.4;0;0;0;0; 0.6;0.4;0;0;0;0; 0;0.9;0;0;0;0];
% x0 = [0;0.5;0;0;0;0;  0.2;0.9;0;0;0;0; 1.0;0.3;0;0;0;0;];

% Augment the matrices for the 3 agent problem
Q  = sparse(blkdiag(Qf,Qf,Qf));
R  = sparse(blkdiag(R,R,R));
QN = sparse(blkdiag(QN,QN,QN));

delta = 0.33; % Inter-agent distance 

% Transformation matrix V creation
d = [eye(2),zeros(nu/M,nx/M-nu/M)];
kron_mat = [ones(M-1,1),-1*eye(M-1) ];

AK_matrix = kron_mat;

for i = 2:M

    v = kron_mat(:, i-1);
    kron_mat(:, i-1) = kron_mat(:, i);
    kron_mat(:, i) = v;
    AK_matrix = [AK_matrix; kron_mat];
    
end

V = kron(AK_matrix, d);

% Prediction horizon
N = 10;

% Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))

% - quadratic objective
P = blkdiag( kron(speye(N), Q), QN, kron(speye(N), R) );

% - linear objective
q = [repmat(-Q*r, N, 1); -QN*r; zeros(N*nu, 1)];

% - linear dynamics
Ax = kron(speye(N+1), -speye(nx)) + kron(sparse(diag(ones(N, 1), -1)), Ad);
Bu = kron([sparse(1, N); speye(N)], Bd);
Aeq = [Ax, Bu];
leq = [-x0; zeros(N*nx, 1)];
ueq = leq;

% Create an OSQP object
prob = osqp;

% augmented non_zero matrix so that we setup the full (258 x 258) problem
G = [ones(nu,nu/M),zeros(nu,nx/M-nu/M)];
A_augment = kron([zeros(N,1),eye(N)],[G,G,G]);
A_augment(N*nu,N*nu+(N+1)*nx) = 0;

A_augment = [A_augment;zeros(N*nu,(N+1)*nx),eye(N*nu)];

% - input and state constraints
A = [Aeq;A_augment];

umin = ones(nu,1)*-1;
umax = ones(nu,1)*1;

lower_inf = ones(N*nu,1)*(-inf);
upper_inf = ones(N*nu,1)*inf;

min_input = repmat(umin,N,1);
max_input = repmat(umax,N,1);

l = [leq;lower_inf; min_input];
u = [ueq;upper_inf; max_input];

% get the indices of non-zero values in new A
[row,col,v] = find(A);
idx = sub2ind(size(A), row, col);

% Setup workspace
prob.setup(P, q, A, l, u,'warm_start', true,'verbose',true);


res = prob.solve();
oldobjective = res.info.obj_val;
x = res.x(1:nx*(N+1));

% Simulate in closed loop
nsim = 100;
nit = 2;
implementedX = x0; % a variable for storing the states for simulation
ctrl_applied_1 =[]; % agent 1
ctrl_applied_2 =[]; % agent 2
ctrl_applied_3 =[]; % agent 3

for i = 1 : nsim

    % the linearisation over previous solution x_bar
    for it = 1:nit
        
        x_bar = x; % save previous solution
        delta_x_bar = (kron(eye(N+1),V) * x_bar); % augmented delta x_bar 
        
        % Get the current A_ineq and l_ineq
        [A_ineq,l_ineq ]= eta_maker(delta_x_bar,N,M,nu,nx,V,delta);
        
        A_new = [Aeq;A_ineq];
        prob.update('Ax',A_new(idx));
        
        l_new = [leq;l_ineq;min_input];
        u_new = [ueq;upper_inf;max_input];
        prob.update('l',l_new,'u',u_new);
                 
        res = prob.solve();
        
        %  for termination with epsilon
        %         it
        %         if (abs(oldobjective-res.info.obj_val) < abs(oldobjective*0.005))
        %             break
        %         end
        %         oldobjective = res.info.obj_val;
        %
        x = res.x(1:nx*(N+1));
    end

    % Apply first control input to the plant
    ctrl = res.x((N+1)*nx+1:(N+1)*nx+nu);
    x0 = Ad*x0 + Bd*ctrl;
    % SEND INPUT TO RUNNING PYTHON CODE VIA ROS TO BE APPLIED TO CF
    ctrl_applied_1(i,1:nu/M) = ctrl(1:nu/M)';
    ctrl_applied_2(i,1:nu/M) = ctrl(nu/M+1:2*nu/M)';
    ctrl_applied_3(i,1:nu/M) = ctrl(2*nu/M+1:3*nu/M)';
    % RECEIVE THE CURRENT STATE OF THE CF FROM PYTHON CODE VIA ROS
    
    implementedX = [implementedX; x0];
    % Update initial state
    leq(1:nx) = -x0;
    ueq(1:nx) = -x0;
    prob.update('l', [leq;l_ineq;min_input],'u', [ueq;upper_inf;max_input])
    
end

% Write to text file to be read by Python for OL Control
% dlmwrite('testinputs1.txt',ctrl_applied_1);
% dlmwrite('testinputs2.txt',ctrl_applied_2);
% dlmwrite('testinputs3.txt',ctrl_applied_3);

%Visualise
%visualise_osqp (r,res.x,N,T,nx/M,nu/M) % for non-mpc
visualise_osqp(r,implementedX,nsim,T,nx/M,nu/M) % for mpc


% objective calculation

finx = res.x(nx+1:(N+1)*nx);
finu = res.x((N+1)*nx+1:end);
finQ = blkdiag(kron(speye(N-1), Q),QN);
finR = kron(speye(N), R);
finr = repmat(r, N, 1);

sum = (finx-finr)'*finQ*(finx-finr)+finu'*finR*finu;

function [A_ineq,l_ineq] = eta_maker (delta_x_bar,N,M,nu,nx,diff_matrix,delta)
    
    %placeholders
    A_ineq = zeros(N*nu,N*nu+(N+1)*nx);
    l_ineq = zeros(N*nu,1);
    
    for k = 2:N+1
        
        eta_M_k = zeros(M*(M-1),nu*(M-1)); % placeholder
        delta_x_k = delta_x_bar((k-1)*nu*(M-1)+1:k*nu*(M-1)); % get k-th delta_x
        
        for i = 1:nu
        
            x_bar_norm = norm(delta_x_k((i-1)*nu/M+1:i*nu/M)); % get the 2 norma of x_bar
            eta_ij_k = delta_x_k((i-1)*nu/M+1:i*nu/M)' * 1/x_bar_norm; % formulate eta
            eta_M_k(i,(i-1)*nu/M+1:(i-1)*nu/M+nu/M) = eta_ij_k; % populate matrix eta_M
            
            l_ineq((k-2)*nu+i) = delta + eta_ij_k * delta_x_k((i-1)*nu/M+1:i*nu/M) - x_bar_norm; % fill in l_ij
           
        end
        
        % Populate the inequality matrix, note that  (60 x 60) final part
        % of it will remain 0-s for the inputs
        A_ineq( (k-2)*nu+1 : (k-1)*nu , (k-1)*nx+1 : k*nx ) = eta_M_k*diff_matrix; 
        
       
    end
    
   A_ineq = [A_ineq;zeros(N*nu,(N+1)*nx),eye(N*nu)]; 
    
end
