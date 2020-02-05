% Discrete time model of a quadcopter
A = [1 0 0.09629 0 0 0.03962;
         0 1 0 0.09629 -0.03962 0;
         0 0 0.8943 0 0 0.7027;
         0 0 0 0.8943 -0.7027 0;
         0 0 0 0.1932 0.4524 0;
         0 0 -0.1932 0 0 0.4524];
B = [0.003709 0; 0 0.003709;0.1057 0;0 0.1057;0 -0.1932;0.1932 0];

T = 0.1;
M = 3; % Number of agents


% Make matrices for the whole centralised system
Ad = sparse(blkdiag(A,A,A));
Bd = sparse(blkdiag(B,B,B));

nx = M*6; % Number of states
nu = M*2; % Number of inputs


% MPC data
Q = eye(nu/M)*14;
R = eye(nu/M)*5;
Qf = Q; Qf(nx/M,nx/M) = 0;
[K,QN,e] = dlqr(A,B,Qf,R);

Q  = sparse(blkdiag(Qf,Qf,Qf));
R  = sparse(blkdiag(R,R,R));
QN = sparse(blkdiag(QN,QN,QN));

delta = 0.3; % Inter-agent distance 

% A_ineq matrix creation

d = [eye(2),zeros(nu/M,nu-nu/M)];
kron_mat = [ones(M-1,1),-1*eye(M-1) ];

AK_matrix = kron_mat;

for i = 2:M

    v = kron_mat(:, i-1);
    kron_mat(:, i-1) = kron_mat(:, i);
    kron_mat(:, i) = v;
    AK_matrix = [AK_matrix; kron_mat];
    
end

diff_matrix = kron(AK_matrix, d);


% Initial and reference states
r = [0.5;1;0;0;0;0; 0;0.5;0;0;0;0; 0;0.5;0;0;0;0];
x0 = [0.5;0;0;0;0;0;  1;0.5;0;0;0;0; 1;1.5;0;0;0;0;];


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
leq = [-Ad*x0; zeros(N*nx, 1)];
ueq = leq;

% - input and state constraints

% To be done

% - OSQP constraints
A = Aeq;
l = leq;
u = ueq;

% Create an OSQP object
prob = osqp;

% Setup workspace
prob.setup(P, q, A, l, u, 'warm_start', true);

res = prob.solve();
x = res.x(1:nx*(N+1));

% Simulate in closed loop
nsim = 1;
for i = 1 : nsim

    x_bar = x;
    delta_x_bar = (kron(eye(N+1),diff_matrix) * x_bar);
    A_ineq = eta_maker(delta_x_bar,N,M,nu,nx,diff_matrix);
    
end

%Visualise
%admm_visualise_osqp (r,res.x,N,T)

function A_ineq = eta_maker (delta_x_bar,N,M,nu,nx,diff_matrix)

    A_ineq = zeros(N*nu,N*nu*(M-1));
    
    for k = 1:N
        
        eta_M_k = zeros(M*(M-1),nu*(M-1));
        delta_x_k = delta_x_bar((k-1)*nu*(M-1)+1:k*nu*(M-1));
        
        for i = 1:nu
        
            eta_M_k(i,(i-1)*nu/M+1:(i-1)*nu/M+nu/M) = delta_x_k((i-1)*nu/M+1:i*nu/M)' ...
                * 1/norm(delta_x_k((i-1)*nu/M+1:i*nu/M));
        end
        
        A_ineq( (k-1)*nu+1 : k*nu , (k-1)*nx+1 : k*nx ) = eta_M_k*diff_matrix;
    end
end
