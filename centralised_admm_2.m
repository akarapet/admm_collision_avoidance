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
Q = eye(nu)*14;
R = eye(nu)*5;


M = 3; % Number of agents
delta = 0.2; % Inter-agent distance 
 
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

x_1_0 = [0.5;0;0;0];
x_2_0 = [1;0.5;0;0];
x_3_0 = [1;1.5;0;0];
ops = sdpsettings('verbose',0);


x1 = linspace(x_1_0(1),r(1),N);
y1 = linspace(x_1_0(2),r(3),N);

x2 = linspace(x_2_0(1),r(1),N);
y2 = linspace(x_2_0(2),r(4),N);

constraints = [];
objective = 0;

for i = 1:M
   

    for k = 1:N

        objective = objective + (x{i,k}(1:nu)-r(i,:)')'* Q * ...
             (x{i,k}(1:nu)-r(i,:)')  + a{i,k}'* R * a{i,k};
        
        constraints = [constraints, x{i,k+1} == A*x{i,k} + B*a{i,k}, x{1,1} == x_1_0,x{2,1} == x_2_0,x{3,1} == x_3_0,x{1,N+1}(3:4) == [0;0],x{2,N+1}(3:4) == [0;0],x{3,N+1}(3:4) == [0;0],a{1,N} == [0;0],a{2,N} == [0;0],a{3,N} == [0;0]];       
    end
   
end

 optimize(constraints,objective);

    for i =1:M
        
    for k = 1:N

        ref{i,k} = value(x{i,k}(1:nu));
       
    end
    
    end 
 
   
    
%% Definition
for m = 1:3  
    
constraints = [];
objective = 0;

    for i =1:M
      for k = 1:N
      x_nominal{i,k} = value(x{i,k}(1:nu));  
      end
    end 

for k = 1:N
   

   
    for i = 1:M
        
        objective = objective  + (x{i,k}(1:nu) - ref{i,k})'* Q * ...
            (x{i,k}(1:nu)-ref{i,k}) + a{i,k}'* R * a{i,k};
        
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

    
    optimize([constraints, x{1,1} == x_1_0,x{2,1} == x_2_0,x{3,1} == x_3_0,x{1,N+1}(3:4) ==[0;0],x{2,N+1}(3:4) == [0;0],x{3,N+1}(3:4) == [0;0],a{1,N} == [0;0],a{2,N} == [0;0],a{3,N} == [0;0]],objective);
   
    m
       
end



%% Visualisation

admm_visualise (r,x,N,T);
%admm_visualise([0.5;1.3],x,N,T);



