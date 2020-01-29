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

x_0 = [0.5   1   1 ;
       0   0.5 1.5 ; 
       0     0   0 ;
       0     0   0];

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

 optimize(constraints,objective);

    for i =1:M
        
    for k = 1:N

        ref{i,k} = value(x{i,k}(1:nu));
       
    end
    
    end 
 
   
    
%% Definition
constraints = [];
objective = 0;

    for i =1:M     
      constraints = [constraints,x{i,N+1}(3:4) == [0;0],a{i,N} == [0;0]];
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

parameters_in = {x{1,1},x{2,1},x{3,1}};
solutions_out = {[a{1,1}],[a{2,1}],[a{3,1}]};
    
    %optimize([constraints, x{1,1} == x_1_0,x{2,1} == x_2_0,x{3,1} == x_3_0,x{1,N+1}(3:4) ==[0;0],x{2,N+1}(3:4) == [0;0],x{3,N+1}(3:4) == [0;0],a{1,N} == [0;0],a{2,N} == [0;0],a{3,N} == [0;0]],objective);
    %optimize(constraints,objective);
    controller = optimizer(constraints, objective,[],parameters_in,solutions_out);

x{1} = x_0(:,1);
x{2} = x_0(:,2);
x{3} = x_0(:,3);
for m = 1:100
    
    inputs = {x{1},x{2},x{3}};
    
    [solutions,diagnostics] = controller{inputs}; 

       
    x{1} = A*x{1} + B*solutions{1};
    x{2} = A*x{2} + B*solutions{2};
    x{3} = A*x{3} + B*solutions{3};
    for i =1:M
         
     implementedX{i,m} =  value(x{i,m}(1:nu));
     for k = 1:N
      x_nominal{i,k} = value(x{i,k}(1:nu));  
      end
    end 
   
  
end


%% Visualisation

admm_visualise (r,implementedX,N-1,T);
%admm_visualise([0.5;1.3],x,N,T);



