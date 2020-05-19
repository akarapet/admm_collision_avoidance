% *****************************************************************************
% *                                                                           *
% *		 Centralised Collision Avoidance with Simple Dynamics - YALMIP	      *
% *				Aren Karapetyan (c) 19/05/2020							      *
% *	  Centralised Optimisation Problem  for Collision Avoidance        	      *
% *                                                                           *
% *****************************************************************************
% *                                                                           *
% *   Fourth Year Project at Engineering Science, University of Oxford        *
% *        Distributed Control of Flying Quadrotors                           *
% *****************************************************************************

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
delta = 0.35; % Inter-agent distance 
 
% initialize the states, reference, control and nominal states
for i = 1:M
    
    x(i,:) = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
    a(i,:) = sdpvar(repmat(nu,1,N),repmat(1,1,N));
 
end

r(1,:) =[1.0,1.4];
r(2,:) =[0.6,0.4];
r(3,:) =[0,0.9];

% initial conditions

x_0 = [0     0.2   1.0 ;
       0.5   0.9 0.3 ; 
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
 oldobjective = value(objective);
    for i =1:M
        
    for k = 1:N

        ref{i,k} = value(x{i,k}(1:nu));
       
    end
    
    end 
 
   
    
%% Definition
for m = 1:10 
    
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

    
    optimize(constraints,objective,ops);
    
    m
    if (abs(oldobjective-value(objective)) < abs(oldobjective*0.005))
        break
    end
    oldobjective =  value(objective) 
end


sum = 0;
for i =1:M
    for k = 1:N
        sum = sum + (value(x{i,k+1}(1:2))-r(i,:)')'*Q*(value(x{i,k+1}(1:2))-r(i,:)')+value(a{i,k})'*R*value(a{i,k});
    end
end
%% Visualisation

visualise (r,x,N,T);
