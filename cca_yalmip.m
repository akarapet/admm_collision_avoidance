% *****************************************************************************
% *                                                                           *
% *		 Centralised Collision Avoidance with CF Dynamics - YALMIP	          *
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

A = [1 0 0.09629 0 0 0.03962;
         0 1 0 0.09629 -0.03962 0;
         0 0 0.8943 0 0 0.7027;
         0 0 0 0.8943 -0.7027 0;
         0 0 0 0.1932 0.4524 0;
         0 0 -0.1932 0 0 0.4524];
B = [0.003709 0; 0 0.003709;0.1057 0;0 0.1057;0 -0.1932;0.1932 0];

nx = 6; % Number of states
nu = 2; % Number of inputs

% MPC data
N = 100;
Q = eye(nu)*10;
R = eye(nu)*13;


M = 3; % Number of agents
delta = 0.0; % Inter-agent distance 
 
% initialize the states, reference, control and nominal states
for i = 1:M
    
    x(i,:) = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
    a(i,:) = sdpvar(repmat(nu,1,N),repmat(1,1,N));  
end

% r(1,:) =[0.6,1.2];
% r(2,:) =[0,0.6];
% r(3,:) =[0,0.6];
r(1,:) =[1.0,1.4];
r(2,:) =[0.6,0.4];
r(3,:) =[0,0.9];
% initial conditions

% x_1_0 = [0.6;0;0;0;0;0];
% x_2_0 = [1.2;0.612;0;0;0;0];
% x_3_0 = [1.2;1.5;0;0;0;0];
x_1_0 = [0;0.5;0;0;0;0];
x_2_0 = [0.2;0.9;0;0;0;0];
x_3_0 = [1.0;0.3;0;0;0;0];

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
        
        constraints = [constraints, x{i,k+1} == A*x{i,k} + B*a{i,k}, x{1,1} == x_1_0,x{2,1} == x_2_0,x{3,1} == x_3_0];       
    end
   
end

 optimize(constraints,objective);
 oldobjective = value(objective);
    for i =1:M
        
    for k = 1:N

        ref{i,k} = value(x{i,k}(1:nu));
       
    end
    
    end 
 
   
    
%% Definition
for m = 1:6  
    
constraints = [];
objective = 0;

    for i =1:M
      for k = 1:N
      x_nominal{i,k} = value(x{i,k}(1:nu));  
      end
    end 

for k = 1:N
   

   
    for i = 1:M
        
        objective = objective  + (x{i,k}(1:nu) - r(i,:)')'* Q * ...
            (x{i,k}(1:nu)-r(i,:)') + a{i,k}'* R * a{i,k};
        
        constraints = [constraints, x{i,k+1} == A*x{i,k} + B*a{i,k},-1 <= a{i,k}(1) <= 1,-1 <= a{i,k}(2) <= 1];
        
        for j = 1:M
        
            if(i~=j && k>2 )
                
              eta_ij = (x_nominal{i,k}(1:nu) - x_nominal{j,k}(1:nu)) * 1/norm(x_nominal{i,k}(1:nu) - x_nominal{j,k}(1:nu));
              h_ij = eta_ij'*((x{i,k}(1:nu) - x{j,k}(1:nu)) - (x_nominal{i,k}(1:nu) - x_nominal{j,k}(1:nu))) - delta ;  
              constraints = [constraints, norm(x_nominal{i,k}(1:nu) - x_nominal{j,k}(1:nu))+h_ij >= 0];
              
            end
        end
            
    end


end

    
    optimize([constraints, x{1,1} == x_1_0,x{2,1} == x_2_0,x{3,1} == x_3_0],objective,ops);
   
    m
    if (abs(oldobjective-value(objective)) < abs(oldobjective*0.005))
        break
    end
    oldobjective =  value(objective)
   
end



%% Visualisation

visualise (r,x,N,T);
