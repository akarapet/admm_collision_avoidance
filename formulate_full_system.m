% *****************************************************************************
% *                                                                           *
% *		      Crazyflie Linearised System Formulator						  *
% *				Aren Karapetyan (c) 19/05/2020							      *
% *	  Creates a linearised model based on the controller.mdl         	      *
% *                                                                           *
% *****************************************************************************
% *                                                                           *
% *   Fourth Year Project at Engineering Science, University of Oxford        *
% *        Distributed Control of Flying Quadrotors                           *
% *****************************************************************************

%define parameters
cmd_2_newtons_conversion_quadratic_coefficient  =  1.3385e-10;
cmd_2_newtons_conversion_linear_coefficient     =  6.4870e-6;
nrotor_vehicle_thrust_max = 0.1597;
g=9.81;

%Get the linearised model of the onboard controller
[Ac,Bc,Cc,Dc] = linmod('controller');

K_sys = ss(Ac,Bc,Cc,Dc);

% Define the linearised model of the crazyflie
    % Specify that the input dimension is four, denoted "nu", this
    % corresponds to the total thrust and the angular rates about the
    % (x,y,z) axes of the body frame.
    nu = 4;

    % Specify that the state dimension for the outer loop, denoted "nx"
    nx = 9;
    %specify the mass of the crazyflie
    m = 0.029;
    % State transition matrix:
    Ag  =  [...
            0 0 0   1 0 0   0 0 0   ;...
            0 0 0   0 1 0   0 0 0   ;...
            0 0 0   0 0 1   0 0 0   ;...
                                             ...
            0 0 0   0 0 0   0 g 0   ;...
            0 0 0   0 0 0  -g 0 0   ;...
            0 0 0   0 0 0   0 0 0   ;...
                                             ...
            0 0 0   0 0 0   0 0 0   ;...
            0 0 0   0 0 0   0 0 0   ;...
            0 0 0   0 0 0   0 0 0   ;...
        ];



    % Input matrix:
    Bg  =  [...
            zeros(1,nu)         ;...
            zeros(1,nu)         ;...
            zeros(1,nu)         ;...
                                 ...
            zeros(1,nu)         ;...
            zeros(1,nu)         ;...
            [1/m,zeros(1,nu-1)]    ;...
                                 ...
            [0,1,0,0]           ;...
            [0,0,1,0]           ;...
            [0,0,0,1]           ;...
        ];
    
    % Output matrix
    Cg = eye(nx);

    % The feed-through matrix
    Dg = zeros(nx,nu);
    
G_sys = ss(Ag,Bg,Cg,Dg);

%OL_sys = series(K_sys,G_sys);

K_sys.InputName = 'e';  
K_sys.OutputName = 'u';

G_sys.InputName = 'u';  
G_sys.OutputName = 'y';

Sum = sumblk('e = r-y',9);

%formulate the closed loop system
CL_sys = connect(G_sys,K_sys,Sum,'r','y');

%discretize the system
CL_sysd = c2d(CL_sys,0.1);

TF = minreal(zpk(CL_sys));

tf_xdot_x = TF(1,4);
tf_ydot_y = TF(2,5);
tf_yaw_yaw = TF (9,9);



