% *****************************************************************************
% *                                                                           *
% *	                        ROS Tests	                                      *
% *				Aren Karapetyan (c) 19/05/2020							      *
% *                                                                           *	         	                                                           
% *                                                                           *
% *****************************************************************************
% *                                                                           *
% *   Fourth Year Project at Engineering Science, University of Oxford        *
% *        Distributed Control of Flying Quadrotors                           *
% *****************************************************************************

%% ROS Initialisation 
clear
clc
rosinit

nsim = 1

sub = rossubscriber('/chatter', 'geometry_msgs/Point',@callback_pos);
    %point = rossubscriber('/chatter');

 while (nsim<10) 
    pause(1)
    nsim = nsim+1
 end
 rosshutdown
function callback_pos(src,msg)

    fprintf('X: %f\n',msg.X)
    fprintf('Y: %f\n',msg.Y)
    fprintf('Z: %f\n',msg.Z) 

end

