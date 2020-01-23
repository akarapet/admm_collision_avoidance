%% Initialisation


yalmip('clear')
clear all

% Model data
T = 0.1;
A = [1 0 0 T 0 0;
     0 1 0 0 T 0;
     0 0 1 0 0 T;
     0 0 0 1 0 0;
     0 0 0 0 1 0
     0 0 0 0 0 1];
B = [0 0 0;0 0 0;0 0 0;T 0 0;0 T 0;0 0 T];




