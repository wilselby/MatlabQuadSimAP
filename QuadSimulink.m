%-----------------------------------------------------------------------%
%                                                                       %
%   This script simulates quadrotor dynamics and implements a control   %                                %
%   algrotihm using the Simulnk Block Diagram file QuadrotorSimulink.mdl                                                        %
%   Developed by: Wil Selby                                             %
%                                                                       %
%                                                                       %
%-----------------------------------------------------------------------%
% Add Paths
addpath utilities

%% Initialize Workspace
clear all;
% close all;
clc;

global Quad;

%% Initialize the plot
init_plot;
plot_quad_model;

%% Initialize Variables
quad_variables;

%% Run Simulation

SimOut = sim('QuadrotorSimulink');

%% Run The Simulation Loop
for S = 1 : 1 : size(SimOut,1)  
    
    Quad.X = X_out.signals.values(S);
    Quad.Y = Y_out.signals.values(S);
    Quad.Z = Z_out.signals.values(S);
    Quad.phi = Phi_out.signals.values(S);
    Quad.theta = Theta_out.signals.values(S);
    Quad.psi = Psi_out.signals.values(S);
  
    % Plot the Quadrotor's Position

        plot_quad         
%         campos([A.X+2 A.Y+2 A.Z+2])
%         camtarget([A.X A.Y A.Z])
%         camroll(0);
        drawnow;
  
end

%% Plot Data
figure();
plot(SimOut,X_out.signals.values)
hold on;
plot(SimOut,X_desired.signals.values);
title('X');

figure();
plot(SimOut,Y_out.signals.values)
hold on;
plot(SimOut,Y_desired.signals.values);
title('Y');

figure();
plot(SimOut,Z_out.signals.values)
hold on;
plot(SimOut,Z_desired.signals.values);
title('Z');

figure();
plot(SimOut,Psi_out.signals.values)
hold on;
plot(SimOut,Psi_desired.signals.values);
title('Psi');


