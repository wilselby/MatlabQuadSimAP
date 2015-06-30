%-----------------------------------------------------------------------%
%                                                                       %
%   This script simulates quadrotor dynamics and implements a control   %                                %
%   algrotihm.                                                          %
%   Developed by: Wil Selby                                             %
%                                                                       %
%                                                                       %
%-----------------------------------------------------------------------%

%% Initialize Workspace
clear all;
close all;
clc;

global Quad;

%% Initialize the plot
init_plot;
plot_quad_model;

%% Initialize Variables
quad_variables;
quad_dynamics_nonlinear;   

%% Run The Simulation Loop
while Quad.t_plot(Quad.counter-1)< max(Quad.t_plot);    
    %% Measure Parameters (for simulating sensor errors)
    
    
    %% Filter Measurements
    
    
    %% Implement Controller
    outer_PID;
    inner_PID;
    
    %% Calculate Desired Motor Speeds
    quad_motor_speed;
    
    %% Update Position With The Equations of Motion
    quad_dynamics_nonlinear;    
    
    %% Plot the Quadrotor's Position
    if(mod(Quad.counter,3)==0)
        plot_quad 
        
%         campos([A.X+2 A.Y+2 A.Z+2])
%         camtarget([A.X A.Y A.Z])
%         camroll(0);
        Quad.counter;
        drawnow
    end
    
    Quad.init = 1;  %Ends initilization after first simulation iteration
end

%% Plot Data
plot_data
