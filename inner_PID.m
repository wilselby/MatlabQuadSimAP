% Wil Selby
% Washington, DC
% May 30, 2015

% This function implements a Proportional Integral Derivative Controller
% (PID) for the quadrotor. A lower level controller takes those inputs and 
% controls the error between the deisred and actual Euler angles. 

function inner_PID

global Quad


%% Z Position PD Controller/Altitude Controller

cp = Quad.Z_KP*(Quad.Z_des_GF-Quad.Z);         %Proportional term
cd = Quad.Z_KD*Quad.Z_dot;                  %Derivative term%Quad.U1 = (cp + cd + Quad.m*Quad.g)/(cos(Quad.theta)*cos(Quad.phi));
Quad.U1 = (cp + cd + Quad.m * Quad.g)/(cos(Quad.theta)*cos(Quad.phi));
% Quad.U1_plot(Quad.counter) = Quad.U1; %(TODO record after motor limits applied?)

%% Attitude Controller

% TODO - use p,q,r instead of dot
% Roll PD Controller
cp = Quad.phi_KP*(Quad.phi_des - Quad.phi);
cd = Quad.phi_KD*Quad.p;
Quad.U2 = cp + cd;
% Quad.U2 = 0;
% Quad.U2_plot(Quad.counter) = Quad.U2;

% Pitch PD Controller
cp = Quad.theta_KP*(Quad.theta_des - Quad.theta);
cd = Quad.theta_KD*Quad.q;
Quad.U3 = cp + cd;
% Quad.U3 = 0;
% Quad.U3_plot(Quad.counter) = Quad.U3;

% Roll PD Controller
cp = Quad.psi_KP*(Quad.psi_des - Quad.psi);
cd = Quad.psi_KD*Quad.r;
Quad.U4 = cp + cd;
% Quad.U4 = 0;
% Quad.U4_plot(Quad.counter) = Quad.U4;

end
















