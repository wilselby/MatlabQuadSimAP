% Wil Selby
% Washington, DC
% May 30, 2015

% This function implements a Proportional Integral Derivative Controller
% (PID) for the quadrotor. A high level controller outputs desired roll and
% pitch angles based on errors between the Global and desired X and Y
% positions. A lower level controller takes those inputs and controls the
% error between the deisred and actual Euler angles. After the control
% inputs are calculated, the desired motor speeds are calculated. See
% www.wilselby.com for derivations of these equations.

function outer_PID

global Quad

%% High Level Translational Controller

% TODO - Rotation function from GF to BF?

% X Position PD controller (simple yaw rotation matrix assuming other angles
%near 0)
Quad.X_des = Quad.X_des_GF*cos(Quad.psi) + Quad.Y_des_GF*sin(Quad.psi); %Convert Deisred position from GF to BF)

Quad.X_BF = Quad.X*cos(Quad.psi) + Quad.Y*sin(Quad.psi); %Convert position from GF to BF)
Quad.X_BF_dot = Quad.X_dot*cos(Quad.psi) + Quad.Y_dot*sin(Quad.psi); %Convert velocity from GF to BF)

cp = Quad.X_KP*(Quad.X_des - Quad.X_BF);    %Proportional term
cd = Quad.X_KD*Quad.X_BF_dot;                     %Derivative term

Quad.theta_des =  cp + cd;

if(abs(Quad.theta_des) > Quad.theta_max)
    Quad.theta_des = sign(Quad.theta_des)*Quad.theta_max;
end

% Y Position PD controller (simple yaw rotation matrix assuming other angles
%near 0)
Quad.Y_des = Quad.Y_des_GF*cos(Quad.psi) - Quad.X_des_GF*sin(Quad.psi); %Convert Deisred position from GF to BF)

Quad.Y_BF = Quad.Y*cos(Quad.psi) - Quad.X*sin(Quad.psi); %Convert position from GF to BF)
Quad.Y_BF_dot = Quad.Y_dot*cos(Quad.psi) - Quad.X_dot*sin(Quad.psi); %Convert velocity from GF to BF)

cp =  Quad.Y_KP*(Quad.Y_des - Quad.Y_BF);    %Proportional term
cd = Quad.Y_KD*Quad.Y_BF_dot;                      %Derivative term

Quad.phi_des = cp + cd; 

if(abs(Quad.phi_des) > Quad.phi_max)
    Quad.phi_des = sign(Quad.phi_des)*Quad.phi_max;
end


end
















