% Trajectory Stabilization
% Wil Selby
% 6.832 Term Project

function [x_non, x_rand, x_stab] = Traj_stabilization()

clear all; close all; clc;

%  parameters
global xdes m K1 K2 K3 Kl Kd l Jx Jy Jz g aviobj count T dt N x_final;

count = 1;

% dynamics dt
dt = 0.01; T = 2;
N = round(T/dt);
tfinal=0:dt:dt*N;

x_final = 2;
z_start = 1.0;  
z_final = 1.0;   
pitch_final = -pi/2;

xdes = [x_final 0 z_final 0 0 0 0 pitch_final 0 0 0 0]'; % the desired final state

m = 0.5;    % mass (kg)
K1 = 0.01;  % drag coefficient, negligible at low speeds
K2 = 0.01;  % drag coefficient, negligible at low speeds
K3 = 0.01;  % drag coefficient, negligible at low speeds
Kl = 0.5;   % lift coefficient
Kd = 0.01;  % drag coefficient
l = 0.25;   % arm length (m) make sure it matches animation 
Jx = 0.005;  % moment of inertia (kg*m^2)
Jy = 0.005;  % moment of inertia (kg*m^2)
Jz = 0.005;  % moment of inertia (kg*m^2)
g = 9.8;    % gravity (m/s)
rotor_r = 0.1;  %rotor radius in m

% Load open loop policy
load('u_nom.mat');
u_tape = u_nom;


dlmwrite('u_nom.txt',u_tape , 'precision', '%.6f');
   
     
u_tape(:,end+1) = 0; %For drawing purposes so x and u are same lengths

% Optimal Trajectory
load('x_nom.mat');
xfinal = x_nom;

dlmwrite('x_nom.txt',xfinal , 'precision', '%.6f');

%Record movie
%aviobj = avifile('quad-stabilization.avi');

% playback the learned policy
x = [0 0 z_start 0 0 0 0 0 0 0 0 0]';


% Draw nominal trajectory
  for i=1:N+1
    x_rand(:,i) = x; 
    %draw_quad((i-1)*dt,x);
    [xdot,dxdot_dx,dxdot_du] = dynamics(x,u_tape(:,i),1);
    x = x + xdot.*dt;
  end
  
% Draw time delayed control 
% since actual position is behing tape, fill tape end with zeros
u_tape(:,end:end+10) = 0;
x = [0 0 z_start 0 0 0 0 0 0 0 0 0]';
for i=1:N+1
    x_delay(:,i) = x; 
    %draw_quad((i-1)*dt,x);
    if i >= 20
       i= i-19;
    end
    [xdot,dxdot_dx,dxdot_du] = dynamics(x,u_tape(:,i),0);
    x = x + xdot.*dt;
end


% Find time varying A and B
for i=1:N+1
    [xdot,dxdot_dx,dxdot_du] = dynamics(xfinal(:,i),u_tape(:,i),0);
    A(:,12*(i-1)+1:12*i) = dxdot_dx*dt + eye(12); 
    B(:,4*(i-1)+1:4*i) = dxdot_du*dt;
end
         
% Solve Riccati Eqn for S(t) recursively
[Q,R,Qend] = get_QR;
Q = Q.*dt; R = R*eye(4).*dt;  Qend = Qend.*dt;

%Terminal condition
S(:,12*(N-1)+1:12*N) = Qend; 

%Recursive
for t = N:-1:2  
    S_n = S(:,12*(t-1)+1:12*t);         % Get S_n(t)
    A_n = A(:,12*((t-1)-1)+1:12*(t-1)); % Get the A(t-1)
    B_n = B(:,4*((t-1)-1)+1:4*(t-1));   % Get the B(t-1)
    
    % Solve for S_n(t-1)
    S(:,12*((t-1)-1)+1:12*(t-1)) = A_n'*(S_n-S_n*B_n*inv(B_n'*S_n*B_n+R)*B_n'*S_n)*A_n + Q;
   
end

S(:,12*((N+1)-1)+1:12*(N+1)) = zeros(12);
S(:,12*((N+2)-1)+1:12*(N+2)) = zeros(12);

% playback the learned policy
x = [0 0 z_start 0 0 0 0 0 0 0 0 0]';

% Draw nominal trajectory with stabilization
for i=1:N+1
    x_stab(:,i) = x; 
    %draw_quad((i-1)*dt,x);
    
    % Apply trajectory stabilization
    x_err(:,i) = x-xfinal(:,i);
    S_n = S(:,12*((i+1)-1)+1:12*(i+1));     % Get S_n(t+1)
    A_n = A(:,12*(i-1)+1:12*i);             % Get the A(t)
    B_n = B(:,4*(i-1)+1:4*i);               % Get the B(t)
    K = inv(B_n'*S_n*B_n+R)*B_n'*S_n*A_n;    % Get gain matrix K(t)
    u_tape(:,i) = u_tape(:,i)-K*x_err(:,i);
    
    % Upadte dnamics
    [xdot,dxdot_dx,dxdot_du] = dynamics(x,u_tape(:,i),1);
    x = x + xdot.*dt;
end

% dlmwrite('k_tape.txt',K , 'precision', '%.6f');

% playback the learned policy
x = [0+.2 0 z_start-.2 0 0 0 0 0 0 0 0 0]';

% Draw nominal trajectory with stabilization
for i=1:N+1
    x_IC1(:,i) = x; 
    %draw_quad((i-1)*dt,x);
    
    % Apply trajectory stabilization
    x_err(:,i) = x-xfinal(:,i);
    S_n = S(:,12*((i+1)-1)+1:12*(i+1));     % Get S_n(t+1)
    A_n = A(:,12*(i-1)+1:12*i);             % Get the A(t)
    B_n = B(:,4*(i-1)+1:4*i);               % Get the B(t)
    K = inv(B_n'*S_n*B_n+R)*B_n'*S_n*A_n;    % Get gain matrix K(t)
    u_tape(:,i) = u_tape(:,i)-K*x_err(:,i);
    
    % Upadte dnamics
    [xdot,dxdot_dx,dxdot_du] = dynamics(x,u_tape(:,i),0);
    x = x + xdot.*dt;
end

% playback the learned policy
x = [0-.2 0 z_start+.3 0 0 0 0 0 0 0 0 0]';

% Draw nominal trajectory with stabilization
for i=1:N+1
    x_IC2(:,i) = x; 
    %draw_quad((i-1)*dt,x);
    
    % Apply trajectory stabilization
    x_err(:,i) = x-xfinal(:,i);
    S_n = S(:,12*((i+1)-1)+1:12*(i+1));     % Get S_n(t+1)
    A_n = A(:,12*(i-1)+1:12*i);             % Get the A(t)
    B_n = B(:,4*(i-1)+1:4*i);               % Get the B(t)
    K = inv(B_n'*S_n*B_n+R)*B_n'*S_n*A_n;    % Get gain matrix K(t)
    u_tape(:,i) = u_tape(:,i)-K*x_err(:,i);
    
    % Upadte dnamics
    [xdot,dxdot_dx,dxdot_du] = dynamics(x,u_tape(:,i),0);
    x = x + xdot.*dt;
end

% Time delay with stabilization
x = [0 0 z_start 0 0 0 0 0 0 0 0 0]';
%zero out end of A,B,S time varying matrices

for i=1:N+1
    x_delay_stab(:,i) = x; 
    %draw_quad((i-1)*dt,x);
    if i >= 20
       i= i-19;
    end
    
    if i <=201
    % Apply trajectory stabilization
    x_err(:,i) = x-xfinal(:,i);
    S_n = S(:,12*((i+1)-1)+1:12*(i+1));     % Get S_n(t+1)
    A_n = A(:,12*(i-1)+1:12*i);             % Get the A(t)
    B_n = B(:,4*(i-1)+1:4*i);               % Get the B(t)
    K = inv(B_n'*S_n*B_n+R)*B_n'*S_n*A_n;    % Get gain matrix K(t)
    u_tape(:,i) = u_tape(:,i)-K*x_err(:,i);
    end
    
    %else use nominal u_tape with zeros
        
    
    % Upadte dnamics
    [xdot,dxdot_dx,dxdot_du] = dynamics(x,u_tape(:,i),0);
    x = x + xdot.*dt;
end

%aviobj = close(aviobj);



% % Plot headwind
% figure(1);
% subplot(2,1,1)
% plot(x_rand(1,:),x_rand(3,:)); 
% hold on;
% plot(x_stab(1,:),x_stab(3,:),'r');
% plot(xfinal(1,:),xfinal(3,:),'g');
% plot(3,1.5,'ro');
% title('Stabilization With Constant "Headwind" Disturbance');
% ylabel('Altitude (m)');
% xlabel('Distance (m)');
% % legend('Unstabilized Trajectory','Stabilized Trajectory','Nominal Trajectory','Desired Position');
% 
% subplot(2,1,2);
% plot(x_rand(1,:),x_rand(8,:)); 
% hold on;
% plot(x_stab(1,:),x_stab(8,:),'r');
% plot(xfinal(1,:),xfinal(8,:),'g');
% plot(3,-pi/2,'ro');
% ylabel('Pitch Angle (rad)');
% xlabel('Distance (m)');
% legend('Unstabilized Trajectory','Stabilized Trajectory','Nominal Trajectory','Desired Position');
% 
% % Plot different ICs
figure(2);
subplot(2,1,1)
plot(xfinal(1,:),xfinal(3,:),'g');
hold on;
plot(x_IC1(1,:),x_IC1(3,:)); 
plot(x_IC2(1,:),x_IC2(3,:),'r');
plot(3,1.5,'ro');
title('Stabilization With Varying Initial Conditions');
ylabel('Altitude (m)');
xlabel('Distance (m)');
% legend('Nominal Trajectory','Initical Conidtion 1','Initial Condistion 2','Desired Position');

subplot(2,1,2);
plot(xfinal(1,:),xfinal(8,:),'g');
hold on;
plot(x_IC1(1,:),x_IC1(8,:)); 
plot(x_IC2(1,:),x_IC2(8,:),'r');
plot(3,-pi/2,'ro');
ylabel('Pitch Angle (rad)');
xlabel('Distance (m)');
legend('Nominal Trajectory','Initial Condition 1','Initial Condition 2','Desired Position');

% Plot time delay
% figure(3);
% subplot(2,1,1)
% plot(xfinal(1,:),xfinal(3,:),'g');
% hold on;
% plot(x_delay(1,:),x_delay(3,:)); 
% plot(x_delay_stab(1,:),x_delay_stab(3,:),'r');
% plot(3,1.5,'ro');
% title('Stabilization With Time Delay');
% ylabel('Altitude (m)');
% xlabel('Distance (m)');
% legend('Nominal Trajectory','Initical Conidtion 1','Initial Condistion 2','Desired Position');
% 
% subplot(2,1,2);
% plot(xfinal(1,:),xfinal(8,:),'g');
% hold on;
% plot(x_delay(1,:),x_delay(8,:));
% plot(x_delay_stab(1,:),x_delay_stab(8,:),'r');
% plot(3,-pi/2,'ro');
% ylabel('Pitch Angle (rad)');
% xlabel('Distance (m)');
% legend('Nominal Trajectory','Time Delay','Stabilized','Desired Position');


end


% ============================================================
% Quadrotor dynamics
% ===========================================================
function [xdot, dxdot_dx, dxdot_du] = dynamics(x,u,dist)
global xdes m K1 K2 K3 Kl Kd l Jx Jy Jz g rotor_r;

r = x(7);   % roll angle
p = x(8);   % pitch angle
y = x(9);   % yaw angle
w = xdes(1) - x(1); % distance to wall
C = 1;      % ground effect constant (keep = 1, unrealistic otherwise)

% In Ground Effect for actions close to the wall
if w <= (2*rotor_r)
    R = w/rotor_r;
    u(1) =  C*u(1)./(1-(1./(4*R).^2)); 
end

xdot = [x(4);
        x(5);
        x(6);
        -(K1/m)*x(4) + (u(1)/m)*(cos(r)*sin(p)*cos(y)+sin(r)*sin(y));
        -(K2/m)*x(5) + (u(1)/m)*(cos(r)*sin(p)*sin(y)-sin(r)*cos(y));
        -(K3/m)*x(6) + (u(1)/m)*(cos(r)*cos(p)) - g;
        x(10);
        x(11);
        x(12);
        -((l*Kl)/Jx)*x(10) + (l/Jx)*u(2);
        -((l*Kl)/Jy)*x(11) + (l/Jy)*u(3);
        -(Kd/Jz)*x(12) + u(4)/Jz];

% Add "random" noise to states
if(dist)
    %xdot(3) = xdot(3) + .1*rand(1);
    %xdot(8) = xdot(8) + .1*rand(1);
    xdot(4) = xdot(4)+(-0.1/m);
end

%COMPUTE GRADIENTS d(xdot)/dx, d(xdot)/du

dxdr = (cos(r)*sin(y)-sin(p)*sin(r)*cos(y))*u(1)/m;
dxdp = (cos(r)*cos(p)*cos(y))*u(1)/m;
dxdy = (cos(y)*sin(r)-sin(p)*sin(y)*cos(r))*u(1)/m;

dydr = (-cos(r)*cos(y)-sin(p)*sin(r)*sin(y))*u(1)/m;
dydp = (cos(r)*cos(p)*sin(y))*u(1)/m;
dydy = (cos(y)*sin(p)*cos(r)+sin(y)*sin(r))*u(1)/m;

dzdr = (cos(p)*sin(r))*u(1)/m;
dzdp = (sin(r)*cos(p))*u(1)/m;
dzdy = 0;

dxdot_dx = [ 0 0 0 1 0 0 0 0 0 0 0 0;  %FILL IN
         0 0 0 0 1 0 0 0 0 0 0 0;
         0 0 0 0 0 1 0 0 0 0 0 0;
         0 0 0 -K1/m 0 0 dxdr dxdp dxdy 0 0 0;
         0 0 0 0 -K2/m 0 dydr dydp dydy 0 0 0;
         0 0 0 0 0 -K3/m dzdr dzdp dzdy 0 0 0;
         0 0 0 0 0 0 0 0 0 1 0 0;
         0 0 0 0 0 0 0 0 0 0 1 0;
         0 0 0 0 0 0 0 0 0 0 0 1;
         0 0 0 0 0 0 0 0 0 -l*Kl/Jx 0 0; 
         0 0 0 0 0 0 0 0 0 0 -l*Kl/Jy 0;
         0 0 0 0 0 0 0 0 0 0 0 -Kd/Jz]; 
     
dxdu1 = (cos(r)*sin(p)*cos(y)+sin(r)*sin(y))/m;
dydu1 = (cos(r)*sin(p)*sin(y)-sin(r)*cos(y))/m;
dzdu1 = (cos(r)*cos(p))/m;
     
dxdot_du = [0 0 0 0;
        0 0 0 0;
        0 0 0 0;
        dxdu1 0 0 0;
        dydu1 0 0 0;
        dzdu1 0 0 0;
        0 0 0 0;
        0 0 0 0;
        0 0 0 0;
        0 l/Jx 0 0;
        0 0 1/Jy 0;
        0 0 0 1/Jz];

end

% ===============================================================
% This is the draw function
%================================================================
function draw_quad(t,X)
global aviobj count x_plot z_plot x_final;

persistent hFig;

x = X(1);
z = X(3);
pitch = -X(8);  % Matches system dynamics


f_fig_bound = 3.5;
r_fig_bound = -2;
t_fig_bound = 3.5;
b_fig_bound = -0.5;


base = .5;
rotor_height = 0.05;
blade_radius = .1;
alpha = atan(rotor_height/base); % angle from cm to rotor
h = base/cos(alpha);             % distace from cm to rotor

f_base = [x + base/2*cos(pitch), z + base/2*sin(pitch)];
r_base = [x - base/2*cos(pitch), z - base/2*sin(pitch)];

f_rot = [f_base(1) - rotor_height*cos(pi/2-pitch), f_base(2) + rotor_height*sin(pi/2-pitch)];
r_rot = [r_base(1) - rotor_height*cos(pi/2-pitch), r_base(2) + rotor_height*sin(pi/2-pitch)];

f_blade_f = [ f_rot(1) + blade_radius*cos(pitch), f_rot(2) + blade_radius*sin(pitch)];
f_blade_r = [ f_rot(1) - blade_radius*cos(pitch), f_rot(2) - blade_radius*sin(pitch)];

r_blade_f = [ r_rot(1) + blade_radius*cos(pitch), r_rot(2) + blade_radius*sin(pitch)];
r_blade_r = [ r_rot(1) - blade_radius*cos(pitch), r_rot(2) - blade_radius*sin(pitch)];


if (isempty(hFig))
    hFig = figure(25);
    set(hFig,'DoubleBuffer','on');
end

figure(hFig);
clf;

% draw base
line([r_base(1) f_base(1)],[r_base(2) f_base(2)]);
hold on;
% plot(f_base(1), f_base(2), 'ro');
% plot(r_base(1), r_base(2), 'ro');

% draw front rotor and blade
line([f_base(1) f_rot(1)],[f_base(2) f_rot(2)]);
line([f_blade_r(1) f_blade_f(1)],[f_blade_r(2) f_blade_f(2)]);

% draw rear rotor and blade
line([r_base(1) r_rot(1)],[r_base(2) r_rot(2)]);
line([r_blade_r(1) r_blade_f(1)],[r_blade_r(2) r_blade_f(2)]);

% draw ground
line([r_fig_bound f_fig_bound],[0 0], 'Color', [.3 .5 .1],'LineWidth', 5);

% draw wall
line([x_final x_final], [0 20],'Color',[.75 .6 .5]); %,'LineWidth', 2

% draw trajectory
% x_plot(1,count) = x;
% z_plot(1,count) = z;
% plot(x_plot(1:end),z_plot(1:end),'+');
% count = count + 1;


axis equal;
axis([r_fig_bound f_fig_bound b_fig_bound t_fig_bound]);
xlabel('Distance (m)');
ylabel('Distance (m)');
title(['time = ', num2str(t), ',  \phi = ', num2str(-pitch*180/pi), ' deg.']);

drawnow;

F = getframe(hFig);
aviobj = addframe(aviobj,F);

end

% ============================================================
% Returns the cost matrices
% ============================================================
function [Q,R,Qend] = get_QR

%FILL IN PENALTY MATRICES
Q = 1*ones(12,12); 
% Q(1,1) = 1e7;      % x pos
% Q(3,3) = 0;      % z pos
% Q(4,4) = 0;      % x vel
% Q(6,6) = 0;      % z vel
% Q(8,8) = 0;      % pitch angle
% Q(11,11) = 0;    % pitch angular velocity

R = [1/20];

Qend = zeros(12,12);
Qend(1,1) = 1000;      % x pos
Qend(3,3) = 100;       % z pos
Qend(4,4) = 0;      % x vel
Qend(6,6) = 2.5;      % z vel
Qend(8,8) = 1000;      % pitch angle
Qend(11,11) = 50;    % pitch angular velocity

end

% ============================================================
% Matrix Time Varying Riccati Differential Equaion
% ============================================================
function dSdt = mRiccati(t,S,A,B,Q,R)

t = 50*t+1;
if(t == 101)
    A=zeros(12);
    B=zeros(12);
else
    A = A(:,12*(t-1)+1:12*t); %Get the A(t)
    B = B(:,4*(t-1)+1:4*t);   % Get the B(t)
end
S = reshape(S, size(A)); %Convert from "n^2"-by-1 to "n"-by-"n"

dSdt = -(Q - S*B*inv(R)*B.'*S + S*A + A.'*S); %Determine derivative

dSdt = dSdt(:); %Convert from "n"-by-"n" to "n^2"-by-1
end







