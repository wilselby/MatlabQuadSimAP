% Wil Selby
% Washington, DC
% May 30, 2015

% This script outputs multiple plots for simulation data analysis


%% Plot Translational Positions

% X,Y,Z
if(1)
figure;
hold on;
subplot(1,3,1);
plot(Quad.t_plot,Quad.X_plot)
title('X Position');
xlabel('Time (s)');
ylabel('Distance (m)');
subplot(1,3,2);
plot(Quad.t_plot,Quad.Y_plot)
title('Y Position');
xlabel('Time (s)');
ylabel('Distance (m)');
subplot(1,3,3);
plot(Quad.t_plot,Quad.Z_plot)
title('Z Position');
xlabel('Time (s)');
ylabel('Distance (m)');
end

% Altitude/u1
if(1)
figure;
subplot(1,3,1);
plot(Quad.t_plot,Quad.Z_plot)
title('Measured Z Position');
xlabel('Time (s)');
ylabel('Distance (m)');
subplot(1,3,2);
plot(Quad.t_plot,Quad.Z_ref_plot)
title('Desired Z Position');
xlabel('Time (s)');
ylabel('Distance (m)');
subplot(1,3,3);
plot(Quad.t_plot,Quad.U1_plot)
title('Altitude Control (U1)');
xlabel('Time (s)');
ylabel('Control Input');
end

if(1)
figure;
subplot(1,3,1);
plot(Quad.t_plot,Quad.X_ddot_meas_plot)
title('Measured Xdd');
xlabel('Time (s)');
ylabel('Distance (m)');
subplot(1,3,2);
plot(Quad.t_plot,Quad.Y_ddot_meas_plot)
title('Measured Ydd');
xlabel('Time (s)');
ylabel('Distance (m)');
subplot(1,3,3);
plot(Quad.t_plot,Quad.Z_ddot_meas_plot)
title('Measured Zdd');
xlabel('Time (s)');
ylabel('Control Input');
end


%% Plot Rotational Positions

% Roll/u2
if(1)
figure;
subplot(1,3,1);
plot(Quad.t_plot,Quad.phi_plot)
title('Measured Roll');
xlabel('Time (s)');
ylabel('Angle (rad)');
subplot(1,3,2);
plot(Quad.t_plot,Quad.phi_ref_plot)
title('Desired Roll');
xlabel('Time (s)');
ylabel('Angle (rad)');
subplot(1,3,3);
plot(Quad.t_plot,Quad.U2_plot)
title('Roll Control (U2)');
xlabel('Time (s)');
ylabel('Control Input');
end

% Pitch/u3
if(0)
figure;
subplot(1,3,1);
plot(Quad.t_plot,Quad.theta_plot)
title('Measured Pitch');
xlabel('Time (s)');
ylabel('Angle (rad)');
subplot(1,3,2);
plot(Quad.t_plot,Quad.theta_ref_plot)
title('Desired Pitch');
xlabel('Time (s)');
ylabel('Angle (rad)');
subplot(1,3,3);
plot(Quad.t_plot,Quad.U3_plot)
title('Pitch Control (U3)');
xlabel('Time (s)');
ylabel('Control Input');
end

% Yaw/u4
if(0)
figure;
subplot(1,3,1);
plot(Quad.t_plot,Quad.psi_plot)
title('Measured Yaw');
xlabel('Time (s)');
ylabel('Angle (rad)');
subplot(1,3,2);
plot(Quad.t_plot,Quad.psi_ref_plot)
title('Desired Yaw');
xlabel('Time (s)');
ylabel('Angle (rad)');
subplot(1,3,3);
plot(Quad.t_plot,Quad.U4_plot)
title('Yaw Control (U4)');
xlabel('Time (s)');
ylabel('Control Input');
end
