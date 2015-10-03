% Wil Selby
% Washington, DC
% July 5, 2015

% This function simulates sensor measurement noise for the GPS, barometer,
% and IMU. The noise variances come from each sensor's respective
% datasheet. See www.wilselby.com for more information.

function sensor_meas

global Quad;

%% GPS Measurements
if(mod(Quad.counter,Quad.GPS_freq) == 0)
    Quad.X = Quad.X + randn(1)*Quad.X_error;
    Quad.Y = Quad.Y + randn(1)*Quad.Y_error;
    
    %% Barometer Measurements
    
    Quad.Z = Quad.Z + randn(1)*Quad.Z_error;
    
end
%% IMU Measurements

Quad.X_ddot = Quad.X_ddot + Quad.x_acc_bias + Quad.x_acc_sd*randn(1);
Quad.Y_ddot = Quad.Y_ddot + Quad.y_acc_bias + Quad.y_acc_sd*randn(1);
Quad.Z_ddot = Quad.Z_ddot + Quad.z_acc_bias + Quad.z_acc_sd*randn(1);

Quad.p = Quad.p + Quad.x_gyro_bias + Quad.x_gyro_sd*randn(1);
Quad.q = Quad.q + Quad.y_gyro_bias + Quad.y_gyro_sd*randn(1);
Quad.r = Quad.r + Quad.z_gyro_bias + Quad.z_gyro_sd*randn(1);

% Quad.phi_meas = Quad.phi + randn(1)*Quad.phi_error;
% Quad.theta_meas = Quad.theta + randn(1)*Quad.theta_error;
% Quad.psi_meas = Quad.psi + randn(1)*Quad.psi_error;


%% Plotting Variables
Quad.X_ddot_meas_plot(Quad.counter) = Quad.X_ddot;
Quad.Y_ddot_meas_plot(Quad.counter) = Quad.Y_ddot;
Quad.Z_ddot_meas_plot(Quad.counter) = Quad.Z_ddot;

% Quad.phi_meas_plot(Quad.counter) = Quad.phi_meas;
% Quad.phi_plot(Quad.counter) = Quad.phi;
% Quad.theta_meas_plot(Quad.counter) = Quad.theta_meas;
% Quad.theta_plot(Quad.counter) = Quad.theta;
% Quad.psi_meas_plot(Quad.counter) = Quad.psi_meas;
% Quad.psi_plot(Quad.counter) = Quad.psi;

end