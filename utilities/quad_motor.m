% Wil Selby
% Washington, DC
% May 30, 2015

% This function maps the control inputs to desired motor speeds. These will
% be the actual commands sent  to the Electronic Speed Controllers from the
% Autopilot. The ESCs will then command each motor to rotate at the desired
% speed.

global Quad

% Calculate square of motor speeds based on desired control inputs
m1 = Quad.U1/(4*Quad.KT) - Quad.U3/(2*Quad.KT*Quad.l) - Quad.U4/(4*Quad.Kdx);
m2 = Quad.U1/(4*Quad.KT) - Quad.U2/(2*Quad.KT*Quad.l) + Quad.U4/(4*Quad.Kdx);
m3 = Quad.U1/(4*Quad.KT) + Quad.U3/(2*Quad.KT*Quad.l) - Quad.U4/(4*Quad.Kdx);
m4 = Quad.U1/(4*Quad.KT) + Quad.U2/(2*Quad.KT*Quad.l) + Quad.U4/(4*Quad.Kdx);

% Limit the motor speeds
if(abs(m1)>Quad.motor_max
    m1 = sign(m1)*Quad.motor_max;
end

if(abs(m2)>Quad.motor_max
    m2 = sign(m2)*Quad.motor_max;
end

if(abs(m3)>Quad.motor_max
    m3 = sign(m3)*Quad.motor_max;
end

if(abs(m4)>Quad.motor_max
    m4 = sign(m4)*Quad.motor_max;
end

% Calculate actual desired motoer speeds (radian/s)
Quad.O1 = sign(m1)*sqrt(abs(m1));
Quad.O2 = sign(m2)*sqrt(abs(m2));
Quad.O3 = sign(m3)*sqrt(abs(m3));
Quad.O4 = sign(m4)*sqrt(abs(m4));

Quad.Obar = Quad.O1 - Quad.O2 + Quad.O3 - Quad.O4;

Quad.O1_plot(Quad.counter) = Quad.O1;
Quad.O2_plot(Quad.counter) = Quad.O2;
Quad.O3_plot(Quad.counter) = Quad.O3;
Quad.O4_plot(Quad.counter) = Quad.O4;


% TODO convert back to U1 and plot see Forces.m




