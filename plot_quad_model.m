% Wil Selby
% Washington, DC
% May 30, 2015

% This function sets the inital vertices and faces of the quadrotor arms
% and motors for drawing purposes. In this coordinate system, the front of
% the quadrotor is to the left and Motor 1 is the left motor.



%% Store data in global variable
load Quad_plotting_model

Quad.X_arm = patch('xdata',Quad.X_armX,'ydata',Quad.X_armY,'zdata',Quad.X_armZ,'facealpha',.9,'facecolor','b');
Quad.Y_arm = patch('xdata',Quad.Y_armX,'ydata',Quad.Y_armY,'zdata',Quad.Y_armZ,'facealpha',.9,'facecolor','b');
Quad.Motor1 = patch('xdata',Quad.Motor1X,'ydata',Quad.Motor1Y,'zdata',Quad.Motor1Z,'facealpha',.3,'facecolor','g');
Quad.Motor2 = patch('xdata',Quad.Motor2X,'ydata',Quad.Motor2Y,'zdata',Quad.Motor2Z,'facealpha',.3,'facecolor','k');
Quad.Motor3 = patch('xdata',Quad.Motor3X,'ydata',Quad.Motor3Y,'zdata',Quad.Motor3Z,'facealpha',.3,'facecolor','k');
Quad.Motor4 = patch('xdata',Quad.Motor4X,'ydata',Quad.Motor4Y,'zdata',Quad.Motor4Z,'facealpha',.3,'facecolor','k');




