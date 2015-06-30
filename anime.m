% Wil and Madalyn
% Animation

% input [x,y,z,r,p,y

function anime()

    clear all; close all; clc;

    draw_quad();

end

function draw_quad(x,t)

persistent hFig;

% x = x(1);
% z = x(3);
% pitch = x(8);

x = 0;
z = 3;
pitch = pi/2;

f_fig_bound = 15;
r_fig_bound = -5;
t_fig_bound = 10;
b_fig_bound = -1;


base = 4;
rotor_height = 0.5;
blade_radius = .5;
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
plot(f_base(1), f_base(2), 'ro');
plot(r_base(1), r_base(2), 'ro');

% draw front rotor and blade
line([f_base(1) f_rot(1)],[f_base(2) f_rot(2)]);
line([f_blade_r(1) f_blade_f(1)],[f_blade_r(2) f_blade_f(2)]);

% draw rear rotor and blade
line([r_base(1) r_rot(1)],[r_base(2) r_rot(2)]);
line([r_blade_r(1) r_blade_f(1)],[r_blade_r(2) r_blade_f(2)]);

% draw ground
line([r_fig_bound f_fig_bound],[0 0], 'Color', [.3 .5 .1],'LineWidth', 5);

% draw wall
line([14 14], [0 10],'Color',[.75 .6 .5],'LineWidth', 5);


axis equal;
axis([r_fig_bound f_fig_bound b_fig_bound t_fig_bound]);
xlabel('Distance (m)');
ylabel('Distance (m)');
title(['time = ', num2str(t), ',  \phi = ', num2str(pitch*180/pi), ' deg.']);

drawnow;

end