% MATLAB controller for Webots
% File:          controlador_proy1.m
% Date:
% Description:
% Author:
% Modifications:

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
%desktop;
%keyboard;

TIME_STEP = 16;%Este valor debe de coincidir con el basicTimeStep?

% get and enable devices, e.g.:
%  camera = wb_robot_get_device('camera');
%  wb_camera_enable(camera, TIME_STEP);
%  motor = wb_robot_get_device('motor');

joint_tags=["BackLbz","BackMby","BackUbx","NeckAy",...
"LArmElx","LArmEly","LArmMwx","LArmShx","LArmUsy","LArmUwy",...
"RArmElx","RArmEly","RArmMwx","RArmShx","RArmUsy","RArmUwy",...
"LLegKny","LLegLax","LLegLhy","LLegMhx","LLegUay","LLegUhz",...
"RLegKny","RLegLax","RLegLhy","RLegMhx","RLegUay","RLegUhz"];


%Body Joints
back_lbz = wb_robot_get_device('BackLbz');
back_mby = wb_robot_get_device('BackMby');
back_ubx = wb_robot_get_device('BackUbx');
neck_ay = wb_robot_get_device('NeckAy');

%Left Arm Joints
l_arm_elx = wb_robot_get_device('LArmElx');
l_arm_ely = wb_robot_get_device('LArmEly');
l_arm_mwx = wb_robot_get_device('LArmMwx');
l_arm_shx = wb_robot_get_device('LArmShx');
l_arm_usy = wb_robot_get_device('LArmUsy');
l_arm_uwy = wb_robot_get_device('LArmUwy');

%Right Arm Joints
r_arm_elx = wb_robot_get_device('RArmElx');
r_arm_ely = wb_robot_get_device('RArmEly');
r_arm_mwx = wb_robot_get_device('RArmMwx');
r_arm_shx = wb_robot_get_device('RArmShx');
r_arm_usy = wb_robot_get_device('RArmUsy');
r_arm_uwy = wb_robot_get_device('RArmUwy');

%Left Leg Joints
l_leg_kny = wb_robot_get_device('LLegKny');
l_leg_lax = wb_robot_get_device('LLegLax');
l_leg_lhy = wb_robot_get_device('LLegLhy');
l_leg_mhx = wb_robot_get_device('LLegMhx');
l_leg_uay = wb_robot_get_device('LLegUay');
l_leg_uaz = wb_robot_get_device('LLegUhz');

%Right Leg Joints
r_leg_kny = wb_robot_get_device('RLegKny');
r_leg_lax = wb_robot_get_device('RLegLax');
r_leg_lhy = wb_robot_get_device('RLegLhy');
r_leg_mhx = wb_robot_get_device('RLegMhx');
r_leg_uay = wb_robot_get_device('RLegUay');
r_leg_uaz = wb_robot_get_device('RLegUhz');



% main loop:
% perform simulation steps of TIME_STEP milliseconds
% and leave the loop when Webots signals the termination
%
while wb_robot_step(TIME_STEP) ~= -1

  % read the sensors, e.g.:
  %  rgb = wb_camera_get_image(camera);

  % Process here sensor data, images, etc.

  % send actuator commands, e.g.:
  wb_motor_set_position(l_arm_shx, -1.3)
  wb_motor_set_position(r_arm_shx, 1.3);

  % if your code plots some graphics, it needs to flushed like this:
  drawnow;

end

% cleanup code goes here: write data to files, etc.
