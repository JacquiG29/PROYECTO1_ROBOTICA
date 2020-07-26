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

%joint_tags=["BackLbz","BackMby","BackUbx","NeckAy",...
%"LArmElx","LArmEly","LArmMwx","LArmShx","LArmUsy","LArmUwy",...
%"RArmElx","RArmEly","RArmMwx","RArmShx","RArmUsy","RArmUwy",...
%"LLegKny","LLegLax","LLegLhy","LLegMhx","LLegUay","LLegUhz",...
%"RLegKny","RLegLax","RLegLhy","RLegMhx","RLegUay","RLegUhz"];

%NO USAR ESTA FUNCION PORQUE ATLAS MUERE
%qss(j) = wb_motor_get_target_position(joint_tags(j))

% DEFINICIÃ“N DE TODAS LAS JUNTAS
juntas = 28;  % Cantidad de juntas en el robot

% Body Joints
back_lbz = wb_robot_get_device('BackLbz');%1
back_mby = wb_robot_get_device('BackMby');%2
back_ubx = wb_robot_get_device('BackUbx');%3
neck_ay = wb_robot_get_device('NeckAy');%4

% Left Arm Joints
l_arm_elx = wb_robot_get_device('LArmElx');%5
l_arm_ely = wb_robot_get_device('LArmEly');%6
l_arm_mwx = wb_robot_get_device('LArmMwx');%7
l_arm_shx = wb_robot_get_device('LArmShx');%8
l_arm_usy = wb_robot_get_device('LArmUsy');%9
l_arm_uwy = wb_robot_get_device('LArmUwy');%10

% Right Arm Joints
r_arm_elx = wb_robot_get_device('RArmElx');%11
r_arm_ely = wb_robot_get_device('RArmEly');%12
r_arm_mwx = wb_robot_get_device('RArmMwx');%13
r_arm_shx = wb_robot_get_device('RArmShx');%14
r_arm_usy = wb_robot_get_device('RArmUsy');%15
r_arm_uwy = wb_robot_get_device('RArmUwy');%16

% Left Leg Joints
l_leg_kny = wb_robot_get_device('LLegKny');%17
l_leg_lax = wb_robot_get_device('LLegLax');%18
l_leg_lhy = wb_robot_get_device('LLegLhy');%19
l_leg_mhx = wb_robot_get_device('LLegMhx');%20
l_leg_uay = wb_robot_get_device('LLegUay');%21
l_leg_uhz = wb_robot_get_device('LLegUhz');%22

% Right Leg Joints
r_leg_kny = wb_robot_get_device('RLegKny');%23
r_leg_lax = wb_robot_get_device('RLegLax');%24
r_leg_lhy = wb_robot_get_device('RLegLhy');%25
r_leg_mhx = wb_robot_get_device('RLegMhx');%26
r_leg_uay = wb_robot_get_device('RLegUay');%27
r_leg_uhz = wb_robot_get_device('RLegUhz');%28

joint_tags=[back_lbz,back_mby,back_ubx,neck_ay,...
    l_arm_elx,l_arm_ely,l_arm_mwx,l_arm_shx,l_arm_usy,l_arm_uwy,...
    r_arm_elx,r_arm_ely,r_arm_mwx,r_arm_shx,r_arm_usy,r_arm_uwy,...
    l_leg_kny,l_leg_lax ,l_leg_lhy,l_leg_mhx,l_leg_uay,l_leg_uhz,...
    r_leg_kny,r_leg_lax ,r_leg_lhy,r_leg_mhx,r_leg_uay,r_leg_uhz];

% CONSTANTES PARA CONTROLADOR
Kq = 0.5*diag([1, 1, 1, 1, ...
    1, 1, 1, 1, ...
    1, 1, 1, 1, ...
    1, 1, 1, 1, ...
    1, 1, 1, 1, ...
    1, 1, 1, 1, ...
    1, 1, 1, 1]);

Kq_p = 0.5*diag([1, 1, 1, 1, ...
    1, 1, 1, 1, ...
    1, 1, 1, 1, ...
    1, 1, 1, 1, ...
    1, 1, 1, 1, ...
    1, 1, 1, 1, ...
    1, 1, 1, 1]);

% VECTORES PARA CONTROLADOR
T_actual = zeros(juntas, 1);  % torque actual
q = zeros(juntas, 1);  % posiciÃ³n actual
q_p = zeros(juntas, 1);  % velocidad actual
qss = zeros(juntas, 1);  % psociÃ³n de referencia
q_p_ss = zeros(juntas, 1);  % velocidad de referencia
Tss = zeros(juntas, 1);  % Torque de referencia
u = zeros(juntas, 1);
U_hist = [];
q_hist = [];
qp_hist = [];

% DEFINIR POSICION INCIAL

% main loop:
% perform simulation steps of TIME_STEP milliseconds
% and leave the loop when Webots signals the termination
%
% DEFINIR POSICION Y TORQUE INCIAL
load('Params.mat');

N_2_target = 1000 / TIME_STEP;

%PARA QUE LLEGUE DE FORMA LENTA A LA POS. DESEADA


while wb_robot_step(TIME_STEP) ~= -1
    
    % read the sensors, e.g.:
    %  rgb = wb_camera_get_image(camera);
    
    for j = 1:juntas
      for n = 1:N_2_target
        ratio = n / N_2_target;
        wb_motor_set_position(joint_tags(j), qss(j) * ratio);
      end
    end

    
    for j = 1:juntas
        T_actual(j) = wb_motor_get_available_torque(joint_tags(j));
        q_p(j) = wb_motor_get_velocity(joint_tags(j));
        q(j) = q(j) + q_p(j)*TIME_STEP;
    end
    
     disp(q_p);
    % Position sensor enable
    
    u = -Kq*(q-qss) - Kq_p*(q_p) + Tss;
    
    for j = 1:juntas
        
        if u(j) > Tss(j)
            u(j) = Tss(j);
        elseif u(j) < -Tss(j)
            u(j) = -Tss(j);
        end
        
        %wb_motor_set_position(joint_tags(j), q(j));
        %wb_motor_set_torque(joint_tags(j), u(j));
    end
    
    %PARA GENERAR PARAMS.M
    %     for a = 1:juntas
    %         %qss(a) = wb_motor_get_target_position(joint_tags(a));
    %         Tss(a) = wb_motor_get_available_torque(joint_tags(a));
    %     end
    
    qp_hist = [qp_hist, q_p];
    U_hist = [U_hist, u];
    q_hist = [q_hist, q];
    save('el_u.mat', 'U_hist', 'q_hist', 'qp_hist','U_hist');
    %save('Params.mat', 'Tss', 'qss');
    
    %disp(T_actual);
    % Process here sensor data, images, etc.
    
    
    %CONTROLADOR
    
    
    
    % if your code plots some graphics, it needs to flushed like this:
    % drawnow;
    
end

% cleanup code goes here: write data to files, etc.
