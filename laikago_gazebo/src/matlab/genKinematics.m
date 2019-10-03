% constants
leg_offset_x = 0.21935;
leg_offset_y = 0.0875;
thigh_offset = 0.037;
thigh_length = 0.25;
calf_length = 0.25;

% variables
syms q_FR_hip q_FL_hip q_RR_hip q_RL_hip real
syms q_FR_thigh q_FL_thigh q_RR_thigh q_RL_thigh real
syms q_FR_calf q_FL_calf q_RR_calf q_RL_calf real
syms roll pitch yaw real
syms pos_x pos_y pos_z real

q_leg = containers.Map;
q_leg('FR') = [q_FR_hip, q_FR_thigh, q_FR_calf]';
q_leg('FL') = [q_FL_hip, q_FL_thigh, q_FL_calf]';
q_leg('RR') = [q_RR_hip, q_RR_thigh, q_RR_calf]';
q_leg('RL') = [q_RL_hip, q_RL_thigh, q_RL_calf]';

q_rot = [roll pitch yaw]';
q_pos = [pos_x pos_y pos_z]';
q_w = [q_rot; q_pos; q_leg('FR'); q_leg('FL'); q_leg('RR'); q_leg('RL')];
q_b = [q_leg('FR'); q_leg('FL'); q_leg('RR'); q_leg('RL')];

legs = {'FR', 'FL', 'RR', 'RL'};
isFront = [1, 1, -1, -1];
isLeft = [-1, 1, -1, 1];

file_folder = getFolderPath();
%%
% rotation matrix
rot_x = @(roll)[1, 0, 0;
    0, cos(roll), -sin(roll);
    0, sin(roll), cos(roll)];
rot_y = @(pitch)[cos(pitch), 0, sin(pitch);
    0, 1, 0;
    -sin(pitch), 0, cos(pitch)];
rot_z = @(yaw)[cos(yaw), -sin(yaw), 0;
    sin(yaw), cos(yaw), 0;
    0, 0, 1];

rot_body = rot_z(yaw)*rot_y(pitch)*rot_x(roll);

% world coordinate
p_origin = [0, 0, 0]';
p_trunk = p_origin + q_pos;

p_hip = containers.Map;
p_thigh = containers.Map;
p_calf = containers.Map;
p_foot = containers.Map;
p_feet = [];
p = [p_trunk];
p_str = {'trunk'};
for n = 1:4
    leg_str = legs{n};
    leg_q = q_leg(leg_str);
    p_hip(leg_str) = p_trunk + rot_body*[isFront(n)*leg_offset_x, isLeft(n)*leg_offset_y, 0]';
    p_thigh(leg_str) = p_hip(leg_str) + rot_body*rot_x(leg_q(1))*[0, isLeft(n)*thigh_offset, 0]';
    p_calf(leg_str) = p_thigh(leg_str) + rot_body*rot_x(leg_q(1))*rot_y(leg_q(2))*[0, 0, -thigh_length]';
    p_foot(leg_str) = p_calf(leg_str) + rot_body*rot_x(leg_q(1))*rot_y(leg_q(2))*rot_y(leg_q(3))*[0, 0, -calf_length]';
    p_feet = [p_feet; p_foot(leg_str)];
    p = [p, p_hip(leg_str), p_thigh(leg_str), p_calf(leg_str), p_foot(leg_str)];
    p_str = [p_str, {[leg_str, '_hip'], [leg_str, '_thigh'], [leg_str, '_calf'], [leg_str, '_foot']}];
end

matlabFunction(p,'Outputs',{'points'},...
    'File', [file_folder, '/gen/kinWorldPoints'],...
    'Vars', {q_w});
save([file_folder, '/gen/kinWorldPoints'], 'p_str');

J_feet = jacobian(p_feet, q_w);
J_feet = simplify(J_feet);

matlabFunction(p_feet, J_feet,'Outputs',{'p_feet', 'J_feet'},...
    'File', [file_folder, '/gen/kinWorldFeet'],...
    'Vars', {q_w});
save([file_folder, '/gen/kinWorldFeet'], 'legs');

% body coordinate
p_trunk = [0, 0, 0]';

p_hip = containers.Map;
p_thigh = containers.Map;
p_calf = containers.Map;
p_foot = containers.Map;
p_feet = [];
for n = 1:4
    leg_str = legs{n};
    leg_q = q_leg(leg_str);
    p_hip(leg_str) = p_trunk + [isFront(n)*leg_offset_x, isLeft(n)*leg_offset_y, 0]';
    p_thigh(leg_str) = p_hip(leg_str) + rot_x(leg_q(1))*[0, isLeft(n)*thigh_offset, 0]';
    p_calf(leg_str) = p_thigh(leg_str) + rot_x(leg_q(1))*rot_y(leg_q(2))*[0, 0, -thigh_length]';
    p_foot(leg_str) = p_calf(leg_str) + rot_x(leg_q(1))*rot_y(leg_q(2))*rot_y(leg_q(3))*[0, 0, -calf_length]';
    p_feet = [p_feet; p_foot(leg_str)];
end

J_feet = jacobian(p_feet, q_b);
J_feet = simplify(J_feet);

matlabFunction(p_feet, J_feet,'Outputs',{'p_feet', 'J_feet'},...
    'File', [file_folder, '/gen/kinBodyFeet'],...
    'Vars', {q_b});
save([file_folder, '/gen/kinBodyFeet'], 'legs');











