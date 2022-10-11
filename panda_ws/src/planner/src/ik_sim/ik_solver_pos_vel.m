% function [pos,vel] = ik_solver_pos_vel(initial_guess,trajectory)
function [pos,vel] = ik_solver_pos_vel(initial_guess,end_pose)

% robot = importrobot('frankaEmikaPanda.urdf');
% ik = robotics.InverseKinematics('RigidBodyTree',robot);
% weights = [0.1 0.1 0.1 1 1 1];
% % initialguess = robot.homeConfiguration;
% % configSoln = zeros()
% % Call inverse kinematics solver for every end-effector position using the
% % previous configuration as initial guess
% for idx = 1:size(trajectory,2)
%     tform = trvec2tform(trajectory(:,idx)');
%     configSoln(idx,:) = ik('panda_hand',tform,weights,initialguess);
%     initialguess = configSoln(idx,:);
% end



robot = importrobot('frankaEmikaPanda.urdf');
ik = robotics.InverseKinematics('RigidBodyTree',robot);
weights = [0.1 0.1 0.1 1 1 1];
% initialguess = robot.homeConfiguration;
pos_and_vel_ref = repmat( struct('JointName',0,'JointPosition',0),...
    size(end_pose,2), size(initial_guess,2));

% Call inverse kinematics solver for every end-effector position using the
% previous configuration as initial guess
idx = 2;
% for idx = 2:size(trajectory,2)
tform = trvec2tform(end_pose(:,idx)');
pos_and_vel_ref(idx,:) = ik('panda_hand',tform,weights,initial_guess);
%     initial_guess = pos_and_vel_ref(idx,:);
% end