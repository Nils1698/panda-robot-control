% function configSoln = ik_solver_ckk(initial_guess,trajectory,robot,ik)
% function configSoln = ik_solver_ckk(initial_guess,end_pose,robot,ik)
function configSoln = ik_solver_pos(initial_guess,end_pose)
%%%
% initial_guess:    start joint configuration of robot arm
% end pose:         desired final endeffector position of robot arm

% initial_guess:    The initial pose of the end-effector of the robot arm.
%   Dimensions:     
% end_pose:         The desired final pose of the robot arm.
% robot:            The robot object
% ik:               The inverse kinematics solver object


robot = importrobot('frankaEmikaPanda.urdf');
ik = robotics.InverseKinematics('RigidBodyTree',robot);

weights = [0.1 0.1 0.1 1 1 1];

initialguess = robot.homeConfiguration;

tform = trvec2tform(end_pose');
configSoln = ik('panda_hand',tform,weights,initialguess);
% start_pose = configSoln(idx,:);




% start_pose = repmat( struct('JointName',0,'JointPosition',0),...
%     size(end_pose,2), size(initial_guess,2));
% configSoln = zeros()
% Call inverse kinematics solver for every end-effector position using the
% previous configuration as initial guess
% % for idx = 1:size(trajectory,2)
%     tform = trvec2tform(trajectory(:,idx)');
%     configSoln(idx,:) = ik('panda_hand',tform,weights,start_pose);
%     start_pose = configSoln(idx,:);
% end


% for idx = 1:size(trajectory,2)
% tform = trvec2tform(trajectory');
% configSoln = ik('panda_hand',trvec2tform(trajectory'),[0.1 0.1 0.1 1 1 1],initialguess);
% initialguess = configSoln(idx,:);
% end
