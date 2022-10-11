function trajectory = cubic_trajectory_generator(wayPoints,wayPointVels)


numTotalPoints = size(wayPoints,1)*10;
waypointTime = 4;

wpTimes = (0:size(wayPoints,1)-1)*waypointTime;
trajTimes = linspace(0,wpTimes(end),numTotalPoints);
trajectory = cubicpolytraj(wayPoints',wpTimes,trajTimes, ...
             'VelocityBoundaryCondition',wayPointVels');

% ik = robotics.InverseKinematics('RigidBodyTree',robot);
% weights = [0.1 0.1 0.1 1 1 1];
% initialguess = robot.homeConfiguration;
% 
% % Call inverse kinematics solver for every end-effector position using the
% % previous configuration as initial guess
% for idx = 1:size(trajectory,2)
%     tform = trvec2tform(trajectory(:,idx)');
%     configSoln(idx,:) = ik('panda_hand',tform,weights,initialguess);
%     initialguess = configSoln(idx,:);
% end