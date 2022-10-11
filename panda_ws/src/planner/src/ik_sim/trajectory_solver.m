function pos_and_vel_ref = trajectory_solver(initialguess,wayPoints,wayPointVels)

numTotalPoints = size(wayPoints,1)*10;
waypointTime = 4;

% s = struct('JointName',{},'JointPosition',{})

wpTimes = (0:size(wayPoints,1)-1)*waypointTime;
trajTimes = linspace(0,wpTimes(end),numTotalPoints);
trajectory = cubicpolytraj(wayPoints',wpTimes,trajTimes, ...
             'VelocityBoundaryCondition',wayPointVels');

robot = importrobot('frankaEmikaPanda.urdf');
ik = robotics.InverseKinematics('RigidBodyTree',robot);
weights = [0.1 0.1 0.1 1 1 1];
% initialguess = robot.homeConfiguration;
pos_and_vel_ref = repmat( struct('JointName',0,'JointPosition',0), size(trajectory,2), size(initialguess,2));

% Call inverse kinematics solver for every end-effector position using the
% previous configuration as initial guess
for idx = 2:size(trajectory,2)
    tform = trvec2tform(trajectory(:,idx)');
    pos_and_vel_ref(idx,:) = ik('panda_hand',tform,weights,initialguess);
    initialguess = pos_and_vel_ref(idx,:);
end