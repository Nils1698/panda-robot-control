%% Planning and control loop simulation

% Determine initial position

% Generate Waypoints
wayPoints       = [0.4 -0.2 0.3; 0.3 0.2 0.4];
wayPointVels    = [0   0    0  ; 0   0   0  ];

% Generate Trajectory
trajectory = cubic_trajectory_generator(wayPoints,wayPointVels);


% Low level control loop
%   Take in trajectory reference
%   compute inverse kinematics based on current position



% robot = importrobot('frankaEmikaPanda.urdf');

%%
robot = importrobot('frankaEmikaPanda.urdf');
ik = robotics.InverseKinematics('RigidBodyTree',robot);
weights = [0.1 0.1 0.1 1 1 1];
initialguess = robot.homeConfiguration;
pos_and_vel_ref = repmat( struct('JointName',0,'JointPosition',0),...
    size(trajectory,2), size(initialguess,2));

% Call inverse kinematics solver for every end-effector position using the
% previous configuration as initial guess
for idx = 2:size(trajectory,2)
    tform = trvec2tform(trajectory(:,idx)');
    pos_and_vel_ref(idx,:) = ik('panda_hand',tform,weights,initialguess);
    initialguess = pos_and_vel_ref(idx,:);
end

%% Visualize robot configurations
figure
title('Robot waypoint tracking visualization')
robot = importrobot('frankaEmikaPanda.urdf');
for idx = 1:size(trajectory,2)
    axis([-1 1 -1 1 0 1]);
    plot3(wayPoints(:,1)',wayPoints(:,2)',wayPoints(:,3)','LineStyle','none','Marker','.','MarkerSize',20)
    hold on
    plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:),'r-','LineWidth',2);
    show(robot,pos_and_vel_ref(idx,:), 'PreservePlot', false,'Frames','on');
    pause(0.2)
    hold off
end

%%
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




% 
% 
% ik = robotics.InverseKinematics('RigidBodyTree',robot);
% weights = [0.1 0.1 0.1 1 1 1];
% initialguess = robot.homeConfiguration;
% configSoln = zeros()
% % Call inverse kinematics solver for every end-effector position using the
% % previous configuration as initial guess
% for idx = 1:size(trajectory,2)
%     tform = trvec2tform(trajectory(:,idx)');
%     configSoln(idx,:) = ik('panda_hand',tform,weights,initialguess);
%     initialguess = configSoln(idx,:);
% end