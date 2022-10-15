%% Waypoint tracking demonstration using Robotics System Toolbox

% This demonstration performs inverse kinematics of a
% robot manipulator to follow a desired set of waypoints.

% Copyright 2017-2019 The MathWorks, Inc.
%% Load and display robot
clear
clc

robot = importrobot('frankaEmikaPanda.urdf');

axes = show(robot);
axes.CameraPositionMode = 'auto';
%% Create a set of desired waypoints
clc
waypointType = 'very_simple'; % 'complex'; %'simple'; % or 
switch waypointType
    case 'very_simple'
        wayPoints = [0.4 -0.2 0.52; 0.3 0.2 0.52]; % Alternate set of wayPoints
        wayPointVels = [0 0 0;0 0 0];
    case 'simple_franka'
        wayPoints = [0.4 -0.2 0.52; 0.4 0.2 0.52; 0.3 0.2 0.52; 0.3 -0.2 0.52; 0.5 -0.2 0.52]; % Alternate set of wayPoints
        wayPointVels = [0.1 0 0; -0.1 0 0; -0.1 0 0; 0.1 -0.1 0; 0 0 0];
    case 'traj3'
        wayPoints = [0.4 -0.2 0.3; 0.4 0.2 0.35; 0.3 0.2 0.4; 0.3 -0.2 0.45; 0.5 -0.2 0.5]; % Alternate set of wayPoints
        wayPointVels = [0 0 0; 0 0 0; 0 0 0; 0 0 0; 0 0 0];
    case 'traj4'
        wayPoints = [0.4 -0.2 0.3; 0.4 0.2 0.35; 0.3 0.2 0.4; 0.3 -0.2 0.45; 0.5 -0.2 0.5]; % Alternate set of wayPoints
        wayPointVels = [0.1 0 0.1; -0.1 0 0.1; -0.1 0 0.1; 0.1 -0.1 0.1; 0 0 0.1];
    case 'simple'
        wayPoints = [0.2 -0.2 0.02;0.25 0 0.15; 0.2 0.2 0.02]; % Alternate set of wayPoints
        wayPointVels = [0 0 0;0 0.1 0;0 0 0];
    case 'complex'
        wayPoints = [0.2 -0.2 0.02;0.15 0 0.28;0.15 0.05 0.2; 0.15 0.09 0.15;0.1 0.12 0.1; 0.04 0.1 0.2;0.25 0 0.15; 0.2 0.2 0.02];
        wayPointVels = [0 0 0; 0 0.05 0; 0 0.025 -0.025; 0 0.025 -0.025;-0.1 0 0;0.05 0 0;0 -0.1 0;0 0 0];
end
% exampleHelperPlotWaypoints(wayPoints);
%% Create a smooth trajectory from the waypoints
% hold off
% figure
numTotalPoints = size(wayPoints,1)*10;
waypointTime = 4;
trajType = 'cubic'; % or 'trapezoidal'
switch trajType
    case 'trapezoidal' % Position and velocity reference
        trajectory = trapveltraj(wayPoints',numTotalPoints,'EndTime',waypointTime);
    case 'cubic' % Position, velocity and acceleration reference
        wpTimes = (0:size(wayPoints,1)-1)*waypointTime;
        trajTimes = linspace(0,wpTimes(end),numTotalPoints);
        [trajectory,traj_vel] = cubicpolytraj(wayPoints',wpTimes,trajTimes, ...
                     'VelocityBoundaryCondition',wayPointVels');
end
% Plot trajectory spline and waypoints

hold on
plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:),'r-','LineWidth',2);
% %%
% figure
% 
% plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:),'r-','LineWidth',2);
% hold on
plot3(wayPoints(:,1)',wayPoints(:,2)',wayPoints(:,3)','LineStyle','none','Marker','.','MarkerSize',20)
%% Perform Inverse Kinematics
% Use desired weights for solution (First three are orientation, last three are translation)
% Since it is a 4-DOF robot with only one revolute joint in Z we do not
% put a weight on Z rotation; otherwise it limits the solution space

ik = robotics.InverseKinematics('RigidBodyTree',robot);
weights = [0.1 0.1 0.1 1 1 1];
initialguess = robot.homeConfiguration;

% Call inverse kinematics solver for every end-effector position using the
% previous configuration as initial guess
for idx = 1:size(trajectory,2)
    tform = trvec2tform(trajectory(:,idx)');
    configSoln(idx,:) = ik('panda_hand',tform,weights,initialguess);
    initialguess = configSoln(idx,:);
end
%% Visualize robot configurations
figure
title('Robot waypoint tracking visualization')

for idx = 1:size(trajectory,2)
    axis([-1 1 -1 1 0 1]);
    plot3(wayPoints(:,1)',wayPoints(:,2)',wayPoints(:,3)','LineStyle','none','Marker','.','MarkerSize',20)
    hold on
    plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:),'r-','LineWidth',2);
    show(robot,configSoln(idx,:), 'PreservePlot', false,'Frames','on');
    pause(0.2)
    hold off
end

%% Waypoint tracking demonstration using Robotics System Toolbox
% 
% % This demonstration performs inverse kinematics of a
% % robot manipulator to follow a desired set of waypoints.
% 
% % Copyright 2017-2019 The MathWorks, Inc.
% 
% %% Load and display robot
% clear
% clc
% 
% addpath(genpath(strcat(pwd,'\Dependencies')))
% robot = createRigidBodyTree;
% axes = show(robot);
% axes.CameraPositionMode = 'auto';
% 
% %% Create a set of desired waypoints
% waypointType = 'simple'; % or 'complex'
% switch waypointType
%     case 'simple'
%         wayPoints = [0.2 -0.2 0.02;0.25 0 0.15; 0.2 0.2 0.02]; % Alternate set of wayPoints
%         wayPointVels = [0 0 0;0 0.1 0;0 0 0];
%     case 'complex'
%         wayPoints = [0.2 -0.2 0.02;0.15 0 0.28;0.15 0.05 0.2; 0.15 0.09 0.15;0.1 0.12 0.1; 0.04 0.1 0.2;0.25 0 0.15; 0.2 0.2 0.02];
%         wayPointVels = [0 0 0; 0 0.05 0; 0 0.025 -0.025; 0 0.025 -0.025;-0.1 0 0;0.05 0 0;0 -0.1 0;0 0 0];
% end
% exampleHelperPlotWaypoints(wayPoints);
% 
% %% Create a smooth trajectory from the waypoints
% numTotalPoints = size(wayPoints,1)*10;
% waypointTime = 4;
% trajType = 'cubic'; % or 'trapezoidal'
% switch trajType
%     case 'trapezoidal'
%         trajectory = trapveltraj(wayPoints',numTotalPoints,'EndTime',waypointTime);
%     case 'cubic'
%         wpTimes = (0:size(wayPoints,1)-1)*waypointTime;
%         trajTimes = linspace(0,wpTimes(end),numTotalPoints);
%         trajectory = cubicpolytraj(wayPoints',wpTimes,trajTimes, ...
%                      'VelocityBoundaryCondition',wayPointVels');
% end
% % Plot trajectory spline and waypoints
% 
% hold on
% plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:),'r-','LineWidth',2);
% 
% %% Perform Inverse Kinematics
% % Use desired weights for solution (First three are orientation, last three are translation)
% % Since it is a 4-DOF robot with only one revolute joint in Z we do not
% % put a weight on Z rotation; otherwise it limits the solution space
% 
% ik = robotics.InverseKinematics('RigidBodyTree',robot);
% weights = [0.1 0.1 0 1 1 1];
% initialguess = robot.homeConfiguration;
% 
% % Call inverse kinematics solver for every end-effector position using the
% % previous configuration as initial guess
% for idx = 1:size(trajectory,2)
%     tform = trvec2tform(trajectory(:,idx)');
%     configSoln(idx,:) = ik('end_effector',tform,weights,initialguess);
%     initialguess = configSoln(idx,:);
% end
% 
% %% Visualize robot configurations
title('Robot waypoint tracking visualization')
axis([-0.1 0.4 -0.35 0.35 0 0.35]);
for idx = 1:size(trajectory,2)
    show(robot,configSoln(idx,:), 'PreservePlot', false,'Frames','off');
    pause(0.1)
end
hold off