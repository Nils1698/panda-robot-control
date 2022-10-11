% test of trajectory_solver
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

robot = importrobot('frankaEmikaPanda.urdf');

initialguess = robot.homeConfiguration;
pos_and_vel_ref = trajectory_solver(initialguess, wayPoints, wayPointVels);

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
