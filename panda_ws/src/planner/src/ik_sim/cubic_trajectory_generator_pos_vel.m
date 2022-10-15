function [traj_pos,traj_vel] = cubic_trajectory_generator_pos_vel(wayPoints,wayPointVels)

numTotalPoints = size(wayPoints,1)*10;
waypointTime = 4;

wpTimes = (0:size(wayPoints,1)-1)*waypointTime;
trajTimes = linspace(0,wpTimes(end),numTotalPoints);
[traj_pos,traj_vel,~,~] = cubicpolytraj(wayPoints',wpTimes,trajTimes, ...
    'VelocityBoundaryCondition',wayPointVels');
