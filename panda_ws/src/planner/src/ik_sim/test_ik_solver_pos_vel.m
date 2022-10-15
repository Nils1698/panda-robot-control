% test_ik_solver_pos_vel


% Generate Waypoints
wayPoints       = [0.4 -0.2 0.3; 0.3 0.2 0.4];
wayPointVels    = [0   0    0  ; 0   0   0  ];

% Generate Trajectory
trajectory = cubic_trajectory_generator(wayPoints,wayPointVels);



initial_guess = robot.homeConfiguration.JointPosition


[pos,vel] = ik_solver_pos_vel(initial_guess,end_pose)