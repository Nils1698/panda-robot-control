% test_ik_solver_pos_vel


% Generate Waypoints
wayPoints       = [0.4 -0.2 0.3; 0.3 0.2 0.4];
wayPointVels    = [0   0    0  ; 0   0   0  ];

% Generate Trajectory
trajectory = cubic_trajectory_generator(wayPoints,wayPointVels);


robot = importrobot('frankaEmikaPanda.urdf');


start_pose = robot.homeConfiguration.JointPosition

end_pose = trajectory(:,end)
joint_configuration = ik_solver_pos(start_pose,end_pose)