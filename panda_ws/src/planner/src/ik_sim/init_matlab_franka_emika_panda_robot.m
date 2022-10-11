function robot = init_matlab_franka_emika_panda_robot()
% Initializes the robot object of the Franka Emika Panda robot arm
robot = importrobot('frankaEmikaPanda.urdf');