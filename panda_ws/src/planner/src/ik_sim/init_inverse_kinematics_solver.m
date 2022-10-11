function ik = init_inverse_kinematics_solver(robot)
% Initialize Inverse Kinematics object for the robot
ik = robotics.InverseKinematics('RigidBodyTree',robot);