import time

from RobotInterface import ROBOT_IP, RobotInterface
from GripperInterface import GripperInterface
from GripperInterface import *

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
from rtde import serialize
import sys
import os
import logging

import csv

base_folder = os.path.dirname(__file__)

j1 = np.array([-18, -100, 110, -10, 70, 0])/180*np.pi
j2 = np.array([-5.0, -52.9, 112.0, -60.1, 112.6, 0.29])/180*np.pi


y_init = 0
z_init = 0.07
Y_STEP = 0.005

if(__name__=="__main__"):
    robot = RobotInterface()
    robot.connect()
    if(len(sys.argv) == 1):
        robot.movej(j1)
        robot.movel(np.array([-0.71, y_init, z_init+0.10]),[3.372, 1.932, -2.122])
        robot.movel(np.array([-0.71, y_init, z_init]),[3.372, 1.932, -2.122])
        robot.shutdown()
        gripper.shutdown()
        sys.exit()
    elif(len(sys.argv) != 2):
        print("Provide object name")
        robot.shutdown()
        sys.exit()

    obj_name = sys.argv[1]

    gripper:GripperInterface = GripperInterface("/dev/ttyUSB0")
    gripper.verbose=False

    gripper.set_selected_signals(np.array([2,3]))
    gripper.execute("opt stream", blocking=False)

    conf = rtde_config.ConfigFile(base_folder+"/record_configuration.xml")
    output_names, output_types = conf.get_recipe('out')

    #robot.move_home()
    with open(base_folder+f"/tmp/opt_test/{obj_name}.csv", 'w') as f:
        writer = csv.writer(f)
        writer.writerow(["k","robot y","robot z","opt x","opt y"])

        for k in range(8):
            y = y_init - k*Y_STEP
            robot.connect()
            robot.movel(np.array([-0.71, y, z_init]),[3.372, 1.932, -2.122])
            # time.sleep(1)

            con = rtde.RTDE(ROBOT_IP, 30004)
            con.connect()
            con.get_controller_version()
            
            # setup recipes
            if not con.send_output_setup(output_names, output_types, frequency=50):
                logging.error('Unable to configure output')
                sys.exit()

            #start data synchronization
            if not con.send_start():
                logging.error('Unable to start synchronization')
                sys.exit()

            gripper.execute("reset slide", blocking=False)
            gripper.execute("opt stream", blocking=False)

            keep_running = True
            while keep_running:
                try:
                    state = con.receive()
                    if state is not None:
                        data = []
                        for i in range(len(output_names)):
                            size = serialize.get_item_size(output_types[i])
                            value = state.__dict__[output_names[i]]
                            if size > 1:
                                data.extend(value)
                            else:
                                data.append(value)

                            pos = state.actual_TCP_pose[:3]
                            if(pos[2] > z_init+0.005):
                                opt = gripper.current_signals()
                                y = pos[1]-y_init
                                z = pos[2]-z_init
                                print(f"{k}, {y:.3}, {z:.3}, {opt[0]}, {opt[1]}")
                                writer.writerow(np.array([k, y, z, opt[0], opt[1]]))
                            # print(", ".join( [f"{x:.4f}" for x in state.actual_TCP_pose] ), end=" : ")
                            # print( gripper.current_signals() )

                except KeyboardInterrupt:
                    keep_running = False
                except rtde.RTDEException:
                    con.disconnect()
                    sys.exit()

            gripper.execute("stop",blocking=False)

    time.sleep(1)
    robot.shutdown()
    gripper.shutdown()