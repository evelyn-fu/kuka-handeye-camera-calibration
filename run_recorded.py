# import rospy
# from rospy.numpy_msg import numpy_msg
# from rospy_tutorials.msg import Floats
import os
import argparse
import numpy as np
import time
from pydrake.geometry import (
    StartMeshcat
)
from pydrake.systems.primitives import (
    PortSwitch
)
from pydrake.systems.controllers import (
    InverseDynamicsController
)
from pydrake.manipulation import (
    SimIiwaDriver,
    IiwaControlMode
)
from pydrake.all import (
    MeshcatVisualizer,
    RigidTransform,
    RollPitchYaw,
    RotationMatrix
)
from manipulation.station import MakeHardwareStation, LoadScenario
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from end_effector_transform_publisher import EndEffectorTransformPublisher, TakeSampleTriggerPublisher
from trajectory_sources import ToggleHoldControlModeSource, ZeroTorqueCommander, StateFromPositionVelocity
from iiwa import IiwaHardwareStationDiagram, IiwaForwardKinematics
import rclpy
from manipulation.scenarios import AddIiwaDifferentialIK
from pydrake.all import DiagramBuilder, MeshcatPoseSliders, Simulator

def main(use_hardware = False, camera = "back_right"):
    if camera not in ["back_right", "back_left", "front"]:
        raise Exception("Invalid camera selection. Valid options are \'back_right\', \'back_left\', or \'front\'")
    
    builder = DiagramBuilder()
    dir_path = os.path.dirname(os.path.realpath(__file__))
    filename = os.path.join(dir_path, "scenario_data_teleop.yml")
    scenario = LoadScenario(filename=filename)
    station: IiwaHardwareStationDiagram = builder.AddNamedSystem(
        "iiwa_hardware_station",
        IiwaHardwareStationDiagram(
            scenario=scenario,
            has_wsg=False,
            use_hardware=use_hardware,
            control_mode=scenario.model_drivers["iiwa"].control_mode,
            package_xmls=[],
        ),
    )

    if not use_hardware:
        station.disable_gravity()

    rclpy.init()
    ee_publisher = builder.AddSystem(EndEffectorTransformPublisher(station.get_internal_plant()))
    take_sample_publisher = TakeSampleTriggerPublisher()

    builder.Connect(
        station.GetOutputPort("body_poses"),
        ee_publisher.GetInputPort("body_poses"),
    )
    controller_plant = station.get_iiwa_controller_plant()
    differential_ik = AddIiwaDifferentialIK(
        builder,
        controller_plant,
        frame=controller_plant.GetFrameByName("iiwa_link_7"),
    )
    builder.Connect(
        differential_ik.get_output_port(),
        station.GetInputPort("iiwa.position"),
    )
    builder.Connect(
        station.GetOutputPort("iiwa.state_estimated"),
        differential_ik.GetInputPort("robot_state"),
    )

    # Set up teleop widgets
    teleop = builder.AddSystem(
        MeshcatPoseSliders(
            station.internal_meshcat,
            lower_limit=[0, -0.5, -np.pi, 0.32, -0.3, 0.35],
            upper_limit=[2 * np.pi, np.pi, np.pi, 0.7, 0.3, 0.7],
        )
    )
    builder.Connect(
        teleop.get_output_port(), differential_ik.GetInputPort("X_AE_desired")
    )
    iiwa_forward_kinematics = builder.AddSystem(
        IiwaForwardKinematics(station.get_internal_plant())
    )
    builder.Connect(
        station.GetOutputPort("iiwa.position_commanded"),
        iiwa_forward_kinematics.get_input_port(),
    )
    builder.Connect(iiwa_forward_kinematics.get_output_port(), teleop.get_input_port())
    
    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)

    samples_back_right = [] # roll, pitch, yaw, x, y, z
    for z in [0.35, 0.45]:
        for y in [-0.3, 0.0]:
            for x in [0.5, 0.6]:
                for roll in [3.14, 2.36]:
                    for pitch in [0, 0.4, -0.4]:
                        for yaw in [1, 0.7, 1.3]:
                            samples_back_right.append([roll, pitch, yaw, x, y, z])
    samples_back_right.append(samples_back_right[0])

    samples_back_left = [] # roll, pitch, yaw, x, y, z
    for z in [0.35, 0.45]:
        for y in [0.3, 0.0]:
            for x in [0.5, 0.6]:
                for roll in [3.14, 3.92]:
                    for pitch in [0, 0.4, -0.4]:
                        for yaw in [-1, -0.7, -1.3]:
                            samples_back_left.append([roll, pitch, yaw, x, y, z])
    samples_back_left.append(samples_back_left[0])

    samples_front = [] # roll, pitch, yaw, x, y, z
    for z in [0.55, 0.45]:
        for y in [-0.1, 0.0, 0.1]:
            for x in [0.5, 0.6]:
                for roll in [2.36, 3.14, 3.92]:
                    for pitch in [0, 0.4]:
                        for yaw in [-3, 3]:
                            samples_front.append([roll, pitch, yaw, x, y, z])
    samples_front.append(samples_front[0])

    samples = []
    if camera == "back_right":
        samples = samples_back_right
    if camera == "back_left":
        samples = samples_back_left
    if camera == "front":
        samples = samples_front

    start = time.time()
    moving = False
    sample_ind = 0
    done = True
    station.internal_meshcat.AddButton("Stop Simulation")
    station.internal_meshcat.AddButton("Start Calibration")
    start_calibration_clicks = station.internal_meshcat.GetButtonClicks("Start Calibration")
    while station.internal_meshcat.GetButtonClicks("Stop Simulation") < 1:
        if not done:
            if (moving and time.time() - start > 2.0) or (not moving and time.time() - start > 1.0):
                if moving:
                    take_sample_publisher.publish()
                    moving = False
                else:
                    s = samples[sample_ind]
                    print(s)
                    teleop.SetPose(RigidTransform(RotationMatrix(RollPitchYaw(s[0], s[1], s[2])), s[3:]))
                    sample_ind += 1
                    if sample_ind == len(samples):
                        done = True # Don't publish last one (return to home)
                    moving = True
                start = time.time()
        else:
            if station.internal_meshcat.GetButtonClicks("Start Calibration") > start_calibration_clicks:
                done = False
        
        start_calibration_clicks = station.internal_meshcat.GetButtonClicks("Start Calibration")
        simulator.AdvanceTo(simulator.get_context().get_time() + 0.1)
        

    station.internal_meshcat.DeleteButton("Stop Simulation")
    ee_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--use_hardware",
        action="store_true",
        help="Whether to use real world hardware.",
    )
    parser.add_argument(
        "--camera",
        default="back_right",
        help="which camera to calibrate",
    )
    args = parser.parse_args()
    main(use_hardware=args.use_hardware, camera=args.camera)