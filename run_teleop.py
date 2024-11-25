# import rospy
# from rospy.numpy_msg import numpy_msg
# from rospy_tutorials.msg import Floats
import os
import argparse
import numpy as np
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
)
from manipulation.station import MakeHardwareStation, LoadScenario
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from end_effector_transform_publisher import EndEffectorTransformPublisher
from trajectory_sources import ToggleHoldControlModeSource, ZeroTorqueCommander, StateFromPositionVelocity
from iiwa import IiwaHardwareStationDiagram, IiwaForwardKinematics
import rclpy
from manipulation.scenarios import AddIiwaDifferentialIK
from pydrake.all import DiagramBuilder, MeshcatPoseSliders, Simulator

def main(use_hardware = False):
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

    station.internal_meshcat.AddButton("Stop Simulation")
    while station.internal_meshcat.GetButtonClicks("Stop Simulation") < 1:
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
    args = parser.parse_args()
    main(use_hardware=args.use_hardware)