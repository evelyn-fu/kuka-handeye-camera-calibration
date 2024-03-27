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
from joint_position_publisher import JointPositionPublisher
from trajectory_sources import ToggleHoldControlModeSource, ZeroTorqueCommander, StateFromPositionVelocity
from iiwa import IiwaHardwareStationDiagram

def main(use_hardware = False):
    builder = DiagramBuilder()
    dir_path = os.path.dirname(os.path.realpath(__file__))
    filename = os.path.join(dir_path, "scenario_data.yml")
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

    # pub = rospy.Publisher('iiwa_joint_positions', numpy_msg(Floats))
    # joint_publisher = builder.AddSystem(JointPositionPublisher(pub))

    # builder.Connect(
    #     external_station.GetOutputPort("iiwa.position_commanded"),
    #     joint_publisher.GetInputPort("iiwa_position"),
    # )
    controller_plant = station.get_iiwa_controller_plant()
    num_positions = 7
    kp_gains=np.full(num_positions, 600)
    damping_ratios=np.full(num_positions, 0.2)

    control_mode_source = builder.AddSystem(ToggleHoldControlModeSource(station.internal_meshcat))
    zero_torque_source = builder.AddSystem(ZeroTorqueCommander())
    inverse_dynamics_controller = builder.AddSystem(
        InverseDynamicsController(
            controller_plant,
            kp=kp_gains,
            ki=[1] * num_positions,
            kd=2 * damping_ratios * np.sqrt(kp_gains),
            has_reference_acceleration=False,
        ),
    )

    switch = builder.AddSystem(PortSwitch(7))
    builder.Connect(
        inverse_dynamics_controller.GetOutputPort("generalized_force"), 
        switch.DeclareInputPort("pd_torque"),
    )
    builder.Connect(
        zero_torque_source.GetOutputPort("torque_cmd"),
        switch.DeclareInputPort("zero_torque"),
    )
    builder.Connect(
        control_mode_source.GetOutputPort("control_mode"),
        switch.get_port_selector_input_port(),
    )

    builder.Connect(
        control_mode_source.GetOutputPort("hold_state"),
        inverse_dynamics_controller.GetInputPort("desired_state"),
    )
    builder.Connect(
        station.GetOutputPort("iiwa.position_measured"),
        control_mode_source.GetInputPort("current_position"),
    )
    builder.Connect(
        station.GetOutputPort("iiwa.state_estimated"),
        inverse_dynamics_controller.GetInputPort("estimated_state"),
    )

    builder.Connect(
        switch.get_output_port(),
        station.GetInputPort("iiwa.torque"),
    )

    # Required for visualizing the internal station
    _ = MeshcatVisualizer.AddToBuilder(
        builder, station.GetOutputPort("query_object"), station.internal_meshcat
    )

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)

    station.internal_meshcat.AddButton("Stop Simulation", "Escape")
    print("Press Escape to stop the simulation")
    while station.internal_meshcat.GetButtonClicks("Stop Simulation") < 1:
        simulator.AdvanceTo(simulator.get_context().get_time() + 0.1)
    station.internal_meshcat.DeleteButton("Stop Simulation")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--use_hardware",
        action="store_true",
        help="Whether to use real world hardware.",
    )
    args = parser.parse_args()
    main(use_hardware=args.use_hardware)