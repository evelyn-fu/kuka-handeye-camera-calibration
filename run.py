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
from manipulation.station import MakeHardwareStation, LoadScenario
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from joint_position_publisher import JointPositionPublisher
from trajectory_sources import ToggleHoldControlModeSource, ZeroTorqueCommander, StateFromPositionVelocity

def main(use_hardware = False):
    # Start the visualizer.
    meshcat = StartMeshcat()

    builder = DiagramBuilder()
    dir_path = os.path.dirname(os.path.realpath(__file__))
    filename = os.path.join(dir_path, "scenario_data.yml")
    scenario = LoadScenario(filename=filename)
    station = builder.AddSystem(MakeHardwareStation(scenario, meshcat, hardware=False))
    external_station = builder.AddSystem(MakeHardwareStation(scenario, meshcat, hardware=True))

    # pub = rospy.Publisher('iiwa_joint_positions', numpy_msg(Floats))
    # joint_publisher = builder.AddSystem(JointPositionPublisher(pub))

    # builder.Connect(
    #     external_station.GetOutputPort("iiwa.position_commanded"),
    #     joint_publisher.GetInputPort("iiwa_position"),
    # )
    controller_plant = station.GetSubsystemByName(
        "plant"
    )
    num_positions = 7
    kp_gains=np.full(num_positions, 1)
    kd_gains=np.full(num_positions, 0)
    damping_ratios=np.full(num_positions, 0.2)

    state_from_position_velocity = builder.AddSystem(StateFromPositionVelocity())
    builder.Connect(
        external_station.GetOutputPort("iiwa.position_measured"),
        state_from_position_velocity.GetInputPort("position"),
    )
    builder.Connect(
        external_station.GetOutputPort("iiwa.velocity_estimated"),
        state_from_position_velocity.GetInputPort("velocity"),
    )

    control_mode_source = builder.AddSystem(ToggleHoldControlModeSource(meshcat))
    zero_torque_source = builder.AddSystem(ZeroTorqueCommander())
    inverse_dynamics_controller = builder.AddSystem(
        InverseDynamicsController(
            controller_plant,
            kp=kp_gains,
            ki=[0.0] * num_positions,
            kd=kd_gains,
            has_reference_acceleration=True,
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
        zero_torque_source.GetOutputPort("torque_cmd"), # vector of all zeros, does the same job of 0 accel
        inverse_dynamics_controller.GetInputPort("desired_acceleration"),
    )
    builder.Connect(
        external_station.GetOutputPort("iiwa.position_measured"),
        control_mode_source.GetInputPort("current_position"),
    )
    builder.Connect(
        state_from_position_velocity.GetOutputPort("state"),
        inverse_dynamics_controller.GetInputPort("estimated_state"),
    )

    if use_hardware:
        builder.Connect(
            switch.get_output_port(),
            external_station.GetInputPort("iiwa.torque"),
        )
        builder.Connect(
            external_station.GetOutputPort("iiwa.torque_measured"),
            station.GetInputPort("iiwa.torque"),
        )

    else:
        builder.Connect(
            zero_torque_source.GetOutputPort("torque_cmd"),
            external_station.GetInputPort("iiwa.torque"),
        )
        builder.Connect(
            switch.get_output_port(),
            station.GetInputPort("iiwa.torque"),
        )


    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)

    meshcat.AddButton("Stop Simulation", "Escape")
    print("Press Escape to stop the simulation")
    while meshcat.GetButtonClicks("Stop Simulation") < 1:
        simulator.AdvanceTo(simulator.get_context().get_time() + 0.1)
    meshcat.DeleteButton("Stop Simulation")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--use_hardware",
        action="store_true",
        help="Whether to use real world hardware.",
    )
    args = parser.parse_args()
    main(use_hardware=args.use_hardware)