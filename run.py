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
    Demultiplexer
)
from pydrake.systems.controllers import (
    InverseDynamics
)
from pydrake.manipulation import (
    SimIiwaDriver,
    IiwaControlMode
)
from manipulation.station import MakeHardwareStation, LoadScenario
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from joint_position_publisher import JointPositionPublisher
from trajectory_sources import ToggleHoldTorqueSource, DummyZeroTorqueCommander

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

    gravity_compensation: InverseDynamics = builder.AddNamedSystem(
        "gravity_compensation",
        InverseDynamics(
            plant=controller_plant,
            mode=InverseDynamics.InverseDynamicsMode.kGravityCompensation,
        ),
    )

    torque_source = builder.AddSystem(ToggleHoldTorqueSource(meshcat))
    if use_hardware:
        builder.Connect(
            external_station.GetOutputPort("iiwa.state_estimated"),
            gravity_compensation.get_input_port_estimated_state(),
        )

        builder.Connect(
            gravity_compensation.get_output_port_generalized_force(),
            torque_source.GetInputPort("current_cmd"),
        )
        builder.Connect(
            torque_source.GetOutputPort("torque_cmd"),
            external_station.GetInputPort("iiwa.torque"),
        )
        builder.Connect(
            external_station.GetOutputPort("iiwa.torque_measured"),
            station.GetInputPort("iiwa.torque"),
        )

    else:
        dummy = builder.AddSystem(DummyZeroTorqueCommander())
        builder.Connect(
            dummy.GetOutputPort("torque_cmd"),
            external_station.GetInputPort("iiwa.torque"),
        )

        # sim_driver = builder.AddSystem(SimIiwaDriver(IiwaControlMode.kTorqueOnly, controller_plant, 0.1, np.full(7, 100.0)))
        # builder.Connect(
        #     torque_source.GetOutputPort("torque_cmd"),
        #     sim_driver.GetInputPort("torque"),
        # )
        # builder.Connect(
        #     station.GetOutputPort("iiwa.state_estimated"),
        #     sim_driver.GetInputPort("state"),
        # )

        builder.Connect(
            external_station.GetOutputPort("iiwa.state_estimated"),
            gravity_compensation.get_input_port_estimated_state(),
        )

        builder.Connect(
            gravity_compensation.get_output_port_generalized_force(),
            torque_source.GetInputPort("current_cmd"),
        )
        builder.Connect(
            external_station.GetOutputPort("iiwa.position_commanded"),
            torque_source.GetInputPort("current_position"),
        )
        builder.Connect(
            torque_source.GetOutputPort("torque_commanded"),
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