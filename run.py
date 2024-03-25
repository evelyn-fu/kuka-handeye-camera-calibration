import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import os
from pydrake.geometry import (
    StartMeshcat
)
from manipulation.station import MakeHardwareStation, LoadScenario
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from joint_position_publisher import JointPositionPublisher
from trajectory_sources import ToggleHoldTorqueSource


if __name__ == '__main__':
    # Start the visualizer.
    meshcat = StartMeshcat()

    builder = DiagramBuilder()
    dir_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    filename = os.path.join(dir_path, "scenario_data.yml")
    scenario = LoadScenario(filename=filename)
    station = builder.AddSystem(MakeHardwareStation(scenario, meshcat, hardware=False))
    external_station = builder.AddSystem(MakeHardwareStation(scenario, meshcat, hardware=True))

    pub = rospy.Publisher('iiwa_joint_positions', numpy_msg(Floats))
    joint_publisher = builder.AddSystem(JointPositionPublisher(pub))

    builder.Connect(
        external_station.GetOutputPort("iiwa.position_commanded"),
        joint_publisher.GetInputPort("iiwa_position"),
    )

    torque_source = builder.AddSystem(ToggleHoldTorqueSource(meshcat))
    builder.Connect(
        external_station.GetOutputPort("iiwa.torque_commanded"),
        torque_source.GetInputPort("current_cmd"),
    )
    builder.Connect(
        torque_source.GetOutputPort("torque_cmd"),
        external_station.GetInputPort("iiwa.feedforward_torque"),
    )
    
    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)

    meshcat.AddButton("Stop Simulation", "Escape")
    print("Press Escape to stop the simulation")
    while meshcat.GetButtonClicks("Stop Simulation") < 1:
        simulator.AdvanceTo(simulator.get_context().get_time() + 0.1)
    meshcat.DeleteButton("Stop Simulation")