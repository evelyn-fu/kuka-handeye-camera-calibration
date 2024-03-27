import numpy as np
from pydrake.all import BasicVector, Context, LeafSystem
from enum import Enum
from pydrake.common.value import (
    AbstractValue
)
from pydrake.geometry import Meshcat

class ControlState(Enum):
    HOLD = 1
    FREE = 2

num_iiwa_positions = 7

class ToggleHoldPositionSource(LeafSystem):
    def __init__(self, meshcat: Meshcat):
        super().__init__()
        self._mode_index = self.DeclareAbstractState(
            AbstractValue.Make(ControlState.HOLD)
        )

        # The current position cmd to be used if holding
        self._current_cmd_input_port = self.DeclareVectorInputPort(
            "position_commanded", num_iiwa_positions
        )

        self._current_position_input_port = self.DeclareVectorInputPort(
            "position_measured", num_iiwa_positions
        )

        self.DeclarePeriodicUnrestrictedUpdateEvent(0.1, 0.0, self.Update)

        # read button from meshcat to toggle hold/free
        self._meshcat = meshcat
        self._button = "Toggle hold/free"
        meshcat.AddButton(self._button, "Space")

        self.DeclareVectorOutputPort(
            "position_cmd", num_iiwa_positions, self._calc_torque_value
        )

    def __del__(self):
        self._meshcat.DeleteButton(self._button)

    def Update(self, context, state):
        if (self._meshcat.GetButtonClicks(self._button) % 2) == 1:
            state.get_mutable_abstract_state(
                int(self._mode_index)
            ).set_value(ControlState.FREE)
        else:
            state.get_mutable_abstract_state(
                int(self._mode_index)
            ).set_value(ControlState.HOLD)

    def _calc_torque_value(self, context: Context, output: BasicVector) -> None:
        mode = context.get_abstract_state(int(self._mode_index)).get_value()

        if mode == ControlState.HOLD:
            output.SetFromVector(self._current_cmd_input_port.Eval(context))
        else:
            output.SetFromVector(self._current_position_input_port.Eval(context))

class ToggleHoldTorqueSource(LeafSystem):

    def __init__(self, meshcat: Meshcat):
        super().__init__()
        self._mode_index = self.DeclareAbstractState(
            AbstractValue.Make(ControlState.HOLD)
        )

        # The current torque cmd to be used if holding
        self._current_cmd_input_port = self.DeclareVectorInputPort(
            "current_cmd", num_iiwa_positions
        )

        self._current_position_input_port = self.DeclareVectorInputPort(
            "current_position", num_iiwa_positions
        )

        self.DeclarePeriodicUnrestrictedUpdateEvent(0.1, 0.0, self.Update)

        # read button from meshcat to toggle hold/free
        self._meshcat = meshcat
        self._button = "Toggle hold/free"
        meshcat.AddButton(self._button, "Space")

        self.DeclareVectorOutputPort(
            "torque_cmd", num_iiwa_positions, self._calc_torque_value
        )

    def __del__(self):
        self._meshcat.DeleteButton(self._button)

    def Update(self, context, state):
        if (self._meshcat.GetButtonClicks(self._button) % 2) == 1:
            state.get_mutable_abstract_state(
                int(self._mode_index)
            ).set_value(ControlState.FREE)
        else:
            state.get_mutable_abstract_state(
                int(self._mode_index)
            ).set_value(ControlState.HOLD)

    def _calc_torque_value(self, context: Context, output: BasicVector) -> None:
        mode = context.get_abstract_state(int(self._mode_index)).get_value()

        # print(self._current_position_input_port.Eval(context))
        if mode == ControlState.HOLD:
            print(self._current_cmd_input_port.Eval(context))
            output.SetFromVector(np.zeros(num_iiwa_positions)) # this for now cuz sheesh
            # output.SetFromVector(self._current_cmd_input_port.Eval(context))
        else:
            output.SetFromVector(np.zeros(num_iiwa_positions))

class DummyZeroTorqueCommander(LeafSystem):
    def __init__(self):
        super().__init__()

        self.DeclareVectorOutputPort(
            "torque_cmd", num_iiwa_positions, self._calc_torque_value
        )

    def _calc_torque_value(self, context: Context, output: BasicVector) -> None:
        output.SetFromVector(np.zeros(num_iiwa_positions))