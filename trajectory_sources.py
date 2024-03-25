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

        if mode == ControlState.HOLD:
            output.SetFromVector(self._current_cmd_input_port.Eval(context))
        else:
            output.SetFromVector(np.zeros(num_iiwa_positions))