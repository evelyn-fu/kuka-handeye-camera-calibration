import numpy as np
from pydrake.all import (
    BasicVector, 
    Context, 
    LeafSystem,
    InputPortIndex,
)
from enum import Enum
from pydrake.common.value import (
    AbstractValue
)
from pydrake.geometry import Meshcat

class ControlState(Enum):
    INIT = 1
    HOLD = 2
    FREE = 3

num_iiwa_positions = 7

class ToggleHoldControlModeSource(LeafSystem):

    def __init__(self, meshcat: Meshcat):
        super().__init__()
        self._mode_index = self.DeclareAbstractState(
            AbstractValue.Make(ControlState.INIT)
        )

        self._hold_position_index = self.DeclareAbstractState(
            AbstractValue.Make([0] * num_iiwa_positions)
        )
        self.DeclareInitializationUnrestrictedUpdateEvent(self.Initialize)

        self._current_position_input_port = self.DeclareVectorInputPort(
            "current_position", num_iiwa_positions
        ).get_index()

        self.DeclareAbstractOutputPort(
            "control_mode",
            lambda: AbstractValue.Make(InputPortIndex(0)),
            self.CalcControlMode,
        )

        self.DeclareVectorOutputPort(
            "hold_state", num_iiwa_positions * 2, self.CalcHoldState
        )

        self.DeclarePeriodicUnrestrictedUpdateEvent(0.1, 0.0, self.Update)

        # read button from meshcat to toggle hold/free
        self._meshcat = meshcat
        self._button = "Toggle hold/free"
        meshcat.AddButton(self._button, "Space")

    def __del__(self):
        self._meshcat.DeleteButton(self._button)

    def Update(self, context, state):
        prev_mode = context.get_abstract_state(int(self._mode_index)).get_value()
        if prev_mode == ControlState.INIT:
            print("to hold")
            current_position = self.get_input_port(self._current_position_input_port).Eval(context)
            print(current_position)
            state.get_mutable_abstract_state(
                int(self._hold_position_index)
            ).set_value(current_position)

            state.get_mutable_abstract_state(
                int(self._mode_index)
            ).set_value(ControlState.HOLD)
            return

        if (self._meshcat.GetButtonClicks(self._button) % 2) == 1:
            if (prev_mode != ControlState.FREE):
                print("to free")

                state.get_mutable_abstract_state(
                    int(self._mode_index)
                ).set_value(ControlState.FREE)
        else:
            if (prev_mode != ControlState.HOLD):
                print("to hold")
                current_position = self.get_input_port(self._current_position_input_port).Eval(context)
                print(current_position)
                state.get_mutable_abstract_state(
                    int(self._hold_position_index)
                ).set_value(current_position)

                state.get_mutable_abstract_state(
                    int(self._mode_index)
                ).set_value(ControlState.HOLD)

    def Initialize(self, context, state):
        current_position = self.get_input_port(self._current_position_input_port).Eval(context)
        print(current_position)
        state.get_mutable_abstract_state(
            int(self._hold_position_index)
        ).set_value(current_position)

    def CalcControlMode(self, context, output):
        mode = context.get_abstract_state(int(self._mode_index)).get_value()

        if mode == ControlState.HOLD:
            output.set_value(InputPortIndex(1))  # Hold - PD control
        else:
            output.set_value(InputPortIndex(2))  # Free - Zero torques

    def CalcHoldState(self, context, output):
        hold_position = context.get_abstract_state(self._hold_position_index).get_value().copy()
        output.SetFromVector(hold_position.tolist() + [0] * num_iiwa_positions)


class StateFromPositionVelocity(LeafSystem):
    def __init__(self):
        super().__init__()

        self._position_input_port = self.DeclareVectorInputPort(
            "position", num_iiwa_positions
        ).get_index()
        self._velocity_input_port = self.DeclareVectorInputPort(
            "velocity", num_iiwa_positions
        ).get_index()
        self.DeclareVectorOutputPort(
            "state", num_iiwa_positions * 2, self._calc_state
        )

    def _calc_state(self, context: Context, output: BasicVector) -> None:
        position = self.get_input_port(self._position_input_port).Eval(context)
        velocity = self.get_input_port(self._velocity_input_port).Eval(context)
        output.SetFromVector(position.tolist() + velocity.tolist())

class ZeroTorqueCommander(LeafSystem):
    def __init__(self):
        super().__init__()

        self.DeclareVectorOutputPort(
            "torque_cmd", num_iiwa_positions, self._calc_torque_value
        )

    def _calc_torque_value(self, context: Context, output: BasicVector) -> None:
        output.SetFromVector(np.zeros(num_iiwa_positions))