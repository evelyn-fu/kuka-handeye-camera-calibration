from pydrake.all import AbstractValue, Context, LeafSystem, RigidTransform

class CameraPoseInWorldSource(LeafSystem):

    def __init__(self, X_camera, handeye=True):
        '''
        X_camera = X_ee_camera if using handeye camera, 
        otherwise X_camera = X_WC if camera is stationary
        '''
        super().__init__()

        self.handeye = handeye
        if handeye:
            # The current_cmd that should be passed to output when the current trajectory is invalid
            self._X_EE_input_port = self.DeclareAbstractInputPort(
                "X_EE", AbstractValue.Make(RigidTransform())
            )

        self.DeclareAbstractOutputPort(
            "X_WC",
            lambda: AbstractValue.Make(RigidTransform()),
            self.CalcCameraPose,
        )

        self.X_camera = X_camera

    def CalcCameraPose(self, context: Context, output) -> None:
        if self.handeye:
            X_EE = self.GetInputPort("X_EE").Eval(context)
            X_WC = X_EE @ self.X_camera
        else:
            X_WC = self.X_camera
        
        output.set_value(X_WC)