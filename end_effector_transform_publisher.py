from pydrake.systems.framework import LeafSystem
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from pydrake.math import (
    RigidTransform,
)
from pydrake.common.value import (
    AbstractValue
)

class EndEffectorTransformPublisher(LeafSystem, Node):
    def __init__(self, plant):
        LeafSystem.__init__(self)
        Node.__init__(self, 'minimal_publisher')

        self._ee_index = plant.GetBodyByName("iiwa_link_7").index()
        self._base_index = plant.GetBodyByName("iiwa_link_0").index()
        self.DeclareAbstractInputPort(
            "body_poses", 
            AbstractValue.Make(
                np.array([RigidTransform] * plant.num_bodies())
            ),
        )

        # Calling `ForcePublish()` will trigger the callback.
        self.DeclareForcedPublishEvent(self.Publish)

        # Publish at 10 hz
        self.DeclarePeriodicPublishEvent(period_sec=0.1,
                                         offset_sec=0,
                                         publish=self.Publish)

        self.tf_broadcaster = TransformBroadcaster(self)
        
    def Publish(self, context):
        X_WEe = self.GetInputPort("body_poses").Eval(context)[int(self._ee_index)]
        X_WBase = self.GetInputPort("body_poses").Eval(context)[int(self._base_index)]

        X_BaseEe = X_WBase.inverse() @ X_WEe

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id="external_base"
        t._child_frame_id = "external_eef"

        p = X_BaseEe.GetAsMatrix4()[:3, 3]

        t.transform.translation.x = p[0]
        t.transform.translation.y = p[1]
        t.transform.translation.z = p[2] 
        
        r = R.from_matrix(X_BaseEe.GetAsMatrix4()[:3, :3])

        t.transform.rotation.x = r.as_quat()[0]
        t.transform.rotation.y = r.as_quat()[1]
        t.transform.rotation.z = r.as_quat()[2]
        t.transform.rotation.w = r.as_quat()[3]

        self.tf_broadcaster.sendTransform(t)    

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id="world"
        t._child_frame_id = "external_eef"

        p = X_WEe.GetAsMatrix4()[:3, 3]

        t.transform.translation.x = p[0]
        t.transform.translation.y = p[1]
        t.transform.translation.z = p[2] 
        
        r = R.from_matrix(X_WEe.GetAsMatrix4()[:3, :3])

        t.transform.rotation.x = r.as_quat()[0]
        t.transform.rotation.y = r.as_quat()[1]
        t.transform.rotation.z = r.as_quat()[2]
        t.transform.rotation.w = r.as_quat()[3]

        self.tf_broadcaster.sendTransform(t)    

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id="world"
        t._child_frame_id = "external_base"

        p = X_WBase.GetAsMatrix4()[:3, 3]

        t.transform.translation.x = p[0]
        t.transform.translation.y = p[1]
        t.transform.translation.z = p[2] 
        
        r = R.from_matrix(X_WBase.GetAsMatrix4()[:3, :3])

        t.transform.rotation.x = r.as_quat()[0]
        t.transform.rotation.y = r.as_quat()[1]
        t.transform.rotation.z = r.as_quat()[2]
        t.transform.rotation.w = r.as_quat()[3]

        self.tf_broadcaster.sendTransform(t)    
