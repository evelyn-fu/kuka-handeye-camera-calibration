from pydrake.systems.framework import LeafSystem
import rospy
from rospy import Publisher
import numpy as np

class JointPositionPublisher(LeafSystem):
    def __init__(self, publisher):
        super().__init__()

        self.DeclareVectorInputPort(name="iiwa_position", size=7)

        # Calling `ForcePublish()` will trigger the callback.
        self.DeclareForcedPublishEvent(self.Publish)

        # Publish at 10 hz
        self.DeclarePeriodicPublishEvent(period_sec=0.1,
                                         offset_sec=0,
                                         publish=self.Publish)

        self.pub = publisher
        
    def Publish(self, context):
        position = self.GetInputPort("iiwa_position").Eval(context).data
        self.pub.publish(np.array(position))