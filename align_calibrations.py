# Imports
import numpy as np
from typing import List
from pydrake.all import PointCloud, Rgba, RigidTransform, RotationMatrix, StartMeshcat
from scipy.spatial import KDTree
import argparse
import random
from enum import Enum
import os
from scipy.spatial.transform import Rotation as R
import open3d as o3d
from pydrake.geometry import (
    StartMeshcat,
    RenderLabel,
    Role,
)
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.sensors import CameraInfo
from pydrake.systems.primitives import (
    PortSwitch,
    Multiplexer,
    Demultiplexer
)
from pydrake.perception import (
    DepthImageToPointCloud
)

from manipulation.scenarios import AddIiwaDifferentialIK
from manipulation.systems import ExtractPose
from manipulation.station import MakeHardwareStation, LoadScenario

import pydrake.planning as mut
from pydrake.common import RandomGenerator, Parallelism, use_native_cpp_logging
from pydrake.planning import (RobotDiagramBuilder,
                              SceneGraphCollisionChecker,
                              CollisionCheckerParams)
from pydrake.math import (
    RigidTransform,
    RotationMatrix,
    RollPitchYaw,
)
from camera_in_world import CameraPoseInWorldSource
from pydrake.all import AbstractValue, Context, LeafSystem, RigidTransform, BaseField


class AlignPcdsSystemState(Enum):
    IDLE = 1
    SHOW = 2
    ALIGN = 3
    SAVE = 4

class AlignPcds(LeafSystem):
    def __init__(
            self, 
            meshcat, 
            num_cameras=3, 
            camera_names=("front", "back_right", "back_left"), 
            main_camera="front", 
            initial_calibrations=None,
        ):
        '''
        X_camera = X_ee_camera if using handeye camera, 
        otherwise X_camera = X_WC if camera is stationary
        '''
        super().__init__()

        assert len(camera_names) == num_cameras

        for i in range(num_cameras):
            print("AlignPcds: Creating input port", camera_names[i])
            self.DeclareAbstractInputPort(camera_names[i], AbstractValue.Make(PointCloud(0)))

        self.num_cameras = num_cameras
        self.camera_names = camera_names
        self.main_camera = main_camera
        self.main_camera_ind = camera_names.index(main_camera)
        if initial_calibrations == None:
            initial_calibrations = [RigidTransform() for i in range(num_cameras)]
        self.initial_calibrations = initial_calibrations

        self.alignments = [RigidTransform() for i in range(num_cameras)]
        self.meshcat = meshcat

        # Publish at 10 hz
        self.DeclarePeriodicPublishEvent(period_sec=0.1,
                                         offset_sec=0,
                                         publish=self.Publish)
        
        self.state = AlignPcdsSystemState.IDLE
        self.dirstr = "temp"
    
    def updateState(self, new_state: AlignPcdsSystemState) -> None:
        self.state = new_state

    def Publish(self, context: Context) -> None:
        if self.state != AlignPcdsSystemState.IDLE:
            if self.state == AlignPcdsSystemState.SHOW:
                self.ShowUncalibrated(context)
            if self.state == AlignPcdsSystemState.ALIGN:
                self.AlignCalibrations(context)
            if self.state == AlignPcdsSystemState.SAVE:
                self.SaveCalibrations()
            self.state = AlignPcdsSystemState.IDLE

    def ShowUncalibrated(self, context: Context) -> None:
        '''
        Rerenders pointclouds in meshcat from all cameras with current calibration
        '''
        for i in range(self.num_cameras):
            cloud = self.GetInputPort(self.camera_names[i]).Eval(context)
            down_sampled_pcd = cloud.VoxelizedDownSample(voxel_size=0.005)
            self.meshcat.SetObject(
                self.camera_names[i], 
                down_sampled_pcd, 
                point_size=0.005
            )

    def AlignCalibrations(self, context: Context) -> None:
        '''
        Aligns cameras to the main camera using icp
        Rerenders pointclouds in meshcat from all cameras with current calibration
        '''
        main_pcd = self.GetInputPort(self.main_camera).Eval(context)
        main_pcd = main_pcd.Crop(lower_xyz=[0.0, -0.5, 0.1], upper_xyz=[1.3, 0.5, 1.5])
        source = o3d.geometry.PointCloud()
        source.points = o3d.utility.Vector3dVector(main_pcd.xyzs().T)
        self.meshcat.SetObject(
            self.main_camera, 
            main_pcd, 
            point_size=0.005, 
        )

        for i in range(self.num_cameras):
            if i == self.main_camera_ind:
                continue
            
            print("aligning", self.camera_names[i])

            cloud = self.GetInputPort(self.camera_names[i]).Eval(context)
            cloud = cloud.Crop(lower_xyz=[0.0, -0.5, -0.1], upper_xyz=[1.3, 0.5, 1.5])

            # align using open3d
            trans_init = np.eye(4)
            threshold = 0.001
            target = o3d.geometry.PointCloud()
            target.points = o3d.utility.Vector3dVector(cloud.xyzs().T)
            reg_p2p = o3d.pipelines.registration.registration_icp(
                source, target, threshold, trans_init,
                o3d.pipelines.registration.TransformationEstimationPointToPoint()
            )

            print(reg_p2p)
            print("Transformation is:")
            print(reg_p2p.transformation)
            print("")

            self.alignments[i] = RigidTransform(reg_p2p.transformation)

            transformed_xyzs = RigidTransform(self.alignments[i]) @ cloud.xyzs()
            cloud.mutable_xyzs()[:] = transformed_xyzs

            self.meshcat.SetObject(
                self.camera_names[i], 
                cloud, 
                point_size=0.005, 
            )

    def SaveCalibrations(self) -> None:
        '''
        Takes in old calibrations and calculates new calibrations from self.alignments
        Save all new calibrations to dirstr/[camera name].txt as 4x4 array
        '''
        save_dir_path = os.path.abspath(os.path.join(os.path.dirname( __file__ ), 'calibrations', self.dirstr))
        if not os.path.exists(save_dir_path):
            os.makedirs(save_dir_path)
        for i in range(self.num_cameras):
            print(self.alignments[i], self.initial_calibrations[i])
            new_calibration = self.alignments[i] @ self.initial_calibrations[i]
            np.savetxt(save_dir_path + "/" + self.camera_names[i] + ".txt", new_calibration.GetAsMatrix4())

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scenario_path",
        default="scenario_data_cameras.yml",
        help="yaml file with scenario",
    )
    parser.add_argument(
        "--main_camera",
        default="back_right",
        help="which camera to align to",
    )
    parser.add_argument(
        "--save_dir",
        default="temp",
        help="directory to save calibrations to",
    )
    args = parser.parse_args()

    # Start the visualizer.
    meshcat = StartMeshcat()

    meshcat.ResetRenderMode()

    builder = DiagramBuilder()

    dir_path = os.path.dirname(os.path.realpath(__file__))
    filename = os.path.join(dir_path, args.scenario_path)
    scenario = LoadScenario(filename=filename)
    station = builder.AddSystem(MakeHardwareStation(scenario, meshcat, hardware=True))

    # from realsense intrinsics
    front_camera_pcd = builder.AddSystem(
        DepthImageToPointCloud(
            camera_info=CameraInfo(848, 480, 600.165, 600.165, 429.152, 232.822),
            fields=BaseField.kXYZs | BaseField.kRGBs,
        )
    )
    back_right_pcd = builder.AddSystem(
        DepthImageToPointCloud(
            camera_info=CameraInfo(848, 480, 626.633, 626.633, 432.041, 245.465),
            fields=BaseField.kXYZs | BaseField.kRGBs,
        )
    )
    back_left_pcd = builder.AddSystem(
        DepthImageToPointCloud(
            camera_info=CameraInfo(848, 480, 596.492, 596.492, 416.694, 240.225),
            fields=BaseField.kXYZs | BaseField.kRGBs,
        )
    )

    builder.Connect(station.GetOutputPort("front.depth_image"), front_camera_pcd.GetInputPort("depth_image"))
    builder.Connect(station.GetOutputPort("front.rgb_image"), front_camera_pcd.GetInputPort("color_image"))
    builder.Connect(station.GetOutputPort("back_right.depth_image"), back_right_pcd.GetInputPort("depth_image"))
    builder.Connect(station.GetOutputPort("back_right.rgb_image"), back_right_pcd.GetInputPort("color_image"))
    builder.Connect(station.GetOutputPort("back_left.depth_image"), back_left_pcd.GetInputPort("depth_image"))
    builder.Connect(station.GetOutputPort("back_left.rgb_image"), back_left_pcd.GetInputPort("color_image"))

    # from camera calibation
    # Front camera
    x_front_rgb = RigidTransform(np.loadtxt("/home/real2sim/calibrations/12_6_calibrations/front_calibration_12_6_daniilidis.txt"))

    # Back Right camera
    x_back_right_rgb = RigidTransform(np.loadtxt("/home/real2sim/calibrations/12_6_calibrations/back_right_calibration_12_5_daniilidis.txt"))

    # Back Left camera
    x_back_left_rgb = RigidTransform(np.loadtxt("/home/real2sim/calibrations/12_6_calibrations/back_left_calibration_12_5_daniilidis.txt"))

    # rgb calibration to depth calibration (from realsense specs)
    # Front camera
    x_depth_rgb_front = RigidTransform([[0.999986,      -0.000127587,   0.00531376, 0.015102],
                                        [0.000116105,   0.999998,       0.00216102, 6.44158e-05],
                                        [-0.00531402,   -0.00216038,    0.999984,   -0.000426644],
                                        [0,             0,              0,          1]])
    x_front_camera = x_front_rgb @ x_depth_rgb_front

    # Back Right camera
    x_depth_rgb_back_right = RigidTransform([[0.999968,  -0.00700185,   0.00399879,     0.015085],
                                            [ 0.00701494,      0.99997,  -0.00326805,  -2.1265e-05],
                                            [-0.00397579,   0.00329599,     0.999987, -0.000455872],
                                            [          0,            0,            0,            1]])
    x_back_right_camera = x_back_right_rgb @ x_depth_rgb_back_right

    # Back Left camera
    x_depth_rgb_back_left = RigidTransform([[0.999998, -0.000191981,  -0.00215977,    0.0150991],
                                            [0.000214442,     0.999946,    0.0104041,  7.71731e-05],
                                            [0.00215765,   -0.0104046,     0.999944, -0.000317806],
                                            [          0,            0,            0,            1]])
    x_back_left_camera = x_back_left_rgb @ x_depth_rgb_back_left

    og_calibrations = [x_front_camera, x_back_right_camera, x_back_left_camera]

    # connect camera pcd pose sources
    front_camera_pose_source = builder.AddSystem(CameraPoseInWorldSource(x_front_camera, handeye=False))
    back_right_camera_pose_source = builder.AddSystem(CameraPoseInWorldSource(x_back_right_camera, handeye=False))
    back_left_camera_pose_source = builder.AddSystem(CameraPoseInWorldSource(x_back_left_camera, handeye=False))

    builder.Connect(
        front_camera_pose_source.GetOutputPort("X_WC"),
        front_camera_pcd.GetInputPort("camera_pose"),
    )
    builder.Connect(
        back_right_camera_pose_source.GetOutputPort("X_WC"),
        back_right_pcd.GetInputPort("camera_pose"),
    )
    builder.Connect(
        back_left_camera_pose_source.GetOutputPort("X_WC"),
        back_left_pcd.GetInputPort("camera_pose"),
    )

    align_system = builder.AddSystem(AlignPcds(meshcat, initial_calibrations=og_calibrations))
    align_system.dirstr = args.save_dir
    builder.Connect(
        front_camera_pcd.GetOutputPort("point_cloud"),
        align_system.GetInputPort("front"),
    )
    builder.Connect(
        back_right_pcd.GetOutputPort("point_cloud"),
        align_system.GetInputPort("back_right"),
    )
    builder.Connect(
        back_left_pcd.GetOutputPort("point_cloud"),
        align_system.GetInputPort("back_left"),
    )

    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    system_context = align_system.GetMyContextFromRoot(context)
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)

    meshcat.AddButton("Stop Simulation")
    meshcat.AddButton("Show PCDs")
    meshcat.AddButton("Align")
    meshcat.AddButton("Save Calibrations")
    show_clicks = meshcat.GetButtonClicks("Show PCDs")
    align_clicks = meshcat.GetButtonClicks("Align")
    save_clicks = meshcat.GetButtonClicks("Save Calibrations")
    while meshcat.GetButtonClicks("Stop Simulation") < 1:
        if meshcat.GetButtonClicks("Show PCDs") > show_clicks:
            align_system.updateState(AlignPcdsSystemState.SHOW)
        if meshcat.GetButtonClicks("Align") > align_clicks:
            align_system.updateState(AlignPcdsSystemState.ALIGN)
        if meshcat.GetButtonClicks("Save Calibrations") > save_clicks:
            align_system.updateState(AlignPcdsSystemState.SAVE)
        
        show_clicks = meshcat.GetButtonClicks("Show PCDs")
        align_clicks = meshcat.GetButtonClicks("Align")
        save_clicks = meshcat.GetButtonClicks("Save Calibrations")
        simulator.AdvanceTo(simulator.get_context().get_time() + 0.1)

    meshcat.DeleteButton("Stop Simulation")
    meshcat.DeleteButton("Show PCDs")
    meshcat.DeleteButton("Align")
    meshcat.DeleteButton("Save Calibrations")