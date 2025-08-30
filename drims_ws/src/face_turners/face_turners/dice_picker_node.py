#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from drims2_msgs.srv import DiceIdentification
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor         
from rclpy.duration import Duration

from face_turners.core import utils
from drims2_motion_server.motion_client import MotionClient
from face_turners_msgs.srv import FaceDetection

import numpy as np
import time

from copy import deepcopy
from scipy.spatial.transform import Rotation as R
from typing import Tuple


HANDE_ACTION_NAME = '/gripper_action_controller/gripper_cmd'


class DicePickerNode(Node):
    def __init__(self):
        super().__init__('dice_picker')

        self.declare_parameter('dice_face_picker_srv_topic', '/dice_face_picker')
        self.declare_parameter('dice_detection_srv_topic', '/dice_detection_pose')
        dice_face_picker_srv_topic = self.get_parameter('dice_face_picker_srv_topic').get_parameter_value().string_value
        dice_identification_srv_topic = self.get_parameter('dice_detection_srv_topic').get_parameter_value().string_value

        self.motion_client_ = MotionClient(gripper_action_name=HANDE_ACTION_NAME)
        
        # Internal node to call the DiceIdentification service
        self.internal_node_ = Node("dice_picker_internal")
        self.internal_executor_ = MultiThreadedExecutor(num_threads=4)
        self.internal_executor_.add_node(self.internal_node_)

        self.detection_client_ = self.internal_node_.create_client(
            DiceIdentification, dice_identification_srv_topic, callback_group=ReentrantCallbackGroup()
        )
    
        # self.detection_client_ = self.create_client(DiceIdentification, dice_identification_srv_topic)

        self.find_face_srv = self.create_service(FaceDetection, dice_face_picker_srv_topic, self.find_die_face_cb)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.home_joints_ = [
            1.5418810844421387,
            -1.2947982114604493,
            2.21130878130068,
            -2.4932671986021937,
            -1.569287125264303,
            0.020456844940781593
        ]

        self.display_joints_ = [
            1.5417613983154297,
            -1.294870213871338,
            2.211240593587057,
            -3.9874021015562953,
            -1.5694344679461878,
            0.02039862424135208,
        ]

        self.die_detection_joints_ = [
            1.5608530812227754,
            -2.1186817784078946,
            2.5644924706120564,
            -2.0233725131767875,
            -1.5693534039924384,
            0.039808164915431314
        ]
        
        self.flip_die_joints_ = [
            1.5359710293591688,
            -1.0346227024536812,
            1.7331824834092626,
            -2.1741344186648637,
            -1.5691815293356701,
            0.014715908622527595,
        ]
        
        self.get_logger().info("Die Picker Node Initialized")

    def find_die_face_cb(self, request, response):
        # Move to detection configuration
        self.get_logger().info("Moving to detection configuration...")
        self.motion_client_.move_to_joint(self.die_detection_joints_)

        # Find the die face using the detection client
        res, face_number, die_pose = self.detect_die()

        if not res:
            self.get_logger().error("Failed to detect die.")
            response.success = False
            return response

        # Loop and hope for the best
        while  face_number != request.target_face:
            # Pick the dies and display a new face to the camera
            self.pick_die(die_pose)

            time.sleep(1.0)
            res, face_number, die_pose = self.detect_die()

            if not res:
                self.get_logger().error("Failed to detect die.")
                response.success = False
                return response

            if face_number == request.target_face:
                self.place_die(die_pose)
                time.sleep(5.0)
                self.get_logger().info("Target face reached, placing die down.")
                response.success = True
                return response
            
            if (7 - face_number) == request.target_face:
                self.rotate_gripper(degrees=180)
                self.get_logger().info("Target face reached, placing die down.")
                time.sleep(5.0)
                self.place_die(die_pose)
                response.success = True
                return response
            
            self.flip_die()

            time.sleep(1.0)
            res, face_number, die_pose = self.detect_die()

            if not res:
                self.get_logger().error("Failed to detect die.")
                response.success = False
                return response

        self.get_logger().info("Die already showing the correct face.")
        response.success = True
        return response
            
    def detect_die(self) -> Tuple[bool, int, PoseStamped]:
        self.get_logger().info("Detecting die...")
        future = self.detection_client_.call_async(DiceIdentification.Request())
        self.internal_executor_.spin_until_future_complete(future, timeout_sec=5)

        res = future.result()

        if not res.success:
            self.get_logger().error("Failed to detect die.")
            return False, -1, PoseStamped()

        face_number = res.face_number
        pose = res.pose
        self.get_logger().info(f"Detected die face: {face_number}")
        return True, face_number, pose

    def pick_die(self, die_pose: PoseStamped):
        # Approach
        self.get_logger().info("Moving to approach pose...")
        approach_pose = deepcopy(die_pose)
        approach_pose.pose.position.z = -0.07
        self.motion_client_.move_to_pose(approach_pose)

        # Open gripper
        self.get_logger().info("Opening gripper...")
        reached_goal, stalled = self.motion_client_.gripper_command(position=0.045)  # 0.045 = open, adjust depending on gripper model

        # Move to grasp pose
        self.get_logger().info("Moving to grasp pose...")
        grasp_pose = deepcopy(die_pose)
        grasp_pose.pose.position.z = -0.02
        self.motion_client_.move_to_pose(grasp_pose, cartesian_motion=True)

        # Close the gripper
        self.get_logger().info("Closing gripper...")
        reached_goal, stalled = self.motion_client_.gripper_command(position=0.0)  # 0.0 = closed
        self.get_logger().info(f"Gripper closed (reached_goal={reached_goal}, stalled={stalled})")

        # Lift the die
        self.get_logger().info("Lifting the die...")
        self.motion_client_.move_to_pose(approach_pose, cartesian_motion=True)

        # Move to home configuration
        self.get_logger().info("Moving home...")
        self.motion_client_.move_to_joint(self.home_joints_)
        
        # Move to display configuration
        self.get_logger().info("Moving to display configuration...")
        self.motion_client_.move_to_joint(self.display_joints_)

    def rotate_gripper(self, degrees: float = 180):
        # Pose to rotate gripper tip frame w.r.t the Z axis
        T_tip = np.eye(4)
        T_tip[:3, :3] = R.from_euler('z', degrees, degrees=True).as_matrix()

        rotated_tip_pose = PoseStamped()
        rotated_tip_pose.header.frame_id = "tip"
        rotated_tip_pose.pose = utils.T_to_pose(T_tip)

        self.get_logger().info(f"Rotating the e.e. tip by {degrees} degrees...")
        self.motion_client_.move_to_pose(rotated_tip_pose, cartesian_motion=True)

    def place_die(self, place_pose: PoseStamped):
         # Approach
        self.get_logger().info("Approach place pose...")
        approach_pose = deepcopy(place_pose)
        approach_pose.pose.position.z = -0.07
        self.motion_client_.move_to_pose(approach_pose)

        # Move to grasp pose
        self.get_logger().info("Move to release pose...")
        release_pose = deepcopy(place_pose)
        release_pose.pose.position.z = -0.02
        self.motion_client_.move_to_pose(release_pose, cartesian_motion=True)

        # Open the gripper
        self.get_logger().info("Opening gripper...")
        reached_goal, stalled = self.motion_client_.gripper_command(position=0.045)

        # Go back to approach pose
        self.get_logger().info("Moving back to approach pose...")
        self.motion_client_.move_to_pose(approach_pose, cartesian_motion=True)

        # Move to home configuration
        self.get_logger().info("Moving home...")
        self.motion_client_.move_to_joint(self.home_joints_)
        
    def flip_die(self):
        self.get_logger().info("Moving to flip configuration...")
        self.motion_client_.move_to_joint(self.flip_die_joints_)
        
        # Open the gripper
        self.get_logger().info("Opening gripper...")
        reached_goal, stalled = self.motion_client_.gripper_command(position=0.045)

        trans = self.tf_buffer.lookup_transform(
            'base',    # target frame
            'tip',     # source frame
            rclpy.time.Time(),
            timeout=Duration(seconds=2.0)
        )

        tip_pose = PoseStamped()
        tip_pose.header = trans.header
        tip_pose.pose.position.x = trans.transform.translation.x
        tip_pose.pose.position.y = trans.transform.translation.y
        tip_pose.pose.position.z = trans.transform.translation.z
        tip_pose.pose.orientation = trans.transform.rotation

        # Move away vertically from the tip pose
        tip_pose.pose.position.z += 0.1
        self.motion_client_.move_to_pose(tip_pose, cartesian_motion=True)

        # Move to detection configuration
        self.get_logger().info("Moving to detection configuration...")
        self.motion_client_.move_to_joint(self.die_detection_joints_)


def main(args=None):
    rclpy.init(args=args)

    node = DicePickerNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
