# Copyright (C) 2024 Robotec.AI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#         http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#


import rclpy
import rclpy.callback_groups
import rclpy.executors
import rclpy.qos
import rclpy.subscription
import rclpy.task
from langchain.tools.render import render_text_description_and_args

from rai.agents.state_based import create_state_based_agent
from rai.node import RaiNode, describe_ros_image
from rai.tools.ros.native import (
    GetCameraImage,
    GetMsgFromTopic,
    Ros2ShowMsgInterfaceTool,
)
from rai.tools.ros.native_actions import Ros2RunActionSync
from rai.tools.ros.tools import GetOccupancyGridTool
from rai.tools.time import WaitForSecondsTool
from rai.utils.model_initialization import get_llm_model


def main():
    rclpy.init()
    llm = get_llm_model(model_type="complex_model")

    observe_topics = [
        "/front_cam/color/image_raw",
    ]

    observe_postprocessors = {"/front_cam/color/image_raw": describe_ros_image}

    topics_whitelist = [
        "/april_tags",
        "/battery",
        "/behavior_server/transition_event",
        "/behavior_tree_log",
        "/bond",
        "/bt_navigator/transition_event",
        "/clicked_point",
        "/clock",
        "/cmd_vel",
        "/cmd_vel_nav",
        "/cmd_vel_teleop",
        "/controller_server/transition_event",
        "/detected_dock_pose",
        "/diagnostics",
        "/dock_pose",
        "/docking_server/transition_event",
        "/ekf_node/set_pose",
        "/filtered_dock_pose",
        "/front_cam/color/camera_info",
        "/front_cam/color/image_raw",
        "/front_cam/depth/camera_info",
        "/front_cam/depth/image_raw",
        "/front_cam/depth/points",
        "/global_costmap/costmap",
        "/global_costmap/costmap_raw",
        "/global_costmap/costmap_updates",
        "/global_costmap/footprint",
        "/global_costmap/global_costmap/transition_event",
        "/global_costmap/published_footprint",
        "/goal_pose",
        "/hardware/e_stop",
        "/imu/data",
        "/imu_broadcaster/transition_event",
        "/initialpose",
        "/joint_state_broadcaster/transition_event",
        "/joint_states",
        "/lights/driver/channel_1_frame",
        "/lights/driver/channel_2_frame",
        "/local_costmap/costmap",
        "/local_costmap/costmap_raw",
        "/local_costmap/costmap_updates",
        "/local_costmap/footprint",
        "/local_costmap/local_costmap/transition_event",
        "/local_costmap/published_footprint",
        "/lookahead_collision_arc",
        "/lookahead_point",
        "/main_lidar/scan",
        "/odom",
        "/odometry/filtered",
        "/odometry/wheels",
        "/panther_base_controller/transition_event",
        "/parameter_events",
        "/plan",
        "/plan_smoothed",
        "/planner_server/transition_event",
        "/preempt_teleop",
        "/received_global_plan",
        "/robot_description",
        "/rosout",
        "/smoother_server/transition_event",
        "/speed_limit",
        "/staging_pose",
        "/tf",
        "/tf_static",
        "/velocity_smoother/transition_event",
        "/waypoint_follower/transition_event"
    ]

    actions_whitelist = [
        "/assisted_teleop",
        "/backup",
        "/compute_path_through_poses",
        "/compute_path_to_pose",
        "/dock_robot",
        "/drive_on_heading",
        "/follow_path",
        "/follow_waypoints",
        "/navigate_through_poses",
        "/navigate_to_pose",
        "/smooth_path",
        "/spin",
        "/undock_robot",
        "/wait",
    ]

    # TODO(boczekbartek): refactor system prompt

    SYSTEM_PROMPT = ""

    node = RaiNode(
        llm=get_llm_model(
            model_type="simple_model"
        ),  # smaller model used to describe the environment
        observe_topics=observe_topics,
        observe_postprocessors=observe_postprocessors,
        whitelist=topics_whitelist + actions_whitelist,
        system_prompt=SYSTEM_PROMPT,
    )

    tools = [
        WaitForSecondsTool(),
        GetMsgFromTopic(node=node),
        Ros2RunActionSync(node=node),
        GetCameraImage(node=node),
        Ros2ShowMsgInterfaceTool(),
        GetOccupancyGridTool(),
    ]

    state_retriever = node.get_robot_state

    SYSTEM_PROMPT = f"""You are an autonomous robot connected to ros2 environment. Your main goal is to fulfill the user's requests.
    Do not make assumptions about the environment you are currently in.
    Use the tooling provided to gather information about the environment:

    {render_text_description_and_args(tools)}

    You can use ros2 topics, services and actions to operate. """

    node.get_logger().info(f"{SYSTEM_PROMPT=}")

    node.system_prompt = node.initialize_system_prompt(SYSTEM_PROMPT)

    app = create_state_based_agent(
        llm=llm,
        tools=tools,
        state_retriever=state_retriever,
        logger=node.get_logger(),
    )

    node.set_app(app)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


main()
