
##################################################################################################################

"""
Sim stepper node. Manages the simulation steps and progress
"""

# Built-in/Generic Imports
import sys
import os
import time
from abc import abstractmethod
from typing import List, Optional
from datetime import datetime, timedelta
import numpy as np
import pandas as pd
from json import dumps, loads
from pprint import pprint, pformat

# Libs
# ROS2 Imports
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist, PoseStamped, Point
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# TODO: Cleanup
# NetworkX
import networkx as nx

# Local Imports
from orchestra_config.orchestra_config import *     # KEEP THIS LINE, DO NOT REMOVE
from maaf_msgs.msg import TeamCommStamped, Bid, Allocation
from orchestra_config.sim_config import *

##################################################################################################################

PRINTS = False

class mbawi_sim(Node):
    def __init__(self):
        # ----------------------------------- Node Configuration
        Node.__init__(
            self,
            node_name="mbawi_sim",
        )

        # self.allocation = {}
        # self.msg_count = 0
        # self.cumulative_cost = 0

        self.last_task_update = None
        self.epoch = -1

        # self.environment = None

        # ---------------------------------- Subscribers
        # ---------- simulator_signals
        self.simulator_signals_sub = self.create_subscription(
            msg_type=TeamCommStamped,
            topic=topic_simulator_signals,
            callback=self.simulator_signals_callback,
            qos_profile=qos_simulator_signals
        )

        # ---------- fleet_msgs
        self.fleet_msgs_sub = self.create_subscription(
            msg_type=TeamCommStamped,
            topic=topic_fleet_msgs,
            callback=self._team_msg_subscriber_callback,
            qos_profile=qos_fleet_msgs
        )

        # ---------- goals
        self.goals_sub = self.create_subscription(
            msg_type=TeamCommStamped,
            topic=topic_goals,
            callback=self.goal_callback,
            qos_profile=qos_goal
        )

        # # ---------- environment
        # self.env_sub = self.create_subscription(
        #     msg_type=TeamCommStamped,
        #     topic=topic_environment,
        #     callback=self.env_callback,
        #     qos_profile=qos_env
        # )

        # ---------------------------------- Publishers
        # ---------- epoch
        self.sim_epoch_pub = self.create_publisher(
            msg_type=TeamCommStamped,
            topic=topic_epoch,
            qos_profile=qos_sim_epoch
        )

        # ---------- tasks
        self.tasks_pub = self.create_publisher(
            msg_type=TeamCommStamped,
            topic=topic_tasks,
            qos_profile=qos_tasks
        )

        # ---------- pose
        self.robot_pose_pub = {}

        # -> Create timer to publish second batch of tasks
        self.sim_epoch_pub_timer = self.create_timer(
            timer_period_sec=0.1,
            callback=self.publish_epoch
        )

        time.sleep(5)
        self.get_logger().info("MBaWi simulation node started")

        # -> Update last task update
        self.last_task_update = datetime.now()

    def _team_msg_subscriber_callback(self, msg: TeamCommStamped):
        """
        Callback for the team message subscription.
        """

        self.last_task_update = datetime.now()

        if PRINTS:
            self.get_logger().info(F"Received team message from {msg.source}")

    def goal_callback(self, msg: TeamCommStamped):
        """
        Callback for the goal subscription.
        """
        self.last_task_update = datetime.now()

        if PRINTS:
            self.get_logger().info(F"Received goal from {msg.source}")

    def publish_epoch(self):
        """
        Publish the epoch.
        """

        # -> Simulation has not started
        if self.last_task_update is None:
            return

        if datetime.now() - self.last_task_update > timedelta(seconds=SIM_EPOCH_MIN_DURATION):
            # -> Update epoch
            self.epoch += 1

            # -> Update last task update
            self.last_task_update = datetime.now()

            # -> Construct the message
            msg = TeamCommStamped()

            msg.source = "mbawi_sim"
            msg.target = "all"
            msg.meta_action = "task"
            msg.memo = dumps({
                "epoch": self.epoch,
            })

            self.get_logger().info(f"\n\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
                                   f"\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Stepping simulation : epoch {self.epoch}"
                                   f"\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")

            # -> Publish new epoch
            self.sim_epoch_pub.publish(msg)

    def simulator_signals_callback(self, msg: TeamCommStamped):
        if msg.meta_action == "order 66":
            self.get_logger().info("Received order 66: Terminating simulation")

            # -> Terminate node
            self.destroy_node()

            # -> Terminate script
            sys.exit()


def main(args=None):
    # `rclpy` library is initialized
    rclpy.init(args=args)

    path_sequence = mbawi_sim()

    rclpy.spin(path_sequence)

    path_sequence.destroy_node()
    rclpy.shutdown()
