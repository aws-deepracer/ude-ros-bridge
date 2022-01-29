#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################
"""A class for ROS Environment Side Channel."""
import abc
import numpy as np

import ude_ros_env.constants as const
from ude_ros_env.service_proxy_wrapper import ServiceProxyWrapper

from ude import (
    AbstractSideChannel, SideChannelData,
    BUILTIN_TYPE_TO_SIDE_CHANNEL_DATATYPE
)

from ude_ros_msgs.srv import (
    UDESideChannelSrv,
    UDESideChannelSrvRequest, UDESideChannelSrvResponse
)

import rospy

# Python 2 and 3 compatible Abstract class
ABC = abc.ABCMeta('ABC', (object,), {})


class ROSEnvironmentSideChannel(AbstractSideChannel):
    """
    ROSEnvironmentSideChannel to be used in ROS Environment.
    """
    def __init__(self):
        """
        Initialize ROSEnvironmentSideChannel.
        """
        super().__init__()

        rospy.loginfo("[ROSEnvironmentSideChannel] initiating...")

        # Create to_env service
        self._to_env_service = rospy.Service(const.SideChannelServiceType.TO_ENV.value,
                                             UDESideChannelSrv,
                                             self.on_message_received)

        # Create to_ude client.
        rospy.loginfo("[ROSEnvironmentSideChannel] wait for to_ude service...")
        self._send_msg_to_ude_cli = ServiceProxyWrapper(const.SideChannelServiceType.TO_UDE.value,
                                                        UDESideChannelSrv)
        rospy.loginfo("[ROSEnvironmentSideChannel] to_ude service available...")

    def on_message_received(self, request: UDESideChannelSrvRequest) -> UDESideChannelSrvResponse:
        """
        Handle side channel message received from UDE.

        Args:
            request (UDESideChannelSrvRequest): side channel message
        """
        key = request.key
        data_type = request.data_type
        try:
            value = getattr(request.data, const.SIDE_CHANNEL_DATATYPE_TO_ROS_MSG_ATTR_MAP[data_type])
        except KeyError:
            raise TypeError("Not supported type: {}".format(data_type))
        if data_type == const.SideChannelDataType.FLOAT_LIST.value:
            value = list(value)
        elif data_type == const.SideChannelDataType.BYTES.value:
            value = bytes(value)

        rospy.loginfo("[ROSEnvironmentSideChannel] on_message_received (key={}, value={})...".format(key,
                                                                                                     str(value)))

        self.store(key=key, value=value)
        self.notify(key=key, value=value)

        response = UDESideChannelSrvResponse()
        return response

    def _send(self, key: str, value: SideChannelData, store_local: bool = False) -> None:
        """
        Send the side channel message to ROS Server

        Args:
            key (str): The string identifier of message
            value (SideChannelData): The data of the message.
            store_local (bool, optional): The flag whether to store locally or not.
        """
        req = UDESideChannelSrvRequest()
        req.key = key

        if type(value).__module__ == np.__name__:
            value = value.item()

        try:
            req.data_type = BUILTIN_TYPE_TO_SIDE_CHANNEL_DATATYPE[type(value)]
            setattr(req.data, const.BUILTIN_TYPE_TO_ROS_MSG_ATTR_MAP[type(value)], value)
        except KeyError:
            raise TypeError("Not supported type: {}".format(type(value)))
        self._send_msg_to_ude_cli(req)
