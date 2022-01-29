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
"""A class for ROS Side Channel."""
import numpy as np

from ude import (
    AbstractSideChannel, SideChannelData,
    BUILTIN_TYPE_TO_SIDE_CHANNEL_DATATYPE
)

import ude_ros_server.constants as const
from ude_ros_server.service_proxy_wrapper import ServiceProxyWrapper

from ude_ros_msgs.srv import (
    UDESideChannelSrv,
    UDESideChannelSrvRequest, UDESideChannelSrvResponse
)

import rospy


class ROSSideChannel(AbstractSideChannel):
    """
    ROSSideChannel to be used for ROS Environment.
    """
    def __init__(self):
        """
        Initialize ROSSideChannel
        """
        super().__init__()

        rospy.loginfo("[ROSSideChannel] initiating...")

        # Create to_ude service
        self._to_ude_service = rospy.Service(const.SideChannelServiceType.TO_UDE.value,
                                             UDESideChannelSrv,
                                             self.on_message_received)

        # Create to_env client.
        rospy.loginfo("[ROSSideChannel] wait for to_env service...")
        self._send_msg_to_env_cli = ServiceProxyWrapper(const.SideChannelServiceType.TO_ENV.value,
                                                        UDESideChannelSrv)
        rospy.loginfo("[ROSSideChannel] to_env service available...")

    def on_message_received(self, request: UDESideChannelSrvRequest) -> UDESideChannelSrvResponse:
        """
        Handle side channel message received from ROS environment.

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
        rospy.loginfo("[ROSSideChannel] on_message_received (key={}, value={})...".format(key,
                                                                                          str(value)))
        self.store(key=key, value=value)
        self.notify(key=key, value=value)

        response = UDESideChannelSrvResponse()
        return response

    def _send(self, key: str, value: SideChannelData, store_local: bool = False) -> None:
        """
        Send the side channel message to ROS environment

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

        self._send_msg_to_env_cli(req)
