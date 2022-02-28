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
"""A class for ROS Environment."""
from typing import cast

import ude_ros_env.constants as const
from ude_ros_env.ros_env_side_channel import ROSEnvironmentSideChannel
from ude_ros_env.ros_env_interface import ROSEnvironmentInterface

from ude import MultiAgentDict, UDESerializationContext

import rospy

from ude_ros_msgs.srv import (
    UDEStepSrv, UDEStepSrvRequest, UDEStepSrvResponse,
    UDEResetSrv, UDEResetSrvRequest, UDEResetSrvResponse,
    UDECloseSrv, UDECloseSrvRequest, UDECloseSrvResponse,
    UDEObservationSpaceSrv, UDEObservationSpaceSrvRequest, UDEObservationSpaceSrvResponse,
    UDEActionSpaceSrv, UDEActionSpaceSrvRequest, UDEActionSpaceSrvResponse,
)


class ROSEnvironment:
    """
    ROS UDE Environment Node
    """
    def __init__(self, env: ROSEnvironmentInterface):
        """
        Initialize EnvNode
        Args:
            env (ROSEnvironmentInterface): environment implementation
        """
        rospy.loginfo("[ROSEnvironment] initiating...")
        self._context = UDESerializationContext.get_context()
        self._env = env
        # Create sync service
        self._step_srv = rospy.Service(const.UDEServiceType.STEP.value,
                                       UDEStepSrv,
                                       self.step_callback)
        self._reset_srv = rospy.Service(const.UDEServiceType.RESET.value,
                                        UDEResetSrv,
                                        self.reset_callback)
        self._close_srv = rospy.Service(const.UDEServiceType.CLOSE.value,
                                        UDECloseSrv,
                                        self.close_callback)
        self._observation_space_srv = rospy.Service(const.UDEServiceType.OBSERVATION_SPACE.value,
                                                    UDEObservationSpaceSrv,
                                                    self.observation_space_callback)
        self._action_space_srv = rospy.Service(const.UDEServiceType.ACTION_SPACE.value,
                                               UDEActionSpaceSrv,
                                               self.action_space_callback)

        self._side_channel = ROSEnvironmentSideChannel()
        rospy.loginfo("[ROSEnvironment] initiating [DONE]")

    @property
    def env(self) -> ROSEnvironmentInterface:
        """
        Returns the instantiation of actual environment implementation

        Returns:
            ROSEnvironmentInterface: the instantiation of actual environment implementation

        """
        return self._env

    @property
    def side_channel(self) -> ROSEnvironmentSideChannel:
        """
        Returns the side channel attached to this environment

        Returns:
            ROSEnvironmentSideChannel: side channel
        """
        return self._side_channel

    def step_callback(self, request: UDEStepSrvRequest) -> UDEStepSrvResponse:
        """
        Handle step requests.

        Args:
            request (UDEStepSrvRequest): step request message

        Returns:
            UDEStepSrvResponse: step response message

        """
        rospy.logdebug("[ROSEnvironment] step_callback")
        action_dict = cast(MultiAgentDict, self._context.deserialize(bytes(request.data)))

        step_result = self.env.step(action_dict=action_dict)
        serialized_obj = bytes(self._context.serialize(step_result).to_buffer())
        response = UDEStepSrvResponse()
        response.data = serialized_obj
        rospy.logdebug("[ROSEnvironment] step_callback [DONE]")
        return response

    def reset_callback(self, request: UDEResetSrvRequest) -> UDEResetSrvResponse:
        """
        Handle reset requests.

        Args:
            request (UDEResetSrvRequest): reset request message

        Returns:
            UDEResetSrvResponse: reset response message
        """
        rospy.logdebug("[ROSEnvironment] reset_callback")
        reset_result = self.env.reset()
        serialized_obj = bytes(self._context.serialize(reset_result).to_buffer())
        response = UDEResetSrvResponse()
        response.data = serialized_obj
        rospy.logdebug("[ROSEnvironment] reset_callback [DONE]")
        return response

    def close_callback(self, request: UDECloseSrvRequest) -> UDECloseSrvResponse:
        """
        Handle reset requests.

        Args:
            request (UDECloseSrvRequest): close request message

        Returns:
            UDECloseSrvResponse: close response message
        """
        rospy.logdebug("[ROSEnvironment] close_callback")
        self.env.close()
        rospy.logdebug("[ROSEnvironment] close_callback [DONE]")
        return UDECloseSrvResponse()

    def observation_space_callback(self, request: UDEObservationSpaceSrvRequest) -> UDEObservationSpaceSrvResponse:
        """
        Handle observation space requests.

        Args:
            request (UDEObservationSpaceSrvRequest): observation space request message

        Returns:
            UDEObservationSpaceSrvResponse: observation space response message
        """
        rospy.logdebug("[ROSEnvironment] observation_space_callback")
        observation_space = self.env.observation_space
        serialized_obj = bytes(self._context.serialize(observation_space).to_buffer())
        response = UDEObservationSpaceSrvResponse()
        response.data = serialized_obj
        rospy.logdebug("[ROSEnvironment] observation_space_callback [DONE]")
        return response

    def action_space_callback(self, request: UDEActionSpaceSrvRequest) -> UDEActionSpaceSrvResponse:
        """
        Handle action space requests.

        Args:
            request (UDEActionSpaceSrvRequest): action space request message

        Returns:
            UDEActionSpaceSrvResponse: action space response message
        """
        rospy.logdebug("[ROSEnvironment] action_space_callback")
        action_space = self.env.action_space
        serialized_obj = bytes(self._context.serialize(action_space).to_buffer())
        response = UDEActionSpaceSrvResponse()
        response.data = serialized_obj
        rospy.logdebug("[ROSEnvironment] action_space_callback [DONE]")
        return response
