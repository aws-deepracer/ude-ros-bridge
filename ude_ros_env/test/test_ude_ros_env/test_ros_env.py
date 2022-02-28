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
from unittest import mock, TestCase
from unittest.mock import patch, MagicMock, call

from ude_ros_env.ros_env import ROSEnvironment
import ude_ros_env.constants as const

from ude import Space, UDESerializationContext

from ude_ros_msgs.srv import (
    UDEStepSrv, UDEStepSrvRequest, UDEStepSrvResponse,
    UDEResetSrv, UDEResetSrvRequest, UDEResetSrvResponse,
    UDECloseSrv, UDECloseSrvRequest, UDECloseSrvResponse,
    UDEObservationSpaceSrv, UDEObservationSpaceSrvRequest, UDEObservationSpaceSrvResponse,
    UDEActionSpaceSrv, UDEActionSpaceSrvRequest, UDEActionSpaceSrvResponse,
)


@patch("ude_ros_env.ros_env.ROSEnvironmentSideChannel")
@patch("ude_ros_env.ros_env.rospy")
@patch("ude_ros_env.ros_env.ROSEnvironmentInterface")
class ROSEnvironmentNodeTest(TestCase):
    def setUp(self) -> None:
        self._context = UDESerializationContext.get_context()

    def test_initialize(self, ros_env_mock, rospy_mock, ros_env_side_channel_mock):
        ros_env = ROSEnvironment(env=ros_env_mock())
        rospy_mock.Service.assert_has_calls([
            call(const.UDEServiceType.STEP.value,
                 UDEStepSrv,
                 ros_env.step_callback),
            call(const.UDEServiceType.RESET.value,
                 UDEResetSrv,
                 ros_env.reset_callback),
            call(const.UDEServiceType.CLOSE.value,
                 UDECloseSrv,
                 ros_env.close_callback),
            call(const.UDEServiceType.OBSERVATION_SPACE.value,
                 UDEObservationSpaceSrv,
                 ros_env.observation_space_callback),
            call(const.UDEServiceType.ACTION_SPACE.value,
                 UDEActionSpaceSrv,
                 ros_env.action_space_callback)
            ])
        ros_env_side_channel_mock.assert_called_once()

    def test_env(self, ros_env_mock, rospy_mock, ros_env_side_channel_mock):
        ros_env_node = ROSEnvironment(env=ros_env_mock())
        assert ros_env_node.env == ros_env_mock.return_value

    def test_side_channel(self, ros_env_mock, rospy_mock, ros_env_side_channel_mock):
        ros_env_node = ROSEnvironment(env=ros_env_mock())
        assert ros_env_node.side_channel == ros_env_side_channel_mock.return_value

    def test_step(self, ros_env_mock, rospy_mock, ros_env_side_channel_mock):
        action_dict = {"1": 1}
        serialized_action_dict = bytes(self._context.serialize(action_dict).to_buffer())
        request_msg = UDEStepSrvRequest()
        request_msg.data = serialized_action_dict

        step_result = ({"1": "obs"},
                       {"1": 42},
                       {"1": False},
                       {"1": 1},
                       {})
        ros_env_mock.return_value.step.return_value = step_result

        ros_env_node = ROSEnvironment(env=ros_env_mock())
        response = ros_env_node.step_callback(request_msg)

        expected_response_msg = UDEStepSrvResponse()
        expected_response_msg.data = bytes(self._context.serialize(step_result).to_buffer())

        assert response == expected_response_msg

        ros_env_mock.return_value.step.assert_called_once_with(action_dict=action_dict)

    def test_reset(self, ros_env_mock, rospy_mock, ros_env_side_channel_mock):
        obs = {"1": "obs"}
        info = {"info"}
        reset_data = (obs, info)
        ros_env_mock.return_value.reset.return_value =reset_data

        ros_env_node = ROSEnvironment(env=ros_env_mock())
        response = ros_env_node.reset_callback(UDEResetSrvRequest())

        expected_response_msg = UDEResetSrvResponse()
        expected_response_msg.data = bytes(self._context.serialize(reset_data).to_buffer())

        assert response == expected_response_msg

        ros_env_mock.return_value.reset.assert_called_once()

    def test_close(self, ros_env_mock, rospy_mock, ros_env_side_channel_mock):
        ros_env_node = ROSEnvironment(env=ros_env_mock())
        ros_env_node.close_callback(UDECloseSrvRequest())
        ros_env_mock.return_value.close.assert_called_once()

    def test_observation_space(self, ros_env_mock, rospy_mock, ros_env_side_channel_mock):
        observation_space = {"1": Space([4, 3])}
        ros_env_mock.return_value.observation_space = observation_space

        ros_env_node = ROSEnvironment(env=ros_env_mock())
        response = ros_env_node.observation_space_callback(UDEObservationSpaceSrvRequest())

        expected_response_msg = UDEObservationSpaceSrvResponse()
        expected_response_msg.data = bytes(self._context.serialize(observation_space).to_buffer())

        assert response == expected_response_msg

    def test_action_space(self, ros_env_mock, rospy_mock, ros_env_side_channel_mock):
        action_space = {"1": Space([4, 3])}
        ros_env_mock.return_value.action_space = action_space

        ros_env_node = ROSEnvironment(env=ros_env_mock())
        response = ros_env_node.action_space_callback(UDEActionSpaceSrvRequest())

        expected_response_msg = UDEActionSpaceSrvResponse()
        expected_response_msg.data = bytes(self._context.serialize(action_space).to_buffer())

        assert response == expected_response_msg
