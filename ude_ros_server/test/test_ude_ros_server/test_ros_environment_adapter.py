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

from ude_ros_server.ros_environment_adapter import ROSEnvironmentAdapter, ROSEnvironmentObserverInterface
from ude import (
    UDEEnvironmentAdapterInterface,
    MultiAgentDict,
    UDEStepResult,
    UDEResetResult,
    UDESerializationContext,
    Discrete
)

from ude_ros_msgs.srv import (
    UDEStepSrv, UDEStepSrvRequest, UDEStepSrvResponse,
    UDEResetSrv, UDEResetSrvResponse,
    UDECloseSrv,
    UDEObservationSpaceSrv, UDEObservationSpaceSrvResponse,
    UDEActionSpaceSrv, UDEActionSpaceSrvResponse
)
import ude_ros_server.constants as const


class DummyObserver(ROSEnvironmentObserverInterface):
    def __init__(self):
        self.mock = MagicMock()

    def on_step(self,
                adapter: UDEEnvironmentAdapterInterface,
                action_dict: MultiAgentDict,
                step_result: UDEStepResult):
        self.mock.on_step(adapter=adapter,
                          action_dict=action_dict,
                          step_result=step_result)

    def on_reset(self,
                 adapter: UDEEnvironmentAdapterInterface,
                 reset_result: UDEResetResult):
        self.mock.on_reset(adapter=adapter,
                           reset_result=reset_result)

    def on_close(self, adapter: UDEEnvironmentAdapterInterface):
        self.mock.on_close(adapter=adapter)


@patch("ude_ros_server.ros_environment_adapter.ServiceProxyWrapper")
@patch("ude_ros_server.ros_environment_adapter.rospy")
class ROSEnvironmentAdapterTest(TestCase):
    def setUp(self) -> None:
        self._context = UDESerializationContext.get_context()
        self._step_cli_mock_obj = MagicMock()
        self._reset_cli_mock_obj = MagicMock()
        self._close_cli_mock_obj = MagicMock()
        self._observation_space_cli_mock_obj = MagicMock()
        self._action_space_cli_mock_obj = MagicMock()

        def service_proxy_mock(service_name, srv):
            if srv == UDEStepSrv and service_name == const.UDEServiceType.STEP.value:
                return self._step_cli_mock_obj
            elif srv == UDEResetSrv and service_name == const.UDEServiceType.RESET.value:
                return self._reset_cli_mock_obj
            elif srv == UDECloseSrv and service_name == const.UDEServiceType.CLOSE.value:
                return self._close_cli_mock_obj
            elif srv == UDEObservationSpaceSrv and service_name == const.UDEServiceType.OBSERVATION_SPACE.value:
                return self._observation_space_cli_mock_obj
            elif srv == UDEActionSpaceSrv and service_name == const.UDEServiceType.ACTION_SPACE.value:
                return self._action_space_cli_mock_obj
            else:
                err_msg = "Unknown srv ({0}) or srv ({0}) and service_name ({1}) don't match.".format(srv,
                                                                                                      service_name)
                raise KeyError(err_msg)
        self._service_proxy_mock = service_proxy_mock

    def test_initialize(self, rospy_mock, service_proxy_wrapper_mock):
        with patch("ude_ros_server.ros_environment_adapter.ROSSideChannel") as ros_side_channel_mock:
            _ = ROSEnvironmentAdapter()
            ros_side_channel_mock.assert_called_once()

            service_proxy_wrapper_mock.assert_has_calls([
                call(const.UDEServiceType.STEP.value,
                     UDEStepSrv),
                call(const.UDEServiceType.RESET.value,
                     UDEResetSrv),
                call(const.UDEServiceType.CLOSE.value,
                     UDECloseSrv)
            ])

    def test_step(self, rospy_mock, service_proxy_wrapper_mock):
        with patch("ude_ros_server.ros_environment_adapter.ROSSideChannel"):
            service_proxy_wrapper_mock.side_effect = self._service_proxy_mock

            action_dict = {"1": 1}
            serialized_action_dict = bytes(self._context.serialize(action_dict).to_buffer())

            expected_observation = {"agent1": [42, 43]}
            serialized_obj = bytes(self._context.serialize(expected_observation).to_buffer())
            response = UDEStepSrvResponse(data=serialized_obj)
            self._step_cli_mock_obj.return_value = response

            adapter = ROSEnvironmentAdapter()
            step_result = adapter.step(action_dict=action_dict)

            assert step_result == expected_observation
            self._step_cli_mock_obj.assert_called_once_with(UDEStepSrvRequest(data=serialized_action_dict))

    def test_reset(self, rospy_mock, service_proxy_wrapper_mock):
        with patch("ude_ros_server.ros_environment_adapter.ROSSideChannel"):

            service_proxy_wrapper_mock.side_effect = self._service_proxy_mock

            expected_observation = {"agent1": [42, 43]}
            expected_info = {"info"}
            expected_reset_result = (expected_observation, expected_info)
            serialized_obj = bytes(self._context.serialize(expected_reset_result).to_buffer())
            response = UDEResetSrvResponse(data=serialized_obj)
            self._reset_cli_mock_obj.return_value = response

            adapter = ROSEnvironmentAdapter()
            obs, info = adapter.reset()
            self._reset_cli_mock_obj.assert_called_once()
            assert obs == expected_observation
            assert expected_info == info

    def test_close(self, rospy_mock, service_proxy_wrapper_mock):
        with patch("ude_ros_server.ros_environment_adapter.ROSSideChannel"):
            service_proxy_wrapper_mock.side_effect = self._service_proxy_mock

            adapter = ROSEnvironmentAdapter()
            adapter.close()
            self._close_cli_mock_obj.assert_called_once()

    def test_step_with_observer(self, rospy_mock, service_proxy_wrapper_mock):
        with patch("ude_ros_server.ros_environment_adapter.ROSSideChannel"):
            service_proxy_wrapper_mock.side_effect = self._service_proxy_mock

            action_dict = {"1": 1}
            serialized_action_dict = bytes(self._context.serialize(action_dict).to_buffer())

            expected_step_result = {"agent1": [42, 43]}
            serialized_obj = bytes(self._context.serialize(expected_step_result).to_buffer())
            response = UDEStepSrvResponse(data=serialized_obj)
            self._step_cli_mock_obj.return_value = response

            adapter = ROSEnvironmentAdapter()
            observer = DummyObserver()
            adapter.register(observer)
            step_result = adapter.step(action_dict=action_dict)

            assert step_result == expected_step_result
            self._step_cli_mock_obj.assert_called_once_with(UDEStepSrvRequest(data=serialized_action_dict))
            observer.mock.on_step.assert_called_once_with(adapter=adapter,
                                                          action_dict=action_dict,
                                                          step_result=expected_step_result)

    def test_reset_with_observer(self, rospy_mock, service_proxy_wrapper_mock):
        with patch("ude_ros_server.ros_environment_adapter.ROSSideChannel"):

            service_proxy_wrapper_mock.side_effect = self._service_proxy_mock

            expected_observation = {"agent1": [42, 43]}
            expected_info = {"info"}
            expected_reset_result = (expected_observation, expected_info)
            serialized_obj = bytes(self._context.serialize(expected_reset_result).to_buffer())
            response = UDEResetSrvResponse(data=serialized_obj)
            self._reset_cli_mock_obj.return_value = response

            adapter = ROSEnvironmentAdapter()
            observer = DummyObserver()
            adapter.register(observer)
            obs, info = adapter.reset()
            self._reset_cli_mock_obj.assert_called_once()
            assert (obs, info) == expected_reset_result
            observer.mock.on_reset.assert_called_once_with(adapter=adapter,
                                                           reset_result=(obs, info))

    def test_close_with_observer(self, rospy_mock, service_proxy_wrapper_mock):
        with patch("ude_ros_server.ros_environment_adapter.ROSSideChannel"):
            service_proxy_wrapper_mock.side_effect = self._service_proxy_mock

            adapter = ROSEnvironmentAdapter()
            observer = DummyObserver()
            adapter.register(observer)
            adapter.close()
            self._close_cli_mock_obj.assert_called_once()
            observer.mock.on_close.assert_called_once_with(adapter=adapter)

    def test_observation_space(self, rospy_mock, service_proxy_wrapper_mock):
        with patch("ude_ros_server.ros_environment_adapter.ROSSideChannel"):
            service_proxy_wrapper_mock.side_effect = self._service_proxy_mock

            expected_observation_space = {"agent1": Discrete(4)}
            serialized_obj = bytes(self._context.serialize(expected_observation_space).to_buffer())
            response = UDEObservationSpaceSrvResponse(data=serialized_obj)
            self._observation_space_cli_mock_obj.return_value = response

            adapter = ROSEnvironmentAdapter()
            observation_space = adapter.observation_space
            self._observation_space_cli_mock_obj.assert_called_once()
            assert observation_space == expected_observation_space

    def test_action_space(self, rospy_mock, service_proxy_wrapper_mock):
        with patch("ude_ros_server.ros_environment_adapter.ROSSideChannel"):
            service_proxy_wrapper_mock.side_effect = self._service_proxy_mock

            expected_action_space = {"agent1": Discrete(4)}
            serialized_obj = bytes(self._context.serialize(expected_action_space).to_buffer())
            response = UDEActionSpaceSrvResponse(data=serialized_obj)
            self._action_space_cli_mock_obj.return_value = response

            adapter = ROSEnvironmentAdapter()
            action_space = adapter.action_space
            self._action_space_cli_mock_obj.assert_called_once()
            assert action_space == expected_action_space

    def test_side_channel(self, rospy_mock, service_proxy_wrapper_mock):
        service_proxy_wrapper_mock.side_effect = self._service_proxy_mock
        with patch("ude_ros_server.ros_environment_adapter.ROSSideChannel") as ros_side_channel_mock:
            adapter = ROSEnvironmentAdapter()
            assert adapter.side_channel == ros_side_channel_mock.return_value
