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
from unittest.mock import patch, MagicMock, ANY, call

from ude_ros_server.ude_ros_server_node import UDEROSServer
import ude_ros_server.constants as const

from ude import (
    UDEStepInvokeType,
    UDEResetMode,
    UDE_COMM_DEFAULT_PORT,
)

import grpc

import rospy
from ude_ros_msgs.srv import (
    SetUDEServerConfig,
    SetUDEServerConfigRequest, SetUDEServerConfigResponse,
    GetUDEServerConfig,
    GetUDEServerConfigRequest, GetUDEServerConfigResponse
)


@patch("ude_ros_server.ude_ros_server_node.rospy.Service")
@patch("ude_ros_server.ude_ros_server_node.ROSEnvironmentAdapter")
@patch("ude_ros_server.ude_ros_server_node.UDEEnvironment")
@patch("ude_ros_server.ude_ros_server_node.UDEServer")
class UDEROSServerNodeTest(TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rospy.init_node(name="ude_ros_server",
                        anonymous=False,
                        log_level=rospy.INFO)

    def test_setup(self, ude_server_mock, ude_env_mock, ros_env_adapter_mock, rospy_service_mock):
        ude_server = UDEROSServer()
        ros_env_adapter_mock.assert_called_once()

        ros_env_adapter_mock.return_value.register.assert_called_once_with(ude_server)

        ude_env_mock.assert_called_once_with(ude_env_adapter=ros_env_adapter_mock.return_value,
                                             reset_mode=ANY,
                                             game_over_cond=ANY)
        ude_server_mock.assert_called_once_with(ude_env=ude_env_mock.return_value,
                                                step_invoke_type=ANY,
                                                step_invoke_period=ANY,
                                                num_agents=ANY,
                                                port=ANY,
                                                compression=ANY,
                                                credentials=ANY,
                                                timeout_wait=ANY)
        ude_server_mock.return_value.start.assert_called_once()

        rospy_service_mock.has_calls(
            call(const.UDEROSServerServiceType.SET_UDE_SERVER_CONFIG.value,
                 SetUDEServerConfig,
                 ude_server.on_config_received),
            call(const.UDEROSServerServiceType.GET_UDE_SERVER_CONFIG.value,
                 GetUDEServerConfig,
                 ude_server.on_config_request),
        )

        assert ude_server.ude_ros_server_config_target.reset_mode == UDEResetMode.MANUAL
        assert ude_server.ude_ros_server_config_target.game_over_cond == any
        assert ude_server.ude_ros_server_config_target.step_invoke_type == UDEStepInvokeType.WAIT_FOREVER
        assert ude_server.ude_ros_server_config_target.step_invoke_period == 120.0
        assert ude_server.ude_ros_server_config_target.num_agents == 1
        assert ude_server.ude_ros_server_config_target.timeout_wait == 60.0
        assert ude_server.ude_ros_server_config_target.port == UDE_COMM_DEFAULT_PORT
        assert ude_server.ude_ros_server_config_target.compression == grpc.Compression.NoCompression

        assert ude_server.ude_ros_server_config.reset_mode == UDEResetMode.MANUAL
        assert ude_server.ude_ros_server_config.game_over_cond == any
        assert ude_server.ude_ros_server_config.step_invoke_type == UDEStepInvokeType.WAIT_FOREVER
        assert ude_server.ude_ros_server_config.step_invoke_period == 120.0
        assert ude_server.ude_ros_server_config.num_agents == 1
        assert ude_server.ude_ros_server_config.timeout_wait == 60.0
        assert ude_server.ude_ros_server_config.port == UDE_COMM_DEFAULT_PORT
        assert ude_server.ude_ros_server_config.compression == grpc.Compression.NoCompression

    def test_on_config_received(self, ude_server_mock, ude_env_mock, ros_env_adapter_mock, rospy_service_mock):
        ude_server = UDEROSServer()

        req = SetUDEServerConfigRequest()
        req.reset_mode = UDEResetMode.AUTO
        req.game_over_cond = "all"
        req.step_invoke_type = UDEStepInvokeType.PERIODIC.value
        req.step_invoke_period = 3.0
        req.num_agents = 4
        req.timeout_wait = 40.0

        ude_server.on_config_received(req)

        # Target config should get updated
        assert ude_server.ude_ros_server_config_target.reset_mode == UDEResetMode.AUTO
        assert ude_server.ude_ros_server_config_target.game_over_cond == all
        assert ude_server.ude_ros_server_config_target.step_invoke_type == UDEStepInvokeType.PERIODIC
        assert ude_server.ude_ros_server_config_target.step_invoke_period == 3.0
        assert ude_server.ude_ros_server_config_target.num_agents == 4
        assert ude_server.ude_ros_server_config_target.timeout_wait == 40.0
        assert ude_server.ude_ros_server_config_target.port == UDE_COMM_DEFAULT_PORT
        assert ude_server.ude_ros_server_config_target.compression == grpc.Compression.NoCompression

        # No change to current config
        assert ude_server.ude_ros_server_config.reset_mode == UDEResetMode.MANUAL
        assert ude_server.ude_ros_server_config.game_over_cond == any
        assert ude_server.ude_ros_server_config.step_invoke_type == UDEStepInvokeType.WAIT_FOREVER
        assert ude_server.ude_ros_server_config.step_invoke_period == 120.0
        assert ude_server.ude_ros_server_config.num_agents == 1
        assert ude_server.ude_ros_server_config.timeout_wait == 60.0
        assert ude_server.ude_ros_server_config.port == UDE_COMM_DEFAULT_PORT
        assert ude_server.ude_ros_server_config.compression == grpc.Compression.NoCompression

    def test_on_config_request(self, ude_server_mock, ude_env_mock, ros_env_adapter_mock, rospy_service_mock):
        ude_server = UDEROSServer()

        req = GetUDEServerConfigRequest()
        res = ude_server.on_config_request(req)
        assert res.reset_mode == UDEResetMode.MANUAL.value
        assert res.game_over_cond == 'any'
        assert res.step_invoke_type == UDEStepInvokeType.WAIT_FOREVER.value
        assert res.step_invoke_period == 120.0
        assert res.num_agents == 1
        assert res.timeout_wait == 60.0
        assert res.port == UDE_COMM_DEFAULT_PORT
        assert res.compression == const.CompressionMode.NO_COMPRESSION

    def test_on_config_request_after_set(self, ude_server_mock, ude_env_mock, ros_env_adapter_mock, rospy_service_mock):
        ude_server = UDEROSServer()

        req = SetUDEServerConfigRequest()
        req.reset_mode = UDEResetMode.AUTO
        req.game_over_cond = "all"
        req.step_invoke_type = UDEStepInvokeType.PERIODIC.value
        req.step_invoke_period = 3.0
        req.num_agents = 4
        req.timeout_wait = 40.0

        ude_server.on_config_received(req)

        # Target config should get updated
        assert ude_server.ude_ros_server_config_target.reset_mode == UDEResetMode.AUTO
        assert ude_server.ude_ros_server_config_target.game_over_cond == all
        assert ude_server.ude_ros_server_config_target.step_invoke_type == UDEStepInvokeType.PERIODIC
        assert ude_server.ude_ros_server_config_target.step_invoke_period == 3.0
        assert ude_server.ude_ros_server_config_target.num_agents == 4
        assert ude_server.ude_ros_server_config_target.timeout_wait == 40.0
        assert ude_server.ude_ros_server_config_target.port == UDE_COMM_DEFAULT_PORT
        assert ude_server.ude_ros_server_config_target.compression == grpc.Compression.NoCompression

        req = GetUDEServerConfigRequest()
        res = ude_server.on_config_request(req)
        assert res.reset_mode == UDEResetMode.MANUAL.value
        assert res.game_over_cond == 'any'
        assert res.step_invoke_type == UDEStepInvokeType.WAIT_FOREVER.value
        assert res.step_invoke_period == 120.0
        assert res.num_agents == 1
        assert res.timeout_wait == 60.0
        assert res.port == UDE_COMM_DEFAULT_PORT
        assert res.compression == const.CompressionMode.NO_COMPRESSION

    def test_on_config_request_after_reset(self, ude_server_mock, ude_env_mock, ros_env_adapter_mock, rospy_service_mock):
        ude_server = UDEROSServer()

        req = SetUDEServerConfigRequest()
        req.reset_mode = UDEResetMode.AUTO
        req.game_over_cond = "all"
        req.step_invoke_type = UDEStepInvokeType.PERIODIC.value
        req.step_invoke_period = 3.0
        req.num_agents = 4
        req.timeout_wait = 40.0

        ude_server.on_config_received(req)

        # Target config should get updated
        assert ude_server.ude_ros_server_config_target.reset_mode == UDEResetMode.AUTO
        assert ude_server.ude_ros_server_config_target.game_over_cond == all
        assert ude_server.ude_ros_server_config_target.step_invoke_type == UDEStepInvokeType.PERIODIC
        assert ude_server.ude_ros_server_config_target.step_invoke_period == 3.0
        assert ude_server.ude_ros_server_config_target.num_agents == 4
        assert ude_server.ude_ros_server_config_target.timeout_wait == 40.0
        assert ude_server.ude_ros_server_config_target.port == UDE_COMM_DEFAULT_PORT
        assert ude_server.ude_ros_server_config_target.compression == grpc.Compression.NoCompression

        req = GetUDEServerConfigRequest()
        res = ude_server.on_config_request(req)
        # Get should still return old config prior to reset call.
        assert res.reset_mode == UDEResetMode.MANUAL.value
        assert res.game_over_cond == 'any'
        assert res.step_invoke_type == UDEStepInvokeType.WAIT_FOREVER.value
        assert res.step_invoke_period == 120.0
        assert res.num_agents == 1
        assert res.timeout_wait == 60.0
        assert res.port == UDE_COMM_DEFAULT_PORT
        assert res.compression == const.CompressionMode.NO_COMPRESSION

        ude_server.on_reset(MagicMock(), MagicMock())

        req = GetUDEServerConfigRequest()
        res = ude_server.on_config_request(req)
        # The config should get updated as target config.
        assert res.reset_mode == UDEResetMode.AUTO.value
        assert res.game_over_cond == 'all'
        assert res.step_invoke_type == UDEStepInvokeType.PERIODIC.value
        assert res.step_invoke_period == 3.0
        assert res.num_agents == 4
        assert res.timeout_wait == 40.0
        assert res.port == UDE_COMM_DEFAULT_PORT
        assert res.compression == const.CompressionMode.NO_COMPRESSION
