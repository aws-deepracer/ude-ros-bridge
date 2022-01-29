#!/usr/bin/env python3

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
"""A class for UDE ROS Server."""

import rospy
import grpc

from ude import (
    UDEEnvironment, UDEServer,
    UDEStepInvokeType,
    UDEResetMode,
    UDE_COMM_DEFAULT_PORT,
    UDEEnvironmentAdapterInterface,
    MultiAgentDict
)

from ude_ros_server.ros_environment_adapter import ROSEnvironmentAdapter, ROSEnvironmentObserverInterface
from ude_ros_server.ude_ros_server_config import UDEROSServerConfig
import ude_ros_server.constants as const

from ude_ros_msgs.srv import (
    SetUDEServerConfig,
    SetUDEServerConfigRequest, SetUDEServerConfigResponse,
    GetUDEServerConfig,
    GetUDEServerConfigRequest, GetUDEServerConfigResponse,
)


class UDEROSServer(ROSEnvironmentObserverInterface):
    def __init__(self) -> None:
        """
        Initialize UDE ROS Server
        """
        reset_mode = UDEResetMode(rospy.get_param('~ude_ros_server/reset_mode', "manual"))

        game_over_cond = rospy.get_param('~ude_ros_server/game_over_cond', 'any')

        step_invoke_type = UDEStepInvokeType(rospy.get_param('~ude_ros_server/step_invoke_type', 'wait_forever'))

        step_invoke_period = float(rospy.get_param('~ude_ros_server/step_invoke_period', 120.0))
        num_agents = int(rospy.get_param('~ude_ros_server/num_agents', 1))
        port = int(rospy.get_param('~ude_ros_server/port', UDE_COMM_DEFAULT_PORT))
        compression = str(rospy.get_param('~ude_ros_server/compression', const.CompressionMode.NO_COMPRESSION)).lower()
        timeout_wait = float(rospy.get_param('~ude_ros_server/timeout_wait', 60.0))

        # Secure channel configuration parameters.
        private_key_file = str(rospy.get_param('~private_key_file', ''))
        certificate_file = str(rospy.get_param('~certificate_file', ''))
        self.ude_ros_server_config = UDEROSServerConfig(reset_mode=reset_mode,
                                                        game_over_cond=game_over_cond,
                                                        step_invoke_type=step_invoke_type,
                                                        step_invoke_period=step_invoke_period,
                                                        num_agents=num_agents,
                                                        port=port,
                                                        compression=compression,
                                                        timeout_wait=timeout_wait)
        self.ude_ros_server_config_target = self.ude_ros_server_config.copy()

        # Create UDE Config services
        self._set_ude_server_config_srv = rospy.Service(const.UDEROSServerServiceType.SET_UDE_SERVER_CONFIG.value,
                                                        SetUDEServerConfig,
                                                        self.on_config_received)
        self._get_ude_server_config_srv = rospy.Service(const.UDEROSServerServiceType.GET_UDE_SERVER_CONFIG.value,
                                                        GetUDEServerConfig,
                                                        self.on_config_request)
        # Start UDE Server
        ude_env_adapter = ROSEnvironmentAdapter()
        ude_env_adapter.register(self)

        ude_env = UDEEnvironment(ude_env_adapter=ude_env_adapter,
                                 reset_mode=self.ude_ros_server_config.reset_mode,
                                 game_over_cond=self.ude_ros_server_config.game_over_cond)

        server_credentials = None
        if private_key_file and certificate_file:
            try:
                with open(private_key_file, 'rb') as f:
                    private_key = f.read()
                with open(certificate_file, 'rb') as f:
                    certificate = f.read()

                # create server credentials
                server_credentials = grpc.ssl_server_credentials(
                    ((private_key, certificate, ), )
                )
            except Exception as ex:
                err_msg_format = "[UDEROSServer] Failed to read or create server credentials " \
                                 "with private key ({}) and cert ({}): {}"
                rospy.logerr(err_msg_format.format(private_key_file, certificate_file, ex))

        if server_credentials:
            rospy.loginfo("[UDEROSServer] Starting secured channel...")
        else:
            rospy.loginfo("[UDEROSServer] Starting unsecured channel...")

        self.print_config()

        self.ude_server = UDEServer(ude_env=ude_env,
                                    step_invoke_type=self.ude_ros_server_config.step_invoke_type,
                                    step_invoke_period=self.ude_ros_server_config.step_invoke_period,
                                    num_agents=self.ude_ros_server_config.num_agents,
                                    port=self.ude_ros_server_config.port,
                                    compression=self.ude_ros_server_config.compression,
                                    credentials=server_credentials,
                                    timeout_wait=self.ude_ros_server_config.timeout_wait)
        self.ude_server.start()

    def print_config(self) -> None:
        """
        Print UDE Server Configuration.
        """
        ude_ros_server_config = self.ude_ros_server_config.copy()
        rospy.logdebug("[UDEROSServer] reset_mode: {}".format(ude_ros_server_config.reset_mode.value))
        rospy.logdebug("[UDEROSServer] game_over_cond: {}".format("any" if ude_ros_server_config.game_over_cond == any
                                                                  else "all"))
        rospy.logdebug("[UDEROSServer] step_invoke_type: {}".format(ude_ros_server_config.step_invoke_type.value))
        rospy.logdebug("[UDEROSServer] step_invoke_period: {}".format(ude_ros_server_config.step_invoke_period))
        rospy.logdebug("[UDEROSServer] num_agents: {}".format(ude_ros_server_config.num_agents))
        rospy.logdebug("[UDEROSServer] timeout_wait: {}".format(ude_ros_server_config.timeout_wait))
        rospy.logdebug("[UDEROSServer] port: {}".format(ude_ros_server_config.port))
        compression_mode = const.UDE_COMPRESSION_MODE_MAP[ude_ros_server_config.compression]
        rospy.logdebug("[UDEROSServer] compression: {}".format(compression_mode))

    def on_config_received(self, request: SetUDEServerConfigRequest) -> SetUDEServerConfigResponse:
        """
        Callback handler for set_ude_server_config request.

        Args:
            request (SetUDEServerConfigRequest): request containing new config values

        Returns:
            SetUDEServerConfigResponse: response
        """
        self.ude_ros_server_config_target.reset_mode = request.reset_mode
        self.ude_ros_server_config_target.game_over_cond = request.game_over_cond
        self.ude_ros_server_config_target.step_invoke_type = request.step_invoke_type
        self.ude_ros_server_config_target.step_invoke_period = request.step_invoke_period
        self.ude_ros_server_config_target.num_agents = request.num_agents
        self.ude_ros_server_config_target.timeout_wait = request.timeout_wait

        res = SetUDEServerConfigResponse()
        return res

    def on_config_request(self, request: GetUDEServerConfigRequest) -> GetUDEServerConfigResponse:
        """
        Callback handler for get_ude_server_config request.

        Args:
            request (GetUDEServerConfigRequest): empty request

        Returns:
            GetUDEServerConfigResponse: response containing current UDE Server config values.
        """
        ude_ros_server_config = self.ude_ros_server_config.copy()
        res = GetUDEServerConfigResponse()

        res.reset_mode = ude_ros_server_config.reset_mode.value
        res.game_over_cond = "any" if ude_ros_server_config.game_over_cond == any else "all"
        res.step_invoke_type = ude_ros_server_config.step_invoke_type.value
        res.step_invoke_period = ude_ros_server_config.step_invoke_period
        res.num_agents = ude_ros_server_config.num_agents
        res.timeout_wait = ude_ros_server_config.timeout_wait
        res.port = ude_ros_server_config.port
        res.compression = const.UDE_COMPRESSION_MODE_MAP[ude_ros_server_config.compression]

        return res

    def on_reset(self,
                 adapter: UDEEnvironmentAdapterInterface,
                 observation_data: MultiAgentDict) -> None:
        """
        Callback handler to update the config during the environment reset.

        Args:
            adapter (UDEEnvironmentAdapterInterface): the environment adapter
            observation_data (MultiAgentDict): new observation data.
        """
        self.ude_ros_server_config = self.ude_ros_server_config_target.copy()

        self.ude_server.num_agent = self.ude_ros_server_config.num_agents
        self.ude_server.step_invoke_type = self.ude_ros_server_config.step_invoke_type
        self.ude_server.step_invoke_period = self.ude_ros_server_config.step_invoke_period
        self.ude_server.timeout_wait = self.ude_ros_server_config.timeout_wait

        self.ude_server.env.game_over_cond = self.ude_ros_server_config.game_over_cond
        self.ude_server.env.reset_mode = self.ude_ros_server_config.reset_mode


if __name__ == '__main__':
    rospy.init_node(name="ude_ros_server",
                    anonymous=False,
                    log_level=rospy.INFO)
    _ = UDEROSServer()
    rospy.spin()
