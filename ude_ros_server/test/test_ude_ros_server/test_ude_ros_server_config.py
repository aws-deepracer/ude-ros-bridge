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

from ude_ros_server.ude_ros_server_config import UDEROSServerConfig
from ude import UDEStepInvokeType, UDE_COMM_DEFAULT_PORT, UDEResetMode

import grpc


class UDEROSServerConfigTest(TestCase):
    def test_initialize(self):
        # Test default values
        config = UDEROSServerConfig()
        assert config.reset_mode == UDEResetMode.MANUAL
        assert config.game_over_cond == any
        assert config.step_invoke_type == UDEStepInvokeType.WAIT_FOREVER
        assert config.step_invoke_period == 120.0
        assert config.num_agents == 1
        assert config.port == UDE_COMM_DEFAULT_PORT
        assert config.compression == grpc.Compression.NoCompression
        assert config.timeout_wait == 60.0

        # Initialize with values other than default.
        config = UDEROSServerConfig(reset_mode=UDEResetMode.AUTO,
                                    game_over_cond=all,
                                    step_invoke_type=UDEStepInvokeType.PERIODIC,
                                    step_invoke_period=3.0,
                                    num_agents=3,
                                    port=3333,
                                    compression=grpc.Compression.Gzip,
                                    timeout_wait=40.0)

        assert config.reset_mode == UDEResetMode.AUTO
        assert config.game_over_cond == all
        assert config.step_invoke_type == UDEStepInvokeType.PERIODIC
        assert config.step_invoke_period == 3.0
        assert config.num_agents == 3
        assert config.port == 3333
        assert config.compression == grpc.Compression.Gzip
        assert config.timeout_wait == 40.0

        # Initialize with string values.
        config = UDEROSServerConfig(reset_mode=UDEResetMode.AUTO.value,
                                    game_over_cond='all',
                                    step_invoke_type=UDEStepInvokeType.PERIODIC.value,
                                    step_invoke_period=3.0,
                                    num_agents=3,
                                    port=3333,
                                    compression="gzip",
                                    timeout_wait=40.0)

        assert config.reset_mode == UDEResetMode.AUTO
        assert config.game_over_cond == all
        assert config.step_invoke_type == UDEStepInvokeType.PERIODIC
        assert config.step_invoke_period == 3.0
        assert config.num_agents == 3
        assert config.port == 3333
        assert config.compression == grpc.Compression.Gzip
        assert config.timeout_wait == 40.0

    def test_reset_mode(self):
        config = UDEROSServerConfig()
        assert config.reset_mode == UDEResetMode.MANUAL

        config.reset_mode = UDEResetMode.AUTO
        assert config.reset_mode == UDEResetMode.AUTO

        config.reset_mode = UDEResetMode.MANUAL.value
        assert config.reset_mode == UDEResetMode.MANUAL

        config.reset_mode = 10
        assert config.reset_mode == UDEResetMode.MANUAL

        config.reset_mode = None
        assert config.reset_mode == UDEResetMode.MANUAL

    def test_game_over_cond(self):
        config = UDEROSServerConfig()
        assert config.game_over_cond == any

        config.game_over_cond = all
        assert config.game_over_cond == all

        config.game_over_cond = 'any'
        assert config.game_over_cond == any

        config.game_over_cond = None
        assert config.game_over_cond == any

    def test_step_invoke_type(self):
        config = UDEROSServerConfig()
        assert config.step_invoke_type == UDEStepInvokeType.WAIT_FOREVER

        config.step_invoke_type = UDEStepInvokeType.PERIODIC
        assert config.step_invoke_type == UDEStepInvokeType.PERIODIC

        config.step_invoke_type = UDEStepInvokeType.WAIT_FOREVER.value
        assert config.step_invoke_type == UDEStepInvokeType.WAIT_FOREVER

        config.step_invoke_type = 10
        assert config.step_invoke_type == UDEStepInvokeType.WAIT_FOREVER

        config.step_invoke_type = None
        assert config.step_invoke_type == UDEStepInvokeType.WAIT_FOREVER

    def test_step_invoke_period(self):
        config = UDEROSServerConfig()
        assert config.step_invoke_period == 120.0

        config.step_invoke_period = 3.0
        assert config.step_invoke_period == 3.0

        config.step_invoke_period = -3.0
        assert config.step_invoke_period == 3.0

    def test_num_agents(self):
        config = UDEROSServerConfig()
        assert config.num_agents == 1

        config.num_agents = 4
        assert config.num_agents == 4

        config.num_agents = -3
        assert config.num_agents == 4

    def test_timeout_wait(self):
        config = UDEROSServerConfig()
        assert config.timeout_wait == 60.0

        config.timeout_wait = 40.0
        assert config.timeout_wait == 40.0

        config.timeout_wait = -30.0
        assert config.timeout_wait == 40.0

    def test_copy(self):
        config = UDEROSServerConfig()
        copied_config = config.copy()
        assert config == copied_config
