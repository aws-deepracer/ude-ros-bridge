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
"""A class for UDE ROS Server Configuration."""
from typing import Union, Optional, Any
from ude import UDEStepInvokeType, UDE_COMM_DEFAULT_PORT, UDEResetMode
from ude_ros_server.constants import GRPC_COMPRESSION_MAP

import math
import rospy
from grpc import Compression


class UDEROSServerConfig(object):
    def __init__(self,
                 reset_mode: Optional[Union[str, UDEResetMode]] = None,
                 game_over_cond: Optional[Union[str, Any]] = None,
                 step_invoke_type: Optional[Union[str, UDEStepInvokeType]] = None,
                 step_invoke_period: float = 120.0,
                 num_agents: int = 1,
                 port: int = UDE_COMM_DEFAULT_PORT,
                 compression: Optional[Union[str, Compression]] = None,
                 timeout_wait: float = 60.0) -> None:
        """
        Initialize UDE ROS Server Config

        Args:
            reset_mode (Optional[Union[str, UDEResetMode]]): reset mode
            game_over_cond (Optional[Union[str, Any]]): game over condition function
            step_invoke_type (Optional[Union[str, UDEStepInvokeType]]): step invoke type (WAIT_FOREVER | PERIODIC)
            step_invoke_period (float): step invoke period in second
            num_agents (int): number of agents
            port (int): UDE Server port
            compression (Optional[Union[str, Compression]]): grpc.Compression
            timeout_wait (float): timeout in seconds
        """
        self._reset_mode = reset_mode or UDEResetMode.MANUAL
        if isinstance(self._reset_mode, str):
            try:
                self._reset_mode = UDEResetMode(self._reset_mode.lower())
            except Exception:
                self._reset_mode = UDEResetMode.MANUAL

        self._game_over_cond = game_over_cond or any
        if isinstance(self._game_over_cond, str):
            self._game_over_cond = all if self._game_over_cond.lower() == 'all' else any

        self._step_invoke_type = step_invoke_type or UDEStepInvokeType.WAIT_FOREVER
        if isinstance(self._step_invoke_type, str):
            try:
                self._step_invoke_type = UDEStepInvokeType(self._step_invoke_type.lower())
            except Exception:
                self._step_invoke_type = UDEStepInvokeType.WAIT_FOREVER

        self._step_invoke_period = step_invoke_period if step_invoke_period > 0.0 else 120.0
        self._num_agents = num_agents if num_agents > 0 else 1
        self._port = port if port > 0 else UDE_COMM_DEFAULT_PORT

        self._compression = compression or Compression.NoCompression
        if isinstance(self._compression, str):
            self._compression = GRPC_COMPRESSION_MAP[self._compression]

        self._timeout_wait = timeout_wait if timeout_wait > 0.0 else 60.0

    @property
    def reset_mode(self) -> UDEResetMode:
        """
        Returns reset Mode

        Returns:
            UDEResetMode: reset mode
        """
        return self._reset_mode

    @reset_mode.setter
    def reset_mode(self, value: Optional[Union[str, UDEResetMode]]) -> None:
        """
        Set new reset mode (Only update with valid argument)

        Args:
            value (Optional[Union[str, UDEResetMode]]): new reset mode
        """
        if value:
            if isinstance(value, str):
                try:
                    self._reset_mode = UDEResetMode(value)
                except Exception:
                    rospy.logerr("[UDEROSServerConfig] Unknown UDEResetMode provided for reset_mode: {}".format(value))
                    return
            elif isinstance(value, UDEResetMode):
                self._reset_mode = value
            else:
                rospy.logerr("[UDEROSServerConfig] Unknown reset_mode type: {}".format(type(value)))

    @property
    def game_over_cond(self) -> Any:
        """
        Returns game over condition.

        Returns:
            Any: the game over condition function.
        """
        return self._game_over_cond

    @game_over_cond.setter
    def game_over_cond(self, value: Optional[Union[str, Any]]) -> None:
        """
        Sets new game over condition function.

        Args:
            value (Optional[Union[str, Any]]): new game over condition function
        """
        if value:
            game_over_cond = value
            if isinstance(game_over_cond, str):
                game_over_cond = all if game_over_cond.lower() == 'all' else any
            self._game_over_cond = game_over_cond

    @property
    def step_invoke_type(self) -> UDEStepInvokeType:
        """
        Returns step invoke type

        Returns:
            UDEStepInvokeType: step invoke type (WAIT_FOREVER | PERIODIC)
        """
        return self._step_invoke_type

    @step_invoke_type.setter
    def step_invoke_type(self, value: Optional[Union[str, UDEStepInvokeType]]) -> None:
        """
        Sets new step invoke type.

        Args:
            value (Optional[Union[str, UDEStepInvokeType]]): new step invoke type
        """
        if value:
            if isinstance(value, str):
                try:
                    self._step_invoke_type = UDEStepInvokeType(value)
                except Exception:
                    err_msg = "[UDEROSServerConfig] Unknown UDEStepInvokeType provided for step_invoke_type: {}"
                    rospy.logerr(err_msg.format(value))
                    return
            elif isinstance(value, UDEStepInvokeType):
                self._step_invoke_type = value
            else:
                rospy.logerr("[UDEROSServerConfig] Unknown step_invoke_type type: {}".format(type(value)))

    @property
    def step_invoke_period(self) -> float:
        """
        Returns step invoke period.

        Returns:
            float: step invoke period in seconds.
        """
        return self._step_invoke_period

    @step_invoke_period.setter
    def step_invoke_period(self, value: float) -> None:
        """
        Sets new step invoke period in seconds.

        Args:
            value (float): new step invoke period in seconds.
        """
        if value > 0.0:
            self._step_invoke_period = value

    @property
    def num_agents(self) -> int:
        """
        Returns the number of agents.

        Returns:
            int: the number of agents.
        """
        return self._num_agents

    @num_agents.setter
    def num_agents(self, value: int) -> None:
        """
        Sets new number of agents.

        Args:
            value (int): new number of agents.
        """
        if value > 0:
            self._num_agents = value

    @property
    def port(self) -> int:
        """
        Returns UDE Server port.

        Returns:
            int: UDE Server port.
        """
        return self._port

    @property
    def compression(self) -> Compression:
        """
        Return UDE Server compression mode.

        Returns:
            Compression: UDE Server compression mode.
        """
        return self._compression

    @property
    def timeout_wait(self) -> float:
        """
        Returns the timeout in seconds.

        Returns:
            float: timeout in seconds.
        """
        return self._timeout_wait

    @timeout_wait.setter
    def timeout_wait(self, value: float) -> None:
        """
        Sets new timeout in seconds.

        Args:
            value (float): new timeout in seconds.
        """
        if value > 0.0:
            self._timeout_wait = value

    def copy(self) -> 'UDEROSServerConfig':
        """
        Copy and return this UDE ROS Server config object.

        Returns:
            UDEROSServerConfig: copied UDE ROS Server config object.
        """
        return UDEROSServerConfig(reset_mode=self.reset_mode,
                                  game_over_cond=self.game_over_cond,
                                  step_invoke_type=self.step_invoke_type,
                                  step_invoke_period=self.step_invoke_period,
                                  num_agents=self.num_agents,
                                  port=self.port,
                                  compression=self.compression,
                                  timeout_wait=self.timeout_wait)

    def __eq__(self, other: 'UDEROSServerConfig') -> bool:
        """
        Equality of config is equality of all config values to within epsilon.

        Args:
            other (UDEROSServerConfig): other to compare

        Returns:
            bool: True if the differences of all config values are within epsilon, Otherwise False.
        """
        return (self.reset_mode == other.reset_mode
                and self.game_over_cond == other.game_over_cond
                and self.step_invoke_type == other.step_invoke_type
                and math.isclose(self.step_invoke_period, other.step_invoke_period)
                and self.num_agents == other.num_agents
                and self.port == other.port
                and self.compression == other.compression
                and math.isclose(self.timeout_wait, other.timeout_wait))

    def __ne__(self, other: 'UDEROSServerConfig') -> bool:
        """
        Inequality of points is inequality of any config values

        Args:
            other (UDEROSServerConfig): other to compare

        Returns:
            bool: True if the differences of any config values are larger than epsilon, Otherwise False.
        """
        return not self.__eq__(other)
