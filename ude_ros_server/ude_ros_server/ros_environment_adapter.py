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
"""A class for ROS Environment Adapter."""
from typing import cast, Dict

from gym import Space

from ude import (
    UDEEnvironmentAdapterInterface,
    MultiAgentDict, UDEStepResult,
    AbstractSideChannel, AgentID,
    UDESerializationContext
)

from ude_ros_server.ros_side_channel import ROSSideChannel
from ude_ros_server.service_proxy_wrapper import ServiceProxyWrapper
import ude_ros_server.constants as const

from threading import RLock

from ude_ros_msgs.srv import (
    UDEStepSrv, UDEStepSrvRequest,
    UDEResetSrv, UDEResetSrvRequest,
    UDECloseSrv, UDECloseSrvRequest,
    UDEObservationSpaceSrv, UDEObservationSpaceSrvRequest,
    UDEActionSpaceSrv, UDEActionSpaceSrvRequest
)
import rospy


class ROSEnvironmentObserverInterface:
    def on_step(self,
                adapter: UDEEnvironmentAdapterInterface,
                action_dict: MultiAgentDict,
                step_result: UDEStepResult):
        """
        Callback on step

        Args:
            adapter (UDEEnvironmentAdapterInterface): environment adapter.
            action_dict (MultiAgentDict): agent action data.
            step_result (UDEStepResult): step result.
        """
        pass

    def on_reset(self,
                 adapter: UDEEnvironmentAdapterInterface,
                 observation_data: MultiAgentDict):
        """
        Callback on reset

        Args:
            adapter (UDEEnvironmentAdapterInterface): environment adapter.
            observation_data (MultiAgentDict): first observation data
        """
        pass

    def on_close(self, adapter: UDEEnvironmentAdapterInterface):
        """
        Callback on close

        Args:
            adapter (UDEEnvironmentAdapterInterface): environment adapter.
        """
        pass


class ROSEnvironmentAdapter(UDEEnvironmentAdapterInterface):
    """
    ROSEnvironmentAdapter class to interface ROS environment to UDE Environment.
    """
    def __init__(self,
                 timeout_sec: float = None):
        """
        Initialize ROSEnvironmentAdapter
        """
        super().__init__()
        rospy.loginfo("[ROSEnvironmentAdapter] initiating...")
        self._context = UDESerializationContext.get_context()
        self._timeout_sec = timeout_sec
        self._observers = set()
        self._observer_lock = RLock()
        self._side_channel = ROSSideChannel()

        rospy.loginfo("[ROSEnvironmentAdapter] wait_for_services...")
        self._step_cli = ServiceProxyWrapper(const.UDEServiceType.STEP.value,
                                             UDEStepSrv)
        self._reset_cli = ServiceProxyWrapper(const.UDEServiceType.RESET.value,
                                              UDEResetSrv)
        self._close_cli = ServiceProxyWrapper(const.UDEServiceType.CLOSE.value,
                                              UDECloseSrv)
        self._observation_space_cli = ServiceProxyWrapper(const.UDEServiceType.OBSERVATION_SPACE.value,
                                                          UDEObservationSpaceSrv)
        self._action_space_cli = ServiceProxyWrapper(const.UDEServiceType.ACTION_SPACE.value,
                                                     UDEActionSpaceSrv)
        rospy.loginfo("[ROSEnvironmentAdapter] initializing [DONE]")

    def register(self, observer: ROSEnvironmentObserverInterface) -> None:
        """
        Register the given observer.

        Args:
            observer (ROSEnvironmentObserverInterface): the observer
        """
        with self._observer_lock:
            self._observers.add(observer)

    def unregister(self, observer: ROSEnvironmentObserverInterface):
        """
        Remove the given observer.

        Args:
            observer (ROSEnvironmentObserverInterface): the observer to remove

        Returns:

        """
        with self._observer_lock:
            self._observers.discard(observer)

    def step(self, action_dict: MultiAgentDict) -> UDEStepResult:
        """
        Performs one multi-agent step with given action, and retrieve
        observation(s), reward(s), done(s), action(s) taken,
        and info (if there is any).

        Args:
            action_dict (MultiAgentDict): the action(s) for the agent(s) with agent_name as key.

        Returns:
            UDEStepResult: observations, rewards, dones, last_actions, info
        """
        rospy.logdebug("[ROSEnvironmentAdapter] step")
        serialized_obj = bytes(self._context.serialize(action_dict).to_buffer())
        res = self._step_cli(UDEStepSrvRequest(data=serialized_obj))
        step_data = self._context.deserialize(bytes(res.data))
        with self._observer_lock:
            observers = self._observers.copy()
        for observer in observers:
            observer.on_step(adapter=self,
                             action_dict=action_dict,
                             step_result=step_data)
        rospy.logdebug("[ROSEnvironmentAdapter] step [DONE]")
        return cast(UDEStepResult, step_data)

    def reset(self) -> MultiAgentDict:
        """
        Reset the environment and start new episode.
        Also, returns the first observation for new episode started.

        Returns:
            MultiAgentDict: first observation in new episode.
        """
        rospy.logdebug("[ROSEnvironmentAdapter] reset")
        res = self._reset_cli(UDEResetSrvRequest())
        obs = self._context.deserialize(bytes(res.data))
        with self._observer_lock:
            observers = self._observers.copy()
        for observer in observers:
            observer.on_reset(adapter=self,
                              observation_data=obs)
        rospy.logdebug("[ROSEnvironmentAdapter] reset [DONE]")
        return cast(MultiAgentDict, obs)

    def close(self) -> None:
        """
        Close the environment, and environment will be no longer available to be used.
        """
        rospy.logdebug("[ROSEnvironmentAdapter] close")
        _ = self._close_cli(UDECloseSrvRequest())
        with self._observer_lock:
            observers = self._observers.copy()
        for observer in observers:
            observer.on_close(adapter=self)
        rospy.logdebug("[ROSEnvironmentAdapter] close [DONE]")

    @property
    def observation_space(self) -> Dict[AgentID, Space]:
        """
        Returns the observation spaces of agents in env.

        Returns:
            Dict[AgentID, Space]: the observation spaces of agents in env.
        """
        rospy.logdebug("[ROSEnvironmentAdapter] observation_space")
        res = self._observation_space_cli(UDEObservationSpaceSrvRequest())
        observation_space = self._context.deserialize(bytes(res.data))
        rospy.logdebug("[ROSEnvironmentAdapter] observation_space [DONE]")
        return cast(Dict[AgentID, Space], observation_space)

    @property
    def action_space(self) -> Dict[AgentID, Space]:
        """
        Returns the action spaces of agents in env.

        Returns:
            Dict[AgentID, Space]: the action spaces of agents in env.
        """
        rospy.logdebug("[ROSEnvironmentAdapter] action_space")
        res = self._action_space_cli(UDEActionSpaceSrvRequest())
        action_space = self._context.deserialize(bytes(res.data))
        rospy.logdebug("[ROSEnvironmentAdapter] action_space [DONE]")
        return cast(Dict[AgentID, Space], action_space)

    @property
    def side_channel(self) -> AbstractSideChannel:
        """
        Returns side channel to send and receive data from UDE Server

        Returns:
            AbstractSideChannel: the instance of side channel.
        """
        return self._side_channel
