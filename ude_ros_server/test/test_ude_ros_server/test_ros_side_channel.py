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
from unittest.mock import patch, MagicMock, ANY

from ude_ros_server.ros_side_channel import ROSSideChannel
import ude_ros_server.constants as const

from ude_ros_msgs.srv import (
    UDESideChannelSrv,
    UDESideChannelSrvRequest, UDESideChannelSrvResponse
)

import numpy as np


@patch("ude_ros_server.ros_side_channel.ServiceProxyWrapper")
@patch("ude_ros_server.ros_side_channel.rospy")
class ROSSideChannelTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_initialize(self, rospy_mock, service_proxy_wrapper_mock):
        ros_side_channel = ROSSideChannel()
        rospy_mock.Service.assert_called_once_with(const.SideChannelServiceType.TO_UDE.value,
                                                   UDESideChannelSrv,
                                                   ros_side_channel.on_message_received)

        service_proxy_wrapper_mock.assert_called_once_with(const.SideChannelServiceType.TO_ENV.value,
                                                           UDESideChannelSrv)

    def test_on_message_received_bool(self, rospy_mock, service_proxy_wrapper_mock):
        req = UDESideChannelSrvRequest()
        req.key = "key"
        req.data_type = const.SideChannelDataType.BOOLEAN
        req.data.bool_val = False

        ros_side_channel = ROSSideChannel()
        ros_side_channel.on_message_received(request=req)
        assert not ros_side_channel.get("key")

    def test_on_message_received_int(self, rospy_mock, service_proxy_wrapper_mock):
        req = UDESideChannelSrvRequest()
        req.key = "key"
        req.data_type = const.SideChannelDataType.INT
        req.data.int_val = 42

        ros_side_channel = ROSSideChannel()
        ros_side_channel.on_message_received(request=req)
        assert ros_side_channel.get("key") == 42

    def test_on_message_received_float(self, rospy_mock, service_proxy_wrapper_mock):
        req = UDESideChannelSrvRequest()
        req.key = "key"
        req.data_type = const.SideChannelDataType.FLOAT
        req.data.float_val = 42.42

        ros_side_channel = ROSSideChannel()
        ros_side_channel.on_message_received(request=req)
        assert ros_side_channel.get("key") == 42.42

    def test_on_message_received_float_list(self, rospy_mock, service_proxy_wrapper_mock):
        req = UDESideChannelSrvRequest()
        req.key = "key"
        req.data_type = const.SideChannelDataType.FLOAT_LIST
        req.data.float_list_val = [42.42, 43.43]

        ros_side_channel = ROSSideChannel()
        ros_side_channel.on_message_received(request=req)
        assert ros_side_channel.get("key") == [42.42, 43.43]

    def test_on_message_received_string(self, rospy_mock, service_proxy_wrapper_mock):
        req = UDESideChannelSrvRequest()
        req.key = "key"
        req.data_type = const.SideChannelDataType.STRING
        req.data.string_val = "The answer is 42."

        ros_side_channel = ROSSideChannel()
        ros_side_channel.on_message_received(request=req)
        assert ros_side_channel.get("key") == "The answer is 42."

    def test_on_message_received_bytes(self, rospy_mock, service_proxy_wrapper_mock):
        req = UDESideChannelSrvRequest()
        req.key = "key"
        req.data_type = const.SideChannelDataType.BYTES
        req.data.bytes_val = "The answer is 42.".encode('utf-8')

        ros_side_channel = ROSSideChannel()
        ros_side_channel.on_message_received(request=req)
        assert ros_side_channel.get("key") == "The answer is 42.".encode('utf-8')

    def test_on_message_received_unknown_type(self, rospy_mock, service_proxy_wrapper_mock):
        req = UDESideChannelSrvRequest()
        req.key = "key"
        req.data_type = 10
        req.data.string_val = "The answer is 42."

        ros_side_channel = ROSSideChannel()
        with self.assertRaises(TypeError):
            ros_side_channel.on_message_received(request=req)

    def test_on_message_received_notify(self, rospy_mock, service_proxy_wrapper_mock):
        observer = MagicMock()
        req = UDESideChannelSrvRequest()
        req.key = "key"
        req.data_type = const.SideChannelDataType.STRING
        req.data.string_val = "The answer is 42."

        ros_side_channel = ROSSideChannel()
        ros_side_channel.register(observer=observer)
        ros_side_channel.on_message_received(request=req)
        assert ros_side_channel.get("key") == "The answer is 42."
        observer.on_received.assert_called_once_with(side_channel=ros_side_channel,
                                                     key="key",
                                                     value="The answer is 42.")

    def test_send_bool(self, rospy_mock, service_proxy_wrapper_mock):
        key = "key"
        value = False

        expected_req = UDESideChannelSrvRequest()
        expected_req.key = key
        expected_req.data_type = const.SideChannelDataType.BOOLEAN
        expected_req.data.bool_val = value

        ros_side_channel = ROSSideChannel()
        ros_side_channel.send(key=key, value=value)
        service_proxy_wrapper_mock.return_value.assert_called_once_with(expected_req)

    def test_send_int(self, rospy_mock, service_proxy_wrapper_mock):
        key = "key"
        value = 42

        expected_req = UDESideChannelSrvRequest()
        expected_req.key = key
        expected_req.data_type = const.SideChannelDataType.INT
        expected_req.data.int_val = value

        ros_side_channel = ROSSideChannel()
        ros_side_channel.send(key=key, value=value)
        service_proxy_wrapper_mock.return_value.assert_called_once_with(expected_req)

    def test_send_float(self, rospy_mock, service_proxy_wrapper_mock):
        key = "key"
        value = 42.42

        expected_req = UDESideChannelSrvRequest()
        expected_req.key = key
        expected_req.data_type = const.SideChannelDataType.FLOAT
        expected_req.data.float_val = value

        ros_side_channel = ROSSideChannel()
        ros_side_channel.send(key=key, value=value)
        service_proxy_wrapper_mock.return_value.assert_called_once_with(expected_req)

    def test_send_float_list(self, rospy_mock, service_proxy_wrapper_mock):
        key = "key"
        value = [42.42, 43.43]

        expected_req = UDESideChannelSrvRequest()
        expected_req.key = key
        expected_req.data_type = const.SideChannelDataType.FLOAT_LIST
        expected_req.data.float_list_val = value

        ros_side_channel = ROSSideChannel()
        ros_side_channel.send(key=key, value=value)
        service_proxy_wrapper_mock.return_value.assert_called_once_with(expected_req)

    def test_send_string(self, rospy_mock, service_proxy_wrapper_mock):
        key = "key"
        value = "the answer is 42."

        expected_req = UDESideChannelSrvRequest()
        expected_req.key = key
        expected_req.data_type = const.SideChannelDataType.STRING
        expected_req.data.string_val = value

        ros_side_channel = ROSSideChannel()
        ros_side_channel.send(key=key, value=value)
        service_proxy_wrapper_mock.return_value.assert_called_once_with(expected_req)

    def test_send_bytes(self, rospy_mock, service_proxy_wrapper_mock):
        key = "key"
        value = "the answer is 42.".encode('utf-8')

        expected_req = UDESideChannelSrvRequest()
        expected_req.key = key
        expected_req.data_type = const.SideChannelDataType.BYTES
        expected_req.data.bytes_val = value

        ros_side_channel = ROSSideChannel()
        ros_side_channel.send(key=key, value=value)
        service_proxy_wrapper_mock.return_value.assert_called_once_with(expected_req)

    def test_send_np_bool(self, rospy_mock, service_proxy_wrapper_mock):
        key = "key"
        value = False

        expected_req = UDESideChannelSrvRequest()
        expected_req.key = key
        expected_req.data_type = const.SideChannelDataType.BOOLEAN
        expected_req.data.bool_val = value

        ros_side_channel = ROSSideChannel()
        ros_side_channel.send(key=key, value=np.bool_(value))
        service_proxy_wrapper_mock.return_value.assert_called_once_with(expected_req)

    def test_send_np_int(self, rospy_mock, service_proxy_wrapper_mock):
        key = "key"
        value = 42

        expected_req = UDESideChannelSrvRequest()
        expected_req.key = key
        expected_req.data_type = const.SideChannelDataType.INT
        expected_req.data.int_val = value

        ros_side_channel = ROSSideChannel()
        ros_side_channel.send(key=key, value=np.int32(value))
        service_proxy_wrapper_mock.return_value.assert_called_once_with(expected_req)

    def test_send_np_float(self, rospy_mock, service_proxy_wrapper_mock):
        key = "key"
        value = 42.42

        expected_req = UDESideChannelSrvRequest()
        expected_req.key = key
        expected_req.data_type = const.SideChannelDataType.FLOAT
        expected_req.data.float_val = np.float32(value).item()

        ros_side_channel = ROSSideChannel()
        ros_side_channel.send(key=key, value=np.float32(value))
        service_proxy_wrapper_mock.return_value.assert_called_once_with(expected_req)

    def test_send_unknown(self, rospy_mock, service_proxy_wrapper_mock):
        ros_side_channel = ROSSideChannel()
        with self.assertRaises(TypeError):
            ros_side_channel.send(key="key", value=MagicMock())
