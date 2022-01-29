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
"""Module to contain UDE ROS Server related constants"""
from enum import Enum, IntEnum
from ude import SideChannelDataType
from grpc import Compression


class UDEServiceType(Enum):
    """
    UDE ROS Service Type
    """
    STEP = 'ude_step'
    RESET = 'ude_reset'
    CLOSE = 'ude_close'
    OBSERVATION_SPACE = 'ude_observation_space'
    ACTION_SPACE = 'ude_action_space'


class DataTypeAttrType(Enum):
    """
    Data Type's Attribute name
    """
    BOOLEAN = 'bool_val'
    INT = 'int_val'
    FLOAT = 'float_val'
    FLOAT_LIST = 'float_list_val'
    STRING = 'string_val'
    BYTES = 'bytes_val'


"""
Date type to attribute map
"""
SIDE_CHANNEL_DATATYPE_TO_ROS_MSG_ATTR_MAP = {SideChannelDataType.BOOLEAN.value: DataTypeAttrType.BOOLEAN.value,
                                             SideChannelDataType.INT.value: DataTypeAttrType.INT.value,
                                             SideChannelDataType.FLOAT.value: DataTypeAttrType.FLOAT.value,
                                             SideChannelDataType.FLOAT_LIST.value: DataTypeAttrType.FLOAT_LIST.value,
                                             SideChannelDataType.STRING.value: DataTypeAttrType.STRING.value,
                                             SideChannelDataType.BYTES.value: DataTypeAttrType.BYTES.value}


"""
builtin type to attribute map
"""
BUILTIN_TYPE_TO_ROS_MSG_ATTR_MAP = {bool: DataTypeAttrType.BOOLEAN.value,
                                    int: DataTypeAttrType.INT.value,
                                    float: DataTypeAttrType.FLOAT.value,
                                    list: DataTypeAttrType.FLOAT_LIST.value,
                                    str: DataTypeAttrType.STRING.value,
                                    bytes: DataTypeAttrType.BYTES.value}


class SideChannelServiceType(Enum):
    """
    Side Channel service types
    """
    TO_ENV = 'ude_side_channel_to_env'
    TO_UDE = 'ude_side_channel_to_ude'


class UDEROSServerServiceType(Enum):
    """
    UDE ROS Server Service types
    """
    SET_UDE_SERVER_CONFIG = 'set_ude_server_config'
    GET_UDE_SERVER_CONFIG = 'get_ude_server_config'


class GameOverConditionType(Enum):
    """
    Game Over Condition Type
    """
    ANY = 0
    ALL = 1


# UDE Server Compression mode
class CompressionMode(object):
    NO_COMPRESSION = "no_compression"
    DEFLATE = "deflate"
    GZIP = "gzip"


# UDE Server Compression mode string to GRPC Compression type map.
GRPC_COMPRESSION_MAP = {
    CompressionMode.NO_COMPRESSION: Compression.NoCompression,
    CompressionMode.DEFLATE: Compression.Deflate,
    CompressionMode.GZIP: Compression.Gzip
}

# GRPC Compression type to UDE Server Compression mode map.
UDE_COMPRESSION_MODE_MAP = {
    Compression.NoCompression: CompressionMode.NO_COMPRESSION,
    Compression.Deflate: CompressionMode.DEFLATE,
    Compression.Gzip: CompressionMode.GZIP
}
