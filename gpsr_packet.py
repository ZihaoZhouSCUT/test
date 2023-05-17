from src.entities.Packet import Packet
from src.entities.UAV import Drones
import numpy as np
import torch

"""
1. entities里面的Packet类包含了所有不同协议里面packet的基本特征,而不同protocol所需要的定制包都可以继承自Packet类
2. data packet, hello packet和 ACK packet通过三个global变量分别对其ID进行控制
"""

class GpsrHelloPacket(Packet):
    # GPSR的HELLO Packet只需要包含发起节点自身的位置信息

    def __init__(self, src_drone: Drones, time_step_creation, id_hello_packet, simulator):
        super().__init__(id_hello_packet, time_step_creation, simulator)

        self.src_drone = src_drone
        self.cur_position = src_drone.coords


