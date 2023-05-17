from src.routing_protocol.GPSR.gpsr_neighbortable import GpsrNeighborTable
from src.entities.Packet import DataPacket, ACKPacket
from src.routing_protocol.GPSR.gpsr_packet import GpsrHelloPacket
from src.entities.UAV import Drones
from utils import config
import numpy
import copy

gl_id_hello_packet = 0
gl_id_ack_packet = 0

class GPSR:
    def __init__(self, simulator, my_drone: Drones):
        self.simulator = simulator                    # 用于获得与仿真相关的任何信息
        self.neighbor_table = GpsrNeighborTable()     # 邻居表

    # 广播
    def broadcast_message(self, packet, src_drone, dst_drones, cur_step):
        for d_drone in dst_drones:
            self.unicast_message(packet, src_drone, d_drone, cur_step)

    # 单播
    def unicast_message(self, packet, src_drone, dst_drone, cur_step):
        self.simulator.network_dispatcher.notify_dispatcher(packet, src_drone, dst_drone, cur_step + config.LIL_DELTA)

    # 发送HELLO包
    def send_hello_packet(self, my_drone, drones, cur_step):
        # my_self: 是自身UAV的类实例
        # drones: 是一个列表,其中的每一个元素都是一个Drone类
        # cur_step: 当前时间步

        global gl_id_hello_packet
        if cur_step % config.HELLO_DELAY != 0:
            return

        gl_id_hello_packet += 1
        my_hello = GpsrHelloPacket(my_drone, cur_step, gl_id_hello_packet, self.simulator)
        self.broadcast_message(my_hello, my_drone, drones, cur_step)

        # 对应做能量的衰减
        energy_consumption = my_drone.energymodel.compute_transmit_consumption()
        my_drone.residual_energy -= energy_consumption

    # 转发packet
    def send_data_packet(self, my_drone: Drones, cur_step):
        # 在此处就应该要purge邻居表,否则会造成在执行best neighbor之前唯一的一个邻居因超时被清除导致没有邻居的报错
        self.neighbor_table.purge(cur_step)

        if (my_drone.get_buffer_length() == 0 and my_drone.get_sendbuffer_length() == 0) or self.neighbor_table.is_empty():
            return
        else:
            all_packets_to_relay = my_drone.get_all_packet()
            all_packets_to_send = my_drone.get_all_packets_in_sendbuffer()
            all_packets = all_packets_to_send + all_packets_to_relay

            to_drop_packet = []
            for pkd in all_packets:
                dst_drone = pkd.dst_drone         # 获取到每一个data packet的目的地

                if pkd.number_retransmission_attempt[my_drone.identifier] < config.MAX_RETRANSMISSION:       # 说明这个packet还可以传
                    pkd.increase_TTL()                                                                       # 该packet的跳数+1
                    pkd.number_retransmission_attempt[my_drone.identifier] += 1                              # 该packet的传输次数+1

                    if self.neighbor_table.is_neighbor(dst_drone):
                        self.unicast_message(pkd, my_drone, dst_drone, cur_step)                             # 将该packet传递至dst_drone
                    else:
                        best_next_hop_id = self.neighbor_table.best_neighbor(my_drone, dst_drone, cur_step)
                        best_drone = self.simulator.drones[best_next_hop_id]
                        self.unicast_message(pkd, my_drone, best_drone, cur_step)

                    # UAV做对应的能量衰减
                    energy_consumption = my_drone.energymodel.compute_transmit_consumption()
                    my_drone.residual_energy -= energy_consumption
                else:
                    to_drop_packet.append(pkd)

            for i in to_drop_packet:
                my_drone.remove_packet(i)              # 将已经传递给dst的packet从自己的buffer中删除
                my_drone.remove_sendbuffer_packet(i)   # 如果在自身的sending buffer中那也需要相应删除

    # 接收packet
    def packet_reception(self, my_drone: Drones, src_drone: Drones, packet, cur_step):
        # 前提是UAV还有足够的Buffer容量
        global gl_id_ack_packet

        # 对收包做能量衰减
        energy_consumption = my_drone.energymodel.compute_receive_consumption()
        my_drone.residual_energy -= energy_consumption

        if isinstance(packet, GpsrHelloPacket):
            self.neighbor_table.add_neighbor(packet, cur_step)

        elif isinstance(packet, DataPacket):
            # 首先判断该data packet的TTL是否超过MAX_TTL,超过则不接受它(即将他丢弃)
            if packet.get_current_TTL() < config.MAX_TTL:
                # 判断该data packet的最终目的地是不是自己
                if packet.dst_drone.identifier == my_drone.identifier:            # 自己是该data packet的目的地
                    packet.time_delivery = cur_step                               # 将当前时刻记录为packet的到达时刻
                    if packet.packet_id not in self.simulator.metrics.deliver_time_dict.keys():  # dst还没有收到过这个packet
                        self.simulator.metrics.deliver_time_dict[packet.packet_id] = cur_step - packet.time_step_creation
                        self.simulator.metrics.hop_numbers_of_data_packet_dict[packet.packet_id] = packet.get_current_TTL()
                        self.simulator.metrics.total_time.append(cur_step)
                    self.simulator.metrics.datapacket_to_destination.add(packet.packet_id)

                    # 为此次数据包的接收构建一个ACK应答
                    gl_id_ack_packet += 1
                    ack_packet = ACKPacket(my_drone, src_drone, gl_id_ack_packet, self.simulator, packet, cur_step)
                    self.unicast_message(ack_packet, my_drone, src_drone, cur_step)
                else:
                    my_drone.accept_packet(packet)

                    # 为此次数据包的接收构建一个ACK应答
                    gl_id_ack_packet += 1
                    ack_packet = ACKPacket(my_drone, src_drone, gl_id_ack_packet, self.simulator, packet, cur_step)
                    self.unicast_message(ack_packet, my_drone, src_drone, cur_step)

                energy_consumption = my_drone.energymodel.compute_transmit_consumption()
                my_drone.residual_energy -= energy_consumption

        elif isinstance(packet, ACKPacket):
            my_drone.remove_packet(packet.ack_packet)
            my_drone.remove_sendbuffer_packet(packet.ack_packet)

    # GPSR routing
    def routing(self, my_drone, drones, cur_step):
        if not my_drone.failure:                                    # UAV没有发生故障
            self.send_hello_packet(my_drone, drones, cur_step)
            if not my_drone.no_transmission:                        # UAV还有足够的剩余能量
                self.send_data_packet(my_drone, cur_step)
            else:
                pass
        else:
            pass
