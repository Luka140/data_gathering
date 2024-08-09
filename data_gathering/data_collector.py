import rclpy 
from rclpy.node import Node

import pyads

class DataCollector(Node):

    def __init__(self):
        super().__init__('acf')
        self.init_parameters()
        self.connect_plc()
        
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.get_logger().info('DataCollector initialised')

    def init_parameters(self):
        self.declare_parameter('plc_target_ams', '5.149.234.177.1.1')
        self.declare_parameter('plc_target_ip', '169.254.200.16')
        self.declare_parameter('timer_period', '1e6') # Time in nanoseconds
                
        self.target_ams_plc = self.get_parameter('plc_target_ams').get_parameter_value().string_value
        self.target_ip_plc  = self.get_parameter('plc_target_ip').get_parameter_value().string_value
        self.timer_period   = self.get_parameter('timer_period').get_parameter_value().double_value
    
    def connect_plc(self):

        self.plc = pyads.Connection(ams_net_id      =self.target_ams_plc, 
                                    ams_net_port    =pyads.PORT_TC3PLC1,
                                    ip_address      =self.target_ip_plc)

        self.plc.open()
        local_ad = self.plc.get_local_address()
        print(f"\nLocal netid: {local_ad.netid} on port {local_ad.port}")
        # check the connection state
        print(f"\nResult of read state: \n{self.plc.read_state()}")
    
    def timer_callback(self):
        
        # Send command to the plc         
        ...
        
        # Send command to the acf 
        ...

def main(args=None):
    rclpy.init(args=args)
    
    data_collector = DataCollector()
    rclpy.spin(data_collector)

    data_collector.plc.close()

    data_collector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()