import rclpy 
from rclpy.node import Node

from std_msgs.msg import Float32, Int32
from ferrobotics_acf.msg import ACFTelem

import pyads
import ctypes


class DataCollector(Node):

    def __init__(self):
        super().__init__('acf')
        self.init_parameters()
        self.connect_plc()
        
        self.publisher_force    = self.create_publisher(Float32, '/acf/force', 10)
        self.publisher_rpm      = self.create_publisher(Int32, '/grinder/rpm', 10)
        self.telem_listener     = self.create_subscription(ACFTelem, '/acf/telem', self.telem_callback, 10)
        self.timer              = self.create_timer(self.timer_period, self.timer_callback)
        
        # TODO make parameters and move to init_parameters
        self.initial_contact_time       = None                                          # Time at which initial contact was made (initually None until contact) 
        self.max_contact_time           = 5e9                                           # Time to grind in nanoseconds
        self.force_desired              = 10.                                           # Desired force on the acf (N)      
        self.rpm_control_var            = "Flow_Control_valve.iScaledDesiredFlowRate"
        self.desired_flowrate_scaled    = 40                                            # Percentage of flowrate to control the grinder RPM
        
        # Maybe this should change to main.bOnOff but it will currently be overwritten by the HMI interface
        self.grinder_on_var             = "HMI.bOnOffPB"                                # PLC variable controlling the on/off switch of the grinder 
        
        # ================================================================================================================
        grinder_enabled                 = False  # -------------- Set to True for the grinder to be enabled --------------
        # ================================================================================================================
    
        self.plc.write_by_name(self.grinder_on_var, grinder_enabled)
        
        atr = pyads.NotificationAttrib(ctypes.sizeof(ctypes.c_uint32))
        # TODO the returned rpm seems to be wrong, probablyPLCTYPE_UDINT datatype issue 
        handles = self.plc.add_device_notification("Encoder.iEncoderActualRPM", atr, self.rpm_callback)
        
        self.get_logger().info('DataCollector initialised')

    def init_parameters(self):
        self.declare_parameter('plc_target_ams', '5.149.234.177.1.1')
        self.declare_parameter('plc_target_ip',  '169.254.200.16')
        self.declare_parameter('timer_period',   '0.005') # Time in seconds
                
        self.target_ams_plc = self.get_parameter('plc_target_ams').get_parameter_value().string_value
        self.target_ip_plc  = self.get_parameter('plc_target_ip').get_parameter_value().string_value
        self.timer_period   = float(self.get_parameter('timer_period').get_parameter_value().string_value)
    
    
    def connect_plc(self):
        self.plc = pyads.Connection(ams_net_id      =self.target_ams_plc, 
                                    ams_net_port    =pyads.PORT_TC3PLC1,
                                    ip_address      =self.target_ip_plc)

        self.plc.open()
        
        local_ad = self.plc.get_local_address()
        self.get_logger().info(f"\nLocal netid: {local_ad.netid} on port {local_ad.port}")
        self.get_logger().info(f"\nPLC State: \n{self.plc.read_state()}")
    
    def timer_callback(self):
        
        # If the maximum contact time has been exceeded 
        if self.initial_contact_time is not None and self.initial_contact_time.nanoseconds + self.max_contact_time <= self.get_clock().now().nanoseconds:
            # Retract the acf
            self.force_desired = -5.
            
            # Stop the grinder
            self.plc.write_by_name(self.grinder_on_var, False)
            self.desired_flowrate_scaled = 0 
            
            self.get_logger().info(f'Grind time of {self.max_contact_time} exceeded')
    
        # Send command to the plc    
        force = Float32(data=self.force_desired)
        self.publisher_force.publish(force)
        
        # Send command to the acf 
        self.plc.write_by_name(self.rpm_control_var, self.desired_flowrate_scaled)
        
    
    def rpm_callback(self, notification, data):
        contents = notification.contents
        self.get_logger().info(f"contents.data: {contents.data}")
        self.publisher_rpm.publish(Int32(data=contents.data))
    
    def telem_callback(self, msg:ACFTelem):
        if self.initial_contact_time is None and msg.in_contact:
            self.initial_contact_time = self.get_clock().now()
            

def main(args=None):
    rclpy.init(args=args)
    
    data_collector = DataCollector()
    try:
        rclpy.spin(data_collector)
    except:
        data_collector.publisher_force.publish(Float32(data=-5.))
        data_collector.plc.write_by_name(data_collector.grinder_on_var, False)
        data_collector.plc.write_by_name(self.rpm_control_var, 0)
        
    data_collector.plc.close()

    data_collector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()