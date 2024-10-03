import rclpy 
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Header 
from stamped_std_msgs.msg import Float32Stamped, Int32Stamped, TimeSync


import pyads
import ctypes
from datetime import datetime


class RPMPublisher(Node):
    def __init__(self):
        super().__init__('rpm_publisher')

        self.declare_parameter('plc_target_ams',     '5.149.234.177.1.1')    # AMS ID of the PLC
        self.declare_parameter('plc_target_ip',      '169.254.200.16')       # IP of the PLC        
                                       
        self.target_ams_plc             = self.get_parameter('plc_target_ams').get_parameter_value().string_value
        self.target_ip_plc              = self.get_parameter('plc_target_ip').get_parameter_value().string_value

        self.connect_plc()

        # Publisher for the RPM data
        self.publisher_rpm = self.create_publisher(Int32Stamped, '/grinder/rpm', 10)

        # Get notifications from the PLC 
        atr = pyads.NotificationAttrib(ctypes.sizeof(ctypes.c_uint32))
        self.rpm_notification_handle = self.plc.add_device_notification("GVL_Var.Actual_RPM", atr, self.rpm_callback)

        self.latest_rpm = None 
        self.create_timer(1/60, self.publish)

           
    def connect_plc(self) -> None:
        self.plc = pyads.Connection(ams_net_id      =self.target_ams_plc, 
                                    ams_net_port    =pyads.PORT_TC3PLC1,
                                    ip_address      =self.target_ip_plc)

        self.plc.open()
        
        local_ad = self.plc.get_local_address()
        self.get_logger().info(f"\nLocal netid: {local_ad.netid} on port {local_ad.port}")
        self.get_logger().info(f"\nPLC State: \n{self.plc.read_state()}")

    
    def rpm_callback(self, notification, data) -> None:
        # self.get_logger(f'Notification type: {type(notification)} datatype: {type(data)}')
        # This publisher only exists to publish the rpm notifications so it is recorded in the rosbag
        self.latest_rpm = self.plc.parse_notification(notification, pyads.PLCTYPE_UDINT)
        # handle, timestamp, value = self.plc.parse_notification(notification, pyads.PLCTYPE_UDINT)

        handle, timestamp, value = self.latest_rpm
        header = Header(stamp=self.datetime_to_msg(timestamp))
        self.publisher_rpm.publish(Int32Stamped(data=value, header=header))
    

    def publish(self):
        if self.latest_rpm is None:
            return 
        
        handle, timestamp, value = self.latest_rpm

        header = Header(stamp=self.datetime_to_msg(timestamp))
        self.publisher_rpm.publish(Int32Stamped(data=value, header=header))
    

    def datetime_to_msg(self, dt_object:datetime):
        sec = dt_object.second + dt_object.minute * 60 + dt_object.hour * 60**2
        time_msg = rclpy.time.Time(seconds=sec,
                                   nanoseconds=dt_object.microsecond * 1000).to_msg()
        return time_msg

def main():
    rclpy.init()
    rpm_node = RPMPublisher()
    rclpy.spin(rpm_node, executor=MultiThreadedExecutor())
    rpm_node.destroy_node()
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()
