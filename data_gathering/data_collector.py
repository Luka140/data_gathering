import rclpy 
from rclpy.node import Node
from rclpy.impl import rcutils_logger

from std_msgs.msg import Header 
from stamped_std_msgs.msg import Float32Stamped, Int32Stamped, TimeSync
from ferrobotics_acf.msg import ACFTelem, ACFTelemStamped

import pyads
import ctypes
from datetime import datetime


class DataCollector(Node):

    def __init__(self) -> None:
        super().__init__('acf')
        self.init_parameters()
        self.connect_plc()
        self.init_handles()
                
        # ===================================================================================================
        grinder_enabled         = True  # ------- Set to True for the grinder to be enabled --------
        # ===================================================================================================
        
        # Setup ROS interfaces 
        self.publisher_force     = self.create_publisher(Float32Stamped, '/acf/force', 10)
        self.publisher_rpm       = self.create_publisher(Int32Stamped, '/grinder/rpm', 10)
        self.publisher_time_sync = self.create_publisher(TimeSync, '/timesync', 10)
        self.telem_listener      = self.create_subscription(ACFTelemStamped, '/acf/telem', self.telem_callback, 10)
        self.timer               = self.create_timer(self.timer_period, self.timer_callback)
        
        # Timer for publishing TimeSync messages. This is set to a timer to ensure that there will be a usable message in the rosbag.
        # A message may not contain useful data if the PLC was not properly connected before creating a message 
        self.time_sync_timer     = self.create_timer(2, self.sync_callback)
        
        # Get notifications from the PLC 
        atr = pyads.NotificationAttrib(ctypes.sizeof(ctypes.c_uint32))
        self.rpm_notification_handle = self.plc.add_device_notification("GVL_Var.iINEncoderSpeed", atr, self.rpm_callback)
               
        # Retract ACF before startup
        self.publisher_force.publish(Float32Stamped(header=Header(), data=-10.))
        
        # Set desired RPM to zero and turn on the grinder 
        self.plc.write_by_name(self.rpm_control_var, 0)
        self.plc.write_by_name(self.grinder_on_var, grinder_enabled)
        
        self.get_logger().info('DataCollector initialised')
        if not grinder_enabled:
            self.get_logger().warn('\n\n\n WARNING: the grinder is turned off - set "grinder_enabled" to "True" to turn it on \n\n\n')
        self.init_time = self.get_clock().now()

    def init_parameters(self) -> None:
        self.declare_parameter('plc_target_ams',     '5.149.234.177.1.1')    # AMS ID of the PLC
        self.declare_parameter('plc_target_ip',      '169.254.200.16')       # IP of the PLC
         
        self.declare_parameter('timer_period',       '0.050')                # Time in seconds
        self.declare_parameter('max_contact_time',   '5.')                   # Time to grind in seconds
        self.declare_parameter('timeout_time',       '20.')                  # Duration before timeout in seconds
        self.declare_parameter('time_before_extend', '2.')                   # Duration after startup before extending the acf
        
        self.declare_parameter('force_desired',      '10.')                  # ACF force in newtons
        self.declare_parameter('rpm_control_var',    'Flow_Control_valve.iScaledDesiredFlowRate') 
        self.declare_parameter('desired_flowrate_scaled', '40')              # Percentage of flowrate to control the grinder RPM
        
        # Maybe this should change to main.bOnOff but it will currently be overwritten by the HMI interface
        self.declare_parameter('grinder_on_var',    'HMI.bOnOffPB')         # PLC variable controlling the on/off switch of the grinder 
        self.declare_parameter('time_var',          'GVL_Var.sTimeSt')      # PLC variable for timestamp
                                       
        self.target_ams_plc             = self.get_parameter('plc_target_ams').get_parameter_value().string_value
        self.target_ip_plc              = self.get_parameter('plc_target_ip').get_parameter_value().string_value
        
        self.timer_period               = float(self.get_parameter('timer_period').get_parameter_value().string_value)
        self.contact_time               = float(self.get_parameter('max_contact_time').get_parameter_value().string_value)
        self.force_desired              = float(self.get_parameter('force_desired').get_parameter_value().string_value)   
        self.timeout_time               = float(self.get_parameter('timeout_time').get_parameter_value().string_value)                           
        self.startup_time               = float(self.get_parameter('time_before_extend').get_parameter_value().string_value)         
        self.desired_flowrate_scaled    = int(self.get_parameter('desired_flowrate_scaled').get_parameter_value().string_value)
        
        self.rpm_control_var            = self.get_parameter('rpm_control_var').get_parameter_value().string_value  
        self.grinder_on_var             = self.get_parameter('grinder_on_var').get_parameter_value().string_value   
        self.time_var                   = self.get_parameter('time_var').get_parameter_value().string_value   
        
        self.initial_contact_time       = None # Time at which initial contact was made (None until contact) 
        self.max_contact_time           = rclpy.duration.Duration(seconds=self.contact_time)
        self.timeout                    = rclpy.duration.Duration(seconds=self.timeout_time)
        self.spin_up_duration           = rclpy.duration.Duration(seconds=self.startup_time)
    
    def connect_plc(self) -> None:
        self.plc = pyads.Connection(ams_net_id      =self.target_ams_plc, 
                                    ams_net_port    =pyads.PORT_TC3PLC1,
                                    ip_address      =self.target_ip_plc)

        self.plc.open()
        
        local_ad = self.plc.get_local_address()
        self.get_logger().info(f"\nLocal netid: {local_ad.netid} on port {local_ad.port}")
        self.get_logger().info(f"\nPLC State: \n{self.plc.read_state()}")
   
    def init_handles(self) -> None:
        # Using handles with pyads.read/write_by_name is faster than just using the variable name 
        # Do not forget to release any handles that are added - they will reduce the bandwidth to the plc otherwise
        self.rpm_control_handle = self.plc.get_handle(self.rpm_control_var)
        self.time_handle        = self.plc.get_handle(self.time_var)
        self.get_logger().info("Handles initialised")
        
    def release_handles(self) -> None:
        self.plc.release_handle(self.rpm_control_handle)
        self.plc.release_handle(self.time_handle)
        self.plc.del_device_notification(self.rpm_notification_handle)
    
    def timer_callback(self) -> None:
        
        time = self.get_clock().now()
    
        # Turn off if the maximum contact time has been exceeded 
        if self.initial_contact_time is not None and self.initial_contact_time + self.max_contact_time <= time:
            self.get_logger().info(f'Grind time of {self.max_contact_time} exceeded')
            self.turn_off_sequence()
            
        # Turn off if the maximum runtime for timeout had been exceeded
        if self.init_time + self.timeout < time:
            self.get_logger().info(f"Maximum runtime of {self.timeout_time} seconds exceeded")
            self.turn_off_sequence()
            raise TimeoutError("Maximum time exceeded")
        
        # Send command to the acf if past spin up time 
        if self.init_time + self.spin_up_duration <= time:
            header = Header() 
            header.stamp = self.get_clock().now().to_msg()
            force = Float32Stamped(header=header, data=self.force_desired)
            self.publisher_force.publish(force)
        
        # Send command to the plc - TODO this is probably not required - the desired rpm should stay the same 
        self.plc.write_by_name('', self.desired_flowrate_scaled, pyads.PLCTYPE_UINT, handle=self.rpm_control_handle)

    def turn_off_sequence(self) -> None:
        self.get_logger().info(f"Turning off")
        # Retract the acf
        self.force_desired = -5.
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        force = Float32Stamped(header=header, data=self.force_desired)
        self.publisher_force.publish(force)
        
        # Stop the grinder
        self.plc.write_by_name(self.grinder_on_var, False)
        self.desired_flowrate_scaled = 0 
        self.plc.write_by_name('', self.desired_flowrate_scaled, pyads.PLCTYPE_UINT, handle=self.rpm_control_handle)
        
        # Stop the command timer 
        self.timer.cancel()
        
    def rpm_callback(self, notification, data) -> None:
        # self.get_logger(f'Notification type: {type(notification)} datatype: {type(data)}')
        # This publisher only exists to publish the rpm notifications so it is recorded in the rosbag
        handle, timestamp, value = self.plc.parse_notification(notification, pyads.PLCTYPE_UDINT)
        
        header = Header(stamp=self.datetime_to_msg(timestamp))
        self.publisher_rpm.publish(Int32Stamped(data=value, header=header))
    
    def telem_callback(self, msg:ACFTelemStamped):
        force_abs_relative_error = abs((self.force_desired - msg.telemetry.force) / (self.force_desired + 1e-3)) # Added 1 mN in divisor to avoid div by zero
        if self.initial_contact_time is None and msg.telemetry.in_contact and (force_abs_relative_error < 0.1):
            self.initial_contact_time = self.get_clock().now()
            
    def sync_callback(self):
        sync_msg = TimeSync()
        plc_datetime = datetime.fromisoformat(self.plc.read_by_name('', pyads.PLCTYPE_STRING, handle=self.time_handle))
        ros_time = self.get_clock().now().to_msg()
        plc_time = self.datetime_to_msg(plc_datetime)

        sync_msg.header1.stamp, sync_msg.header1.frame_id = ros_time, 'ros_time'
        sync_msg.header2.stamp, sync_msg.header2.frame_id = plc_time, 'plc_time'

        self.publisher_time_sync.publish(sync_msg)
        
    def datetime_to_msg(self, dt_object:datetime):
        sec = dt_object.second + dt_object.minute * 60 + dt_object.hour * 60**2
        time_msg = rclpy.time.Time(seconds=sec,
                                   nanoseconds=dt_object.microsecond * 1000).to_msg()
        return time_msg
     
            
def main(args=None):
    rclpy.init(args=args)

    global_logger = rcutils_logger.RcutilsLogger(name="global_logger")
    data_collector = DataCollector()

    try:
        rclpy.spin(data_collector)
    except (rclpy.executors.ExternalShutdownException, KeyboardInterrupt):
        global_logger.info("Shutting down due to external shutdown or keyboard interrupt.")
    except Exception as e:
        global_logger.error(f"Error encountered: {e}")
    finally:        
        global_logger().info(f"Performing cleanup actions")
        
        try:
            # Retract ACF
            data_collector.publisher_force.publish(Float32Stamped(data=-10., header=Header()))
        except Exception as e:
            global_logger().info(f'Error encountered while retracting the ACF during shutdown: \n{e}')
        
        try:    
            # Turn off grinder and set the goal RPM to zero
            data_collector.plc.write_by_name(data_collector.grinder_on_var, False)
            data_collector.plc.write_by_name(data_collector.rpm_control_handle, 0)
            data_collector.release_handles()
            data_collector.plc.close()
        except Exception as e:
            global_logger().info(f'Error encountered while turning off the grinder and releasing handles during shutdown: \n{e}')
            
        
        data_collector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    