import pyads.errorcodes
import rclpy 
from rclpy.node import Node
from rclpy.impl import rcutils_logger
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_msgs.msg import Header 
from stamped_std_msgs.msg import Float32Stamped, Int32Stamped, TimeSync
from ferrobotics_acf.msg import ACFTelemStamped

from std_srvs.srv import Trigger
from data_gathering_msgs.srv import StartGrinder, StopGrinder

import pyads
import ctypes
from datetime import datetime


class GrinderNode(Node):

    def __init__(self) -> None:
        super().__init__('grinder_node')
        self.init_parameters()
        self.connect_plc()
        self.init_handles()
                
        # Setup ROS interfaces 
        self.publisher_rpm       = self.create_publisher(Int32Stamped, '~/rpm', 10)
        self.publisher_rpm_req   = self.create_publisher(Int32Stamped, '~/requested_rpm', 10)
        self.publisher_time_sync = self.create_publisher(TimeSync, '~/timesync', 10)

        # Without a mutually exclusive group, this callback may lead to multiple test_done timers being created unintentionally
        self.start_grinder        = self.create_service(StartGrinder, '~/enable_grinder', self.start_grinder, callback_group=MutuallyExclusiveCallbackGroup())
        self.stop_grinder         = self.create_service(StopGrinder, '~/disable_grinder', self.stop_grinder, callback_group=MutuallyExclusiveCallbackGroup())

        # Initialize flags 
        self.test_success = None                # Indicates whether the test was successful
        self.failure_message = ''               # Stores potential failure messages
        
        self.test_running       = False         # Switch to ingore telem data if not currently testing - and reject enable grinder requests
        self.belt_halted        = False         # Flag for test failure because the grinding belt got caught 
        self.shutdown_started   = False         # Indicates whether the shutdown procedure started 
        self.belt_spun_up       = False         # Indicates whether the belt has approximately reached the desired RPM 

        self.retry_rate = self.create_rate(5)
        self.plc_retries = 10

        # Timer for publishing TimeSync messages. This is set to a timer to ensure that there will be a usable message in the rosbag.
        # A message may not contain useful data if the PLC was not properly connected before creating a message 
        self.time_sync_timer     = self.create_timer(0.5, self.sync_callback)
        self.test_timed_out_timer = None 

        # Get notifications from the PLC 
        atr = pyads.NotificationAttrib(ctypes.sizeof(ctypes.c_uint32))
        self.rpm_notification_handle, self._user_handle = self.plc.add_device_notification("GVL_Var.Actual_RPM", atr, self.rpm_callback, user_handle=3)
        
        # Set desired RPM to zero, turn off the grinder 
        self.desired_rpm = 0 
        self.plc.write_by_name(self.rpm_control_var, 0)
        self.plc.write_by_name(self.grinder_on_var, False)
        
       
        self.get_logger().info('DataCollector initialised')
        if not self.grinder_enabled:
            self.get_logger().warn('\n\n\n WARNING: the grinder is turned off - set "grinder_enabled" to "True" to turn it on \n\n')

    def init_parameters(self) -> None:
        self.declare_parameter('plc_target_ams', '5.149.234.177.1.1')    # AMS ID of the PLC
        self.declare_parameter('plc_target_ip', '169.254.200.16')        # IP of the PLC
         
        # self.declare_parameter('timeout_time', 30.0)                     # Duration before timeout in seconds (float)
        # self.declare_parameter('time_before_extend', 3.0)                # Duration after startup before extending the acf to allow for belt spin-up (float)
        
        self.declare_parameter('rpm_control_var', 'Flow_Control_valve.iScaledDesiredFlowRate') 
        self.declare_parameter('grinder_on_var', 'HMI.bOnOffPB')         # PLC variable controlling the on/off switch of the grinder
        self.declare_parameter('grinder_enabled', True)                  # Turn the grinder on/off with True/False
        self.declare_parameter('time_var', 'GVL_Var.sTimeSt')            # PLC variable for timestamp

        # self.declare_parameter('max_acf_extension', 35.5)                 # The maximum extension that can be reached by the ACF (float)

        # Retrieve parameters (as the correct types)
        self.target_ams_plc         = self.get_parameter('plc_target_ams').get_parameter_value().string_value
        self.target_ip_plc          = self.get_parameter('plc_target_ip').get_parameter_value().string_value
        # self.timeout_duration       = self.get_parameter('timeout_time').get_parameter_value().double_value
        # self.spin_up_duration       = self.get_parameter('time_before_extend').get_parameter_value().double_value
        
        self.rpm_control_var        = self.get_parameter('rpm_control_var').get_parameter_value().string_value  
        self.grinder_on_var         = self.get_parameter('grinder_on_var').get_parameter_value().string_value   
        self.grinder_enabled        = self.get_parameter('grinder_enabled').get_parameter_value().bool_value
        self.time_var               = self.get_parameter('time_var').get_parameter_value().string_value   
        
        # self.max_acf_extension      = self.get_parameter('max_acf_extension').get_parameter_value().double_value

    
    def connect_plc(self) -> None:
        self.plc = pyads.Connection(ams_net_id      =self.target_ams_plc, 
                                    ams_net_port    =pyads.PORT_TC3PLC1,
                                    ip_address      =self.target_ip_plc)

        self.plc.open()
        
        local_ad = self.plc.get_local_address()
        self.get_logger().info(f"\nLocal netid: {local_ad.netid} on port {local_ad.port}")
        self.get_logger().info(f"\nPLC State: \n{self.plc.read_state()}")
   
    def init_handles(self) -> None:
        """ Initializes PLC handles.
                Using handles with pyads.read/write_by_name is faster than just using the variable name. 
                Do not forget to release any handles that are added - they will reduce the bandwidth to the plc otherwise.
        """
         
        self.rpm_control_handle = self.plc.get_handle(self.rpm_control_var)
        self.time_handle        = self.plc.get_handle(self.time_var)
        self.get_logger().info("Handles initialised")
        
    def release_handles(self) -> None:
        """ Release the PLC handles after testing """
        self.plc.release_handle(self.rpm_control_handle)
        self.plc.release_handle(self.time_handle)
        self.plc.del_device_notification(self.rpm_notification_handle, self._user_handle)

    def start_grinder(self, request, response):
        
        # If already enabled 
        if self.test_running:
            response.success = False
            response.message = "The grinder is already enabled"
            return response 
        
        # Reset flags 
        self.test_success = True    # Indicates whether the grind was successful - this will only be returned once the grinder is turned off again.
        self.failure_message = ''   # Stores potential failure messages to be returned when the grinder is turned off again.
        self.test_running = True    # Ensure that only one test can be started 
        self.belt_spun_up = False 

        # Will flip to false if issue occurs 
        response.success = True 
        response.message = ""

        # Convert RPM into flowrate scaled
        # This is the variable that controls RPM in the PLC 
        desired_flowrate_scaled = self.rpm_to_flowrate(request.rpm) 

        # Store the RPM setpoint to compare the current RPM to in the RPM callback
        self.desired_rpm = request.rpm 

        self.get_logger().info(f"Settings received:\n  RPM: {request.rpm}\n  Timeout: {request.timeout_duration} seconds")

        # Publish RPM message for logging purposes
        self.publisher_rpm_req.publish(Int32Stamped(data=int(request.rpm), header=Header(stamp=self.get_clock().now().to_msg())))
        
        for i in range(self.plc_retries):
            try:
                self.plc.write_by_name('', desired_flowrate_scaled, pyads.PLCTYPE_REAL, handle=self.rpm_control_handle)
                self.plc.write_by_name(self.grinder_on_var, self.grinder_enabled)
            except pyads.pyads_ex.ADSError as e:
                response.success = False 
                response.message = f"ADS error occured: {e}"
                self.get_logger().error(f"ADS error occured: {e}")
                self.get_logger().info(f"Retrying {self.plc_retries - i -1} more times") 
                self.retry_rate.sleep()
                if not self.plc.is_open:
                    self.plc.open()
                    self.retry_rate.sleep()
                continue

            # Only reached if a retry was successful 
            response.success = True 
            response.message = ""
            break 

        if request.timeout_duration > 0.0:
            self.test_timed_out_timer = self.create_timer(request.timeout_duration, callback=self.timeout)

        self.get_logger().info("Finished enable grinder request")
        return response
    
    def stop_grinder(self, _, response):
        response.success = True 
        response.message = ""
        
        # Grinder has already stopped - potentially because of timeout
        if not self.test_running:
            # Check if actually true and not false flag
            grinder_on_plc = self.plc.read_by_name(self.grinder_on_var)
            
            if not grinder_on_plc:
                response.success = False 
                response.message = "Grinder was already disabled"
                return response 

            # Flag was set to false but the grinder was actually enabled...
            response.message = "Grinder on flag was set to false, but the grinder was still running.\n"
        
        response.grind_successful = self.test_success
        response.grind_message = self.failure_message
            
        shutdown_success, shutdown_msg = self.shutdown_sequence()
        if not shutdown_success:
            response.success = False 
            response.message += f"\n{shutdown_msg}"
            response.grind_successful = False 
            response.grind_message += f"\n{shutdown_msg}"
        self.get_logger().info(f"response success: {response.success}\nresponse msg: {response.message}\ngrind_success: {response.grind_successful}\ngrind_msg: {response.grind_message}")
        return response 
        
    def timeout(self):
        """ Timer callback for when the timeout time has elapsed eventhough the requested contact duration was not reached """
        if self.test_timed_out_timer is not None:
            self.test_timed_out_timer.cancel()
        self.test_success = False   
        self.failure_message += f'Test timed out. Duration of {self.max_contact_time} exceeded.'
        self.get_logger().info(f'Test timed out. Duration of {self.max_contact_time} exceeded.')
        shutdown_success, shutdown_msg = self.shutdown_sequence()
        if not shutdown_success:
            self.failure_message += f"\n{shutdown_msg}"

    def shutdown_sequence(self) -> tuple[bool, str]:
        self.shutdown_started = True 
        self.get_logger().info(f"Turning off the grinder")
        
        if self.test_timed_out_timer is not None:
            self.test_timed_out_timer.cancel()

        shutdown_success = True 
        shutdown_msg = ""

        # Stop the grinder
        for i in range(self.plc_retries):
            try:
                self.plc.write_by_name('', 0, pyads.PLCTYPE_REAL, handle=self.rpm_control_handle)
                self.plc.write_by_name(self.grinder_on_var, False)
            except pyads.pyads_ex.ADSError as e:
                shutdown_success = False 
                shutdown_msg = f"ADS error occured: {e}"
                self.get_logger().error(f"ADS error occured: {e}")
                self.get_logger().info(f"Retrying {self.plc_retries - i -1} more times") 
                self.retry_rate.sleep()
                if not self.plc.is_open:
                    self.plc.open()
                    self.retry_rate.sleep()
                continue

            # Only reached if a retry was successful 
            shutdown_success = True 
            shutdown_msg = ""
            break 

        if not shutdown_success:
            self.get_logger.error(f"FAILED TO SHUT DOWN GRINDER AFTER {self.plc_retries} ATTEMPTS -  PRESS E-STOP!")
            return shutdown_success, shutdown_msg

        # Set reset flag 
        self.test_running = False 
        return shutdown_success, shutdown_msg

    def rpm_callback(self, notification, data) -> None:
        """ This callback exists to publishe the rpm notifications so it is recorded in the rosbag """
        handle, timestamp, value = self.plc.parse_notification(notification, pyads.PLCTYPE_UDINT)
        header = Header(stamp=self.datetime_to_msg(timestamp))

        # If belt is in spin up phase, check whether it has reached 80% RPM 
        if not self.belt_spun_up:
            if abs(value / (self.desired_rpm + 1e-6)) > 0.8:
                self.belt_spun_up = True 
        
        # Else if the belt is past spin up, check whether the belt is getting caught 
        elif not self.belt_halted and not self.shutdown_started and abs(value / (self.desired_rpm + 1e-6)) < 0.3:
            self.test_success = False 
            self.belt_halted = True 
            self.failure_message += '\nThe belt RPM dropped below 30 percent of the target RPM during contact.'

        # Publish RPM for logging 
        self.publisher_rpm.publish(Int32Stamped(data=value, header=header))

    def sync_callback(self):
        """Creates a TimeSync message which contains a timestamp from the PLC and ROS to compare later.
        """
        sync_msg = TimeSync()
        for i in range(self.plc_retries):
            try:    
                plc_datetime = datetime.fromisoformat(self.plc.read_by_name('', pyads.PLCTYPE_STRING, handle=self.time_handle))
            except pyads.pyads_ex.ADSError as e:
                self.get_logger().error(f"ADS error occured: {e}")
                self.get_logger().info(f"Retrying {self.plc_retries - i -1} more times") 
                self.retry_rate.sleep()
                if not self.plc.is_open:
                    self.plc.open()
                    self.retry_rate.sleep()
                continue
            break 
        
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
    
    def rpm_to_flowrate(self, rpm):
        """
        Calculates the desired_flowrate_scaled based on an inversion of how RPM is calculated in the PLC code.
        """
        return 100 * (rpm - 3400) / 7600
     
            
def main(args=None):
    rclpy.init(args=args)

    global_logger = rcutils_logger.RcutilsLogger(name="global_logger")
    grinder_node = GrinderNode()
    executor = MultiThreadedExecutor()

    # The try except block are an attempt at ensuring that the grinder is turned off and retracted if an error occurs
    try:
        rclpy.spin(grinder_node, executor=executor)
    except (rclpy.executors.ExternalShutdownException, KeyboardInterrupt, TimeoutError):
        global_logger.info("Shutting down due to external shutdown, keyboard interrupt or timeout")
    except Exception as e:
        global_logger.error(f"Error encountered: {e}")
    finally:        
        global_logger.info(f"Performing cleanup actions")
        
        try:    
            # Turn off grinder and set the goal RPM to zero
            grinder_node.plc.write_by_name(grinder_node.grinder_on_var, False)
            grinder_node.plc.write_by_name('', 0, pyads.PLCTYPE_REAL, handle=grinder_node.rpm_control_handle)
            grinder_node.release_handles()
            grinder_node.plc.close()
        except Exception as e:
            global_logger.info(f'Error encountered while turning off the grinder and releasing handles during shutdown: \n{e}')
            
        grinder_node.destroy_node()
        # Avoid stack trace 
        try:
            rclpy.shutdown()
        except rclpy._rclpy_pybind11.RCLError:
            pass 


if __name__ == '__main__':
    main()
    