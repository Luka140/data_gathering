import pyads.errorcodes
import rclpy 
from rclpy.node import Node
from rclpy.impl import rcutils_logger
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_msgs.msg import Header 
from stamped_std_msgs.msg import Float32Stamped, Int32Stamped, TimeSync
from ferrobotics_acf.msg import ACFTelemStamped

from data_gathering_msgs.srv import TestRequest

import pyads
import ctypes
from datetime import datetime


class DataCollector(Node):

    def __init__(self) -> None:
        super().__init__('data_collector')
        self.init_parameters()
        self.connect_plc()
        self.init_handles()
                
        # Setup ROS interfaces 
        self.publisher_force     = self.create_publisher(Float32Stamped, '/acf/force', 10)
        self.publisher_rpm       = self.create_publisher(Int32Stamped, '~/rpm', 10)
        self.publisher_rpm_req   = self.create_publisher(Int32Stamped, '~/requested_rpm', 10)
        self.publisher_time_sync = self.create_publisher(TimeSync, '~/timesync', 10)

        # Without a mutually exclusive group, this callback may lead to multiple test_done timers being created unintentionally
        self.telem_listener      = self.create_subscription(ACFTelemStamped, '/acf/telem', self.telem_callback, 5, callback_group=MutuallyExclusiveCallbackGroup())
        self.test_server         = self.create_service(TestRequest, '~/execute_test', self.start_test, callback_group=MutuallyExclusiveCallbackGroup())

        # Initialize flags 
        self.test_done = False                  # Indicates when TestRequest response is finished. Set in shutdown_sequence
        self.test_success = None                # Indicates whether the test was successful
        self.failure_message = ''               # Stores potential failure messages
        self.initial_contact_time = None        # Stores the time at which the grinder initially made contact with the part
        self.test_running = False               # Switch to ingore telem data if not currently testing
        self.full_extension_reached = False     # Flag for test failure because of ACF maxing out extension
        self.belt_halted = False                # Flag for test failure because the grinding belt got caught 
        self.shutdown_started = False           # Indicates whether the shutdown procedure started 

        self.retry_rate = self.create_rate(5)
        self.plc_retries = 10

        # Timer for publishing TimeSync messages. This is set to a timer to ensure that there will be a usable message in the rosbag.
        # A message may not contain useful data if the PLC was not properly connected before creating a message 
        self.time_sync_timer     = self.create_timer(0.5, self.sync_callback)

        # Get notifications from the PLC 
        atr = pyads.NotificationAttrib(ctypes.sizeof(ctypes.c_uint32))
        self.rpm_notification_handle, self._user_handle = self.plc.add_device_notification("GVL_Var.Actual_RPM", atr, self.rpm_callback, user_handle=3)

        # Retract ACF before startup
        self.publisher_force.publish(Float32Stamped(header=Header(), data=-5.))
        self.force_desired = -5
        
        # Set desired RPM to zero, turn off the grinder and retract acf
        self.plc.write_by_name(self.rpm_control_var, 0)
        self.plc.write_by_name(self.grinder_on_var, False)
        
       
        self.get_logger().info('DataCollector initialised')
        if not self.grinder_enabled:
            self.get_logger().warn('\n\n\n WARNING: the grinder is turned off - set "grinder_enabled" to "True" to turn it on \n\n')

    def init_parameters(self) -> None:
        self.declare_parameter('plc_target_ams', '5.149.234.177.1.1')    # AMS ID of the PLC
        self.declare_parameter('plc_target_ip', '169.254.200.16')        # IP of the PLC
         
        self.declare_parameter('timeout_time', 20.0)                     # Duration before timeout in seconds (float)
        self.declare_parameter('time_before_extend', 3.0)                # Duration after startup before extending the acf to allow for belt spin-up (float)
        
        self.declare_parameter('rpm_control_var', 'Flow_Control_valve.iScaledDesiredFlowRate') 
        self.declare_parameter('grinder_on_var', 'HMI.bOnOffPB')         # PLC variable controlling the on/off switch of the grinder
        self.declare_parameter('grinder_enabled', False)                 # Turn the grinder on/off with True/False
        self.declare_parameter('time_var', 'GVL_Var.sTimeSt')            # PLC variable for timestamp

        self.declare_parameter('max_acf_extension', 0.3)                 # The maximum extension that can be reached by the ACF (float)

        # Retrieve parameters (as the correct types)
        self.target_ams_plc         = self.get_parameter('plc_target_ams').get_parameter_value().string_value
        self.target_ip_plc          = self.get_parameter('plc_target_ip').get_parameter_value().string_value
        self.timeout_duration       = self.get_parameter('timeout_time').get_parameter_value().double_value
        self.spin_up_duration       = self.get_parameter('time_before_extend').get_parameter_value().double_value
        
        self.rpm_control_var        = self.get_parameter('rpm_control_var').get_parameter_value().string_value  
        self.grinder_on_var         = self.get_parameter('grinder_on_var').get_parameter_value().string_value   
        self.grinder_enabled        = self.get_parameter('grinder_enabled').get_parameter_value().bool_value
        self.time_var               = self.get_parameter('time_var').get_parameter_value().string_value   
        
        self.max_acf_extension      = self.get_parameter('max_acf_extension').get_parameter_value().double_value

    
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

    def start_test(self, request, response):
        """ Main service callback. 
            
            This currently blocks an extra thread in a while loop while the test is executed. Fix later :)
        """
        self.test_running = True 
        self.test_done = False 
        self.shutdown_started = False 
        self.initial_contact_time = None  
        self.test_success = None  
        self.failure_message = ''
        self.full_extension_reached = False     
        self.belt_halted = False                

        self.force_desired = request.force 
        self.desired_rpm = request.rpm      # This should not be used to set the RPM, it is just used for some performance checks
        self.desired_flowrate_scaled = self.rpm_to_flowrate(request.rpm)    # This is the variable that controls RPM 
        self.max_contact_time = request.contact_time

        self.get_logger().info(f"Settings received:\n  Force: {self.force_desired} N\n  RPM: {request.rpm}\n  Duration: {request.contact_time} seconds")

        self.init_time = self.get_clock().now() 
        response.force = request.force
        response.rpm = request.rpm
        response.contact_time = request.contact_time 

        # Turn on grinder 
        self.publisher_rpm_req.publish(Int32Stamped(data=int(request.rpm), header=Header(stamp=self.get_clock().now().to_msg())))
        
        for i in range(self.plc_retries):
            try:
                self.plc.write_by_name('', self.desired_flowrate_scaled, pyads.PLCTYPE_REAL, handle=self.rpm_control_handle)
                self.plc.write_by_name(self.grinder_on_var, self.grinder_enabled)
            except pyads.pyads_ex.ADSError as e:
                self.get_logger().error(f"ADS error occured: {e}")
                self.get_logger().info(f"Retrying {self.plc_retries - i -1} more times") 
                self.retry_rate.sleep()
                if not self.plc.is_open:
                    self.plc.open()
                    self.retry_rate.sleep()
                continue
            break 


        # Start after spin up duration
        self.spin_up_wait = self.create_timer(self.spin_up_duration, self.spin_up_period_done)

        wait_for_finish_rate = self.create_rate(1)
        # Hang the response until the test is done as indicated by a shutdown_sequence call
        # TODO Blocks a thread.... Fix later :)
        # Maybe this should be an action server instead of a service. 
        
        while not self.test_done:
            wait_for_finish_rate.sleep()

        response.success = self.test_success
        response.message = self.failure_message

        # Set test_running to false to ignore telem in downtime
        self.test_running = False 
        return response
    
    def spin_up_period_done(self):
        # Spin up duration over - engage the ACF and start grinding.
        self.spin_up_wait.cancel()
        
        # Create timer after which the grinder will switch off in case contact is not detected
        # This is not the actual test duration, it is just a failsafe. 
        self.test_timed_out_timer = self.create_timer(self.timeout_duration, self.timeout)

        # Engage the grinder 
        header = Header() 
        header.stamp = self.get_clock().now().to_msg()
        force = Float32Stamped(header=header, data=self.force_desired)
        self.publisher_force.publish(force)
        
    def contact_time_exceeded(self):
        """ Timer callback for when the test time has elapsed """
        self.test_finished_timer.cancel()
        if self.test_success is None:
            self.test_success = True       # Set success flag 
        self.get_logger().info(f'Grind time of {self.max_contact_time} exceeded')
        self.shutdown_sequence()

    def timeout(self):
        """ Timer callback for when the timeout time has elapsed eventhough the requested contact duration was not reached """
        self.test_timed_out_timer.cancel()
        self.test_success = False   
        self.failure_message += f'Test timed out. Duration of {self.max_contact_time} exceeded. Likely no contact was detected or the desired force was never reached'
        self.get_logger().info(f'Test timed out. Duration of {self.max_contact_time} exceeded. Likely no contact was detected or the desired force was never reached')
        self.shutdown_sequence()

    def shutdown_sequence(self) -> None:
        self.shutdown_started = True 
        self.get_logger().info(f"Turning off")
        
        self.test_timed_out_timer.cancel()

        # Retract the acf
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        force = Float32Stamped(header=header, data=-5.)
        self.publisher_force.publish(force)
        
        # Stop the grinder
        self.desired_flowrate_scaled = 0 

        for i in range(self.plc_retries):
            try:    
                self.plc.write_by_name(self.grinder_on_var, False)
                self.plc.write_by_name('', 0, pyads.PLCTYPE_REAL, handle=self.rpm_control_handle)
            except pyads.pyads_ex.ADSError as e:
                self.get_logger().error(f"ADS error occured: {e}")
                self.get_logger().info(f"Retrying {self.plc_retries - i -1} more times") 
                self.retry_rate.sleep()
                if not self.plc.is_open:
                    self.plc.open()
                    self.retry_rate.sleep()
                continue
            break 
        
        # Stop the command timer 
        self.test_done = True 
        
    def rpm_callback(self, notification, data) -> None:
        """ This callback exists to publishe the rpm notifications so it is recorded in the rosbag """
        handle, timestamp, value = self.plc.parse_notification(notification, pyads.PLCTYPE_UDINT)
        header = Header(stamp=self.datetime_to_msg(timestamp))
        
        # Check whether the belt is getting caught 
        if self.initial_contact_time is not None and not self.belt_halted and not self.shutdown_started and abs(value / self.desired_rpm) < 0.3:
            self.test_success = False 
            self.belt_halted = True 
            self.failure_message += '\nThe belt RPM dropped below 30 percent of the target RPM during contact.'
    
        self.publisher_rpm.publish(Int32Stamped(data=value, header=header))

    def telem_callback(self, msg:ACFTelemStamped):
        if not self.test_running:
            return 

        # Set the initial contact time if a force within 10% of the desired force is reached
        force_abs_relative_error = abs((self.force_desired - msg.telemetry.force) / (self.force_desired + 1e-3)) # Added 1 mN in divisor to avoid div by zero
        if self.initial_contact_time is None and msg.telemetry.in_contact and (force_abs_relative_error < 0.1):
            self.initial_contact_time = self.get_clock().now()

            # Shutdown after contact duration is reached
            self.test_finished_timer = self.create_timer(self.max_contact_time, self.contact_time_exceeded)

        # If an acf extension within 2% of its max is reached, signal that the test may have failed 
        # In this case the ACF may be pushing on its endstop rather than the part
        if not self.full_extension_reached and abs(self.max_acf_extension - msg.telemetry.position) / self.max_acf_extension < 0.02:
            self.full_extension_reached = True
            self.test_success = False
            self.failure_message += '\nThe maximum extension of the ACF was reached. This means force may not have been maintained.'
            
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
    data_collector = DataCollector()
    executor = MultiThreadedExecutor()

    # The try except block are an attempt at ensuring that the grinder is turned off and retracted if an error occurs
    try:
        rclpy.spin(data_collector, executor=executor)
    except (rclpy.executors.ExternalShutdownException, KeyboardInterrupt, TimeoutError):
        global_logger.info("Shutting down due to external shutdown, keyboard interrupt or timeout")
    except Exception as e:
        global_logger.error(f"Error encountered: {e}")
    finally:        
        global_logger.info(f"Performing cleanup actions")
        
        try:
            # Retract ACF
            data_collector.publisher_force.publish(Float32Stamped(data=-10., header=Header()))
        except Exception as e:
            global_logger.info(f'Error encountered while retracting the ACF during shutdown: \n{e}')
        
        try:    
            # Turn off grinder and set the goal RPM to zero
            data_collector.plc.write_by_name(data_collector.grinder_on_var, False)
            data_collector.plc.write_by_name('', 0, pyads.PLCTYPE_REAL, handle=data_collector.rpm_control_handle)
            data_collector.release_handles()
            data_collector.plc.close()
        except Exception as e:
            global_logger.info(f'Error encountered while turning off the grinder and releasing handles during shutdown: \n{e}')
            
        
        data_collector.destroy_node()
        # Avoid stack trace 
        try:
            rclpy.shutdown()
        except rclpy._rclpy_pybind11.RCLError:
            pass 


if __name__ == '__main__':
    main()
    