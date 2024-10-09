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
import math


class DataCollector(Node):

    def __init__(self) -> None:
        super().__init__('data_collector')
        self.init_parameters()
        self.connect_plc()
        self.init_handles()
                
        # Setup ROS interfaces 
        self.publisher_force     = self.create_publisher(Float32Stamped, '/acf/force', 10)
        self.publisher_rpm       = self.create_publisher(Int32Stamped, '/grinder/rpm', 10)
        self.publisher_rpm_req   = self.create_publisher(Int32Stamped, '/grinder/requested_rpm', 10)
        self.publisher_time_sync = self.create_publisher(TimeSync, '/timesync', 10)
        self.telem_listener      = self.create_subscription(ACFTelemStamped, '/acf/telem', self.telem_callback, 1)
        self.test_server         = self.create_service(TestRequest, 'execute_test', self.start_test, callback_group=MutuallyExclusiveCallbackGroup())

        # Variable is set to True in `shutdown_sequence`. This indicates that the service callback self.start_test can return a finished response to the coordinator. 
        self._test_done = False 
        # Variable is set to True if the grinder is shutdown after the maximum runtime is exceeded. Not the most robust metric but temporary solution.
        self._test_success = None
        self._failure_message = ''

        # Timer for publishing TimeSync messages. This is set to a timer to ensure that there will be a usable message in the rosbag.
        # A message may not contain useful data if the PLC was not properly connected before creating a message 
        self.time_sync_timer     = self.create_timer(1, self.sync_callback)

        # Get notifications from the PLC 
        atr = pyads.NotificationAttrib(ctypes.sizeof(ctypes.c_uint32))
        self.rpm_notification_handle = self.plc.add_device_notification("GVL_Var.Actual_RPM", atr, self.rpm_callback)
               
        # Retract ACF before startup
        self.publisher_force.publish(Float32Stamped(header=Header(), data=-5.))
        
        # Set desired RPM to zero, turn off the grinder and retract acf
        self.plc.write_by_name(self.rpm_control_var, 0)
        self.plc.write_by_name(self.grinder_on_var, False)
        self.force_desired = -5

        self.test_running = False   # Switch ingores telem data if not testing
        
        self.get_logger().info('DataCollector initialised')
        if not self.grinder_enabled:
            self.get_logger().warn('\n\n\n WARNING: the grinder is turned off - set "grinder_enabled" to "True" to turn it on \n\n')

    def init_parameters(self) -> None:
        self.declare_parameter('plc_target_ams',     '5.149.234.177.1.1')    # AMS ID of the PLC
        self.declare_parameter('plc_target_ip',      '169.254.200.16')       # IP of the PLC
         
        self.declare_parameter('timer_period',       '0.008')                # Time in seconds
        # self.declare_parameter('max_contact_time',   '5.')                   # Time to grind in seconds
        self.declare_parameter('timeout_time',       '20.')                  # Duration before timeout in seconds
        self.declare_parameter('time_before_extend', '3.')                   # Duration after startup before extending the acf to allow for belt spin up
        
        # self.declare_parameter('force_desired',      '10.')                  # ACF force in newtons
        self.declare_parameter('rpm_control_var',    'Flow_Control_valve.iScaledDesiredFlowRate') 
        # self.declare_parameter('desired_flowrate_scaled', '40')              # Percentage of flowrate to control the grinder RPM
        

        # Maybe this should change to main.bOnOff but it will currently be overwritten by the HMI interface
        self.declare_parameter('grinder_on_var',    'HMI.bOnOffPB')         # PLC variable controlling the on/off switch of the grinder 
        self.declare_parameter('grinder_enabled',    False)                 # Turn the grinder on/off with True/False
        self.declare_parameter('time_var',          'GVL_Var.sTimeSt')      # PLC variable for timestamp

        self.declare_parameter('max_acf_extension', '0.3')                  # The maximum extension that can be reached by the ACF
                                       
        self.target_ams_plc             = self.get_parameter('plc_target_ams').get_parameter_value().string_value
        self.target_ip_plc              = self.get_parameter('plc_target_ip').get_parameter_value().string_value
        
        self.timer_period               = float(self.get_parameter('timer_period').get_parameter_value().string_value)
        # self.contact_time               = float(self.get_parameter('max_contact_time').get_parameter_value().string_value)
        # self.force_desired              = float(self.get_parameter('force_desired').get_parameter_value().string_value)   
        self.timeout_time               = float(self.get_parameter('timeout_time').get_parameter_value().string_value)                           
        self.startup_time               = float(self.get_parameter('time_before_extend').get_parameter_value().string_value)         
        # self.desired_flowrate_scaled    = float(self.get_parameter('desired_flowrate_scaled').get_parameter_value().string_value)
        
        self.rpm_control_var            = self.get_parameter('rpm_control_var').get_parameter_value().string_value  
        self.grinder_on_var             = self.get_parameter('grinder_on_var').get_parameter_value().string_value   
        self.grinder_enabled            = self.get_parameter('grinder_enabled').get_parameter_value().bool_value
        self.get_logger().info(f'Grinder var: {self.grinder_enabled}')
        self.time_var                   = self.get_parameter('time_var').get_parameter_value().string_value   
        
        self.max_acf_extension          = float(self.get_parameter('max_acf_extension').get_parameter_value().string_value)

        self.initial_contact_time       = None # Time at which initial contact was made (None until contact) 
        # self.max_contact_time           = rclpy.duration.Duration(seconds=self.contact_time)
        self.timeout_duration           = rclpy.duration.Duration(seconds=self.timeout_time)
        self.spin_up_duration           = self.startup_time
    
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

    def start_test(self, request, response):
        self.test_running = True 
        self.force_desired = request.force 
        self.desired_flowrate_scaled = self.rpm_to_flowrate(request.rpm)
        self.max_contact_time = rclpy.duration.Duration(seconds=math.floor(request.contact_time), 
                                                        nanoseconds=(request.contact_time % math.floor(request.contact_time)/10**9))

        self.get_logger().info(f"Settings received:\n  Force: {self.force_desired} N\n  RPM: {request.rpm}\n  Duration: {request.contact_time} seconds")

        self.init_time = self.get_clock().now() 
        response.force = request.force
        response.rpm = request.rpm
        response.contact_time = request.contact_time 

        # Turn on grinder 
        self.publisher_rpm_req.publish(Int32Stamped(data=int(request.rpm), header=Header(stamp=self.get_clock().now().to_msg())))
        self.plc.write_by_name(self.grinder_on_var, self.grinder_enabled)
        self.plc.write_by_name('', self.desired_flowrate_scaled, pyads.PLCTYPE_REAL, handle=self.rpm_control_handle)

        # Start after spin up duration
        self.spin_up_wait = self.create_timer(self.spin_up_duration, self.spin_up_period_done)

        wait_for_finish_rate = self.create_rate(1)
        # Hang the response until the test is done as indicated by a shutdown_sequence call
        while not self._test_done:
            wait_for_finish_rate.sleep()
        self._test_done = False 
        self.test_running = False # TODO these two vars do something similar, could be combined

        response.success = self._test_success
        response.message = self._failure_message
        self._test_success = None  
        self._failure_message = ''
        return response
    
    def spin_up_period_done(self):
        self.spin_up_wait.cancel()
        # Spin up duration over - start the main loop
        # self.timer = self.create_timer(self.timer_period, self.timer_callback) # TODO To be removed
        self.test_timed_out_timer = self.create_timer(self.timeout_duration.nanoseconds/10**9, self.timeout)

        # Engage the grinder 
        header = Header() 
        header.stamp = self.get_clock().now().to_msg()
        force = Float32Stamped(header=header, data=self.force_desired)
        self.publisher_force.publish(force)
        
    
    def timer_callback(self) -> None:
        ...
        # time = self.get_clock().now()
    
        # # Turn off if the maximum contact time has been exceeded 
        # # if self.initial_contact_time is not None and self.initial_contact_time + self.max_contact_time <= time:
        # #     self._test_success = True 
        # #     self.get_logger().info(f'Grind time of {self.max_contact_time} exceeded')
        # #     self.shutdown_sequence()
            
        # # Turn off if the maximum runtime for timeout had been exceeded (in case there was no contact )
        # if self.init_time + self.timeout < time:
        #     self.get_logger().info(f"Maximum runtime of {self.timeout_time} seconds exceeded")
        #     self.shutdown_sequence()
        #     raise TimeoutError("Maximum runtime exceeded")
        
        # # Send command to the acf if past spin up time 
        # # Even if the force stays the same, keep publishing force messages otherwise the acf node doesnt publish telemetry
        # # Could by changed by setting the ACF to a fixed frequency, but this would also only update force settings at this frequency.
        # # if time - self.init_time >= self.spin_up_duration:
        # header = Header() 
        # header.stamp = self.get_clock().now().to_msg()
        # force = Float32Stamped(header=header, data=self.force_desired)
        # self.publisher_force.publish(force)
        
        # # Send command to the plc - TODO this is probably not required - the desired rpm should stay the same 
        # # self.plc.write_by_name('', self.desired_flowrate_scaled, pyads.PLCTYPE_UINT, handle=self.rpm_control_handle)
        # self.plc.write_by_name('', self.desired_flowrate_scaled, pyads.PLCTYPE_REAL, handle=self.rpm_control_handle)

    def contact_time_exceeded(self):
        self.test_finished_timer.cancel()   # TODO to be removed
        self.initial_contact_time = None        # Reset for next test 

        if self._test_succes is None:
            self._test_success = True               # Set success flag 
        self.get_logger().info(f'Grind time of {self.max_contact_time} exceeded')
        self.shutdown_sequence()

    def timeout(self):
        self.test_timed_out_timer.cancel()
        self.initial_contact_time = None        # Reset for next test 
        self._test_succes = False   
        self._failure_message += f'Test timed out. Duration of {self.max_contact_time} exceeded. Likely no contact was detected or the desired force was never reached'
        self.get_logger().info(f'Test timed out. Duration of {self.max_contact_time} exceeded. Likely no contact was detected or the desired force was never reached')
        self.shutdown_sequence()

    def shutdown_sequence(self) -> None:
        self.get_logger().info(f"Turning off")
        self.test_timed_out_timer.cancel()

        # Retract the acf
        # self.force_desired = -5.
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        force = Float32Stamped(header=header, data=-5.)
        self.publisher_force.publish(force)
        
        # Stop the grinder
        self.plc.write_by_name(self.grinder_on_var, False)
        self.desired_flowrate_scaled = 0 
        # self.plc.write_by_name('', self.desired_flowrate_scaled, pyads.PLCTYPE_UINT, handle=self.rpm_control_handle)
        self.plc.write_by_name('', 0, pyads.PLCTYPE_REAL, handle=self.rpm_control_handle)
        
        # Stop the command timer 
        # self.timer.cancel() # TODO to be removed 
        self._test_done = True 
        
    def rpm_callback(self, notification, data) -> None:
        # self.get_logger(f'Notification type: {type(notification)} datatype: {type(data)}')
        # This publisher only exists to publish the rpm notifications so it is recorded in the rosbag
        handle, timestamp, value = self.plc.parse_notification(notification, pyads.PLCTYPE_UDINT)
        
        header = Header(stamp=self.datetime_to_msg(timestamp))
        self.publisher_rpm.publish(Int32Stamped(data=value, header=header))
    
    def telem_callback(self, msg:ACFTelemStamped):
        if not self.test_running:
            return 

        # Set the initial contact time if a force within 10% of the desired force is reached
        force_abs_relative_error = abs((self.force_desired - msg.telemetry.force) / (self.force_desired + 1e-3)) # Added 1 mN in divisor to avoid div by zero
        if self.initial_contact_time is None and msg.telemetry.in_contact and (force_abs_relative_error < 0.1):
            self.initial_contact_time = self.get_clock().now()

            # Shutdown after contact duration is reached
            self.test_finished_timer = self.create_timer(self.max_contact_time.nanoseconds/10**9, self.contact_time_exceeded)

        # If an acf extension within 2% of its max is reached, signal that the test may have failed 
        if abs(self.max_acf_extension - msg.telemetry.position) / self.max_acf_extension < 0.02:
            self._test_succes = False
            self._failure_message += '\nThe maximum extension of the ACF was reached. This means force may not have been maintained.'
            

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
    
    def rpm_to_flowrate(self, rpm):
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
            data_collector.plc.write_by_name(data_collector.rpm_control_handle, 0)
            data_collector.release_handles()
            data_collector.plc.close()
        except Exception as e:
            global_logger.info(f'Error encountered while turning off the grinder and releasing handles during shutdown: \n{e}')
            
        
        data_collector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    