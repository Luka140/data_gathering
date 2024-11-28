import rclpy 
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from rcl_interfaces.msg import ParameterDescriptor 
from data_gathering_msgs.srv import TestRequest,  RequestPCL, RequestPCLVolumeDiff
from data_gathering_msgs.msg import BeltWearHistory, GrindArea
from std_msgs.msg import Empty, String, Header
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger 
from stamped_std_msgs.msg import Float32Stamped

import copy
import os 
import pathlib 
from datetime import datetime
import pandas as pd 
import numpy as np
import open3d as o3d
from functools import partial

from data_gathering.rosbag_controller import RosbagRecorder


class TestCoordinator(Node):

    def __init__(self):
        super().__init__('test_coordinator')
        self.declare_parameter("force_settings",        [], ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("rpm_settings",          [], ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("pass_count_settings",   [], ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("feed_rate_settings",    [], ParameterDescriptor(dynamic_typing=True))

        self.declare_parameter("grit", 120)
        self.declare_parameter("sample", "")
        self.declare_parameter("plate_thickness", 0.)
        self.declare_parameter("belt_width", 0.)
        self.declare_parameter("movement_length", 0.)
        self.declare_parameter("feed_rate_threshold", 10)

        self.declare_parameter("belt_prime_force",  3,      ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("belt_prime_rpm",    9000,   ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("belt_prime_time",   5,      ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("initial_prime", False)

        self.declare_parameter("repeat_test_count", 1)

        self.declare_parameter("wear_threshold", 10e6)
        self.declare_parameter("data_path", "")
        self.declare_parameter("wear_tracking_path", "")
        self.declare_parameter("test_tracker_path", "")
        self.declare_parameter("record_path", "")

        self.declare_parameter("recorded_topics", ['-a'])

        self.force_settings         = self.get_parameter("force_settings").value
        self.rpm_settings           = self.get_parameter("rpm_settings").value
        self.pass_count_settings    = self.get_parameter("pass_count_settings").value
        self.feed_rate_settings     = self.get_parameter("feed_rate_settings").value
        self.grit                   = self.get_parameter("grit").value
        self.sample_id              = self.get_parameter("sample").value
        self.plate_thickness        = self.get_parameter("plate_thickness").value 
        self.belt_width             = self.get_parameter("belt_width").value 
        self.movement_length        = self.get_parameter("movement_length").value 
        self.feed_rate_threshold    = self.get_parameter("feed_rate_threshold").value 

        belt_prime_force    = self.get_parameter("belt_prime_force").value
        belt_prime_rpm      = self.get_parameter("belt_prime_rpm").value
        belt_prime_time     = self.get_parameter("belt_prime_time").value
        initial_prime_belt  = self.get_parameter("initial_prime").value 

        # Repeat a test a number of times, and only afterwards scan again. Then divide volume loss by nr. of tests
        # Allows to detect smaller volume loss numbers reliably 
        self.repeat_test_count = self.get_parameter("repeat_test_count").value 

        self.belt_threshold         = self.get_parameter("wear_threshold").value
        self.data_path              = pathlib.Path(self.get_parameter('data_path').value)
        self.belt_tracking_path     = pathlib.Path(self.get_parameter("wear_tracking_path").value)
        self.performed_tests_path   = pathlib.Path(self.get_parameter("test_tracker_path").value)
        self.record_path            = pathlib.Path(self.get_parameter("record_path").value)

        self.recorded_topics        = self.get_parameter("recorded_topics").value 

        # Set default paths if any parameter is empty
        if not self.data_path or self.data_path == pathlib.Path('.'):
            self.data_path = pathlib.Path.cwd() / 'src' / 'data_gathering' / 'data'
        if not self.belt_tracking_path or self.belt_tracking_path == pathlib.Path('.'):
            self.belt_tracking_path = self.data_path / 'belt_tracking.csv'
        if not self.performed_tests_path or self.performed_tests_path == pathlib.Path('.'):
            self.performed_tests_path = self.data_path / 'performed_tests.csv'
        if not self.record_path or self.record_path == pathlib.Path('.'):
            self.record_path = self.data_path / 'test_data'
        
        directories = [self.data_path, self.record_path]
        for directory in directories:
            if not os.path.isdir(directory):
                os.mkdir(directory)

        self.test_setting_validity()
        self.settings = self.create_setting_list(self.force_settings, self.rpm_settings, self.contact_time_settings)
        self.test_index = 0         # Index of the current test in the settings list 
        self.sub_test_index = 0     # Number of times a certain test has been repeated

        self.belt_prime_settings = self.create_setting_list([belt_prime_force], [belt_prime_rpm], [belt_prime_time])[0]
                
        self.test_client            = self.create_client(TestRequest, "data_collector/execute_test")

        # Empty subscriptions for user input to trigger certain actions 
        self.user_stop_testing      = self.create_subscription(Empty, "user/stop_testing", self.usr_stop_testing, 1, callback_group=MutuallyExclusiveCallbackGroup())
        self.user_continue_testing  = self.create_subscription(Empty, "user/continue_testing", self.usr_continue_testing, 1, callback_group=MutuallyExclusiveCallbackGroup())
        self.user_changed_belt      = self.create_subscription(Empty, "user/changed_belt", self.usr_changed_belt, 1, callback_group=MutuallyExclusiveCallbackGroup())

        self.failure_publisher      = self.create_publisher(String, '~/test_failure', 1)                  # Publish failure message for logging 
        self.belt_wear_publisher    = self.create_publisher(BeltWearHistory, '~/belt_wear_history', 1)    # Publish belt wear for logging 
        self.grind_area_publisher   = self.create_publisher(GrindArea, "~/grind_area", 1)                 # Publish belt width and plate thickness for logging 
        self.publisher_volume       = self.create_publisher(Float32Stamped, '~/volume', 1)       # Publish removed volume 

        self.scan_surface_trigger     = self.create_client(RequestPCL, 'execute_loop')                      # Request a scan of the test object
        self.calculate_volume_trigger = self.create_client(RequestPCLVolumeDiff, 'calculate_volume_lost')   # Request calculation of the removed volume 

        self.ready_for_next = True  # Flag to see whether a test is in progress right now or the next test can be started on user input
        self.primed_belt = False    # Flag to indicate that a new belt was just primed, so a new 'before' scan should be made 
        self.belt_worn = False      # Flag to indicate that the belt should be changed - unlocks the self.usr_changed_belt callback

        current_wear = self.read_initial_wear()
        if current_wear > self.belt_threshold:
            raise ValueError(f"The wear threshold {self.belt_threshold} has been exceeded ({current_wear}). Please change the belt.")

        self.rosbag = RosbagRecorder(self.recorded_topics,  str(self.record_path), 'rosbag2', self.get_logger())

        # Wait time in between tests 
        self.startup_delay = 5

        if not initial_prime_belt:
            self.get_logger().info(f"Test coordinator started -- Executing first test in {self.startup_delay} seconds")
            self.test_start_countdown = self.create_timer(self.startup_delay, self.execute_test)
        else:
            # First do a prime run on the belt
            self.get_logger().info(f"Test coordinator started -- Priming belt in {self.startup_delay} seconds")
            self.prime_belt_countdown = self.create_timer(self.startup_delay, self.prime_belt)

    ############################################################################################################################################
    # Main pipeline callback chain
    ############################################################################################################################################

    def start_rosbag(self, extra_suffix=''):
        recording_started, msg = self.rosbag.start_recording(self.generate_rosbag_suffix() + extra_suffix)
        if not recording_started:
            self.get_logger().error(f"Cancelling the test: {msg}")
            return 
        
        # Wait until rosbag is ready
        rosbag_ready, rosbag_msg = self.rosbag.wait_for_rosbag(timeout_sec=10.0)
        if not rosbag_ready:
            self.get_logger().error(f"Cancelling the test: {rosbag_msg}")
            self.rosbag.stop_recording()
            return 
        else:
            self.get_logger().info("Rosbag recording is ready.")
    
    def prime_belt(self):
        self.prime_belt_countdown.cancel()
        self.ready_for_next = False 

        self.start_rosbag('_BELT_PRIME')
    
        # Perform prime grind 
        request = self.belt_prime_settings
        call = self.test_client.call_async(request)
        call.add_done_callback(self.prime_finished_callback)
    
    def prime_finished_callback(self, future):
        self.get_logger().info("Finished priming belt")
        result = future.result()
        success = result.success
        self.primed_belt = True # Indicate that a new 'before' scan should be made 

        if not success:
            self.get_logger().error(f"\n\n\nThe belt prime seems to have failed due to:\n{result.message}\n\n")
            self.failure_publisher.publish(String(data=result.message))  # Leave a message so the recording is marked as a failed test
        
        self.update_wear_history(result.force, result.rpm, result.contact_time, self.belt_width * self.plate_thickness)

        self.rosbag.stop_recording()
        self.ask_next_test()       

    def execute_test(self):
        self.test_start_countdown.cancel()
        self.ready_for_next = False 
        
        self.start_rosbag()

        # Publish the current wear history in one second to ensure it is not lost while the rosbag is starting up 
        self.belt_pub_timer = self.create_timer(2, self.pub_wear)
        
        # Perform an initial scan if this is the first test or if the belt was just primed
        if (self.test_index == 0 and self.sub_test_index == 0) or self.primed_belt:
            self.primed_belt = False 
            scan_call = self.scan_surface_trigger.call_async(RequestPCL.Request())
            scan_call.add_done_callback(self.initial_scan_done_callback)
            return 
        
        elif self.test_index != 0:
            # On subsequent tests the 'initial' is the 'final' of the previous test
            # But not on the subsequent subtests of test 0 
            self.initial_scan = self.final_scan 

        # Perform grind 
        self.call_test()

    def initial_scan_done_callback(self, future):
        result = future.result()
        success = result.success
        self.initial_scan = result.pointcloud
        if success:
            # Perform grind 
            self.call_test()
        else:
            self.get_logger().error("Scan service failed...")

    def call_test(self):
        # Perform grind 
        request = self.settings[self.test_index]
        call = self.test_client.call_async(request)
        call.add_done_callback(self.test_finished_callback)

    def test_finished_callback(self, future):
        result = future.result()
        success = result.success

        # Update how many times the current test has been ran
        self.sub_test_index += 1 
        
        self.update_wear_history(result.force, result.rpm, result.contact_time, self.plate_thickness * self.belt_width)

        if not success:
            self.get_logger().error(f"\n\nThe test seems to have failed due to:\n{result.message}\n")
            self.failure_publisher.publish(String(data=result.message))  # Leave a message so the recording is marked as a failed test

        if self.sub_test_index < self.repeat_test_count:
            ... # TODO bypass second scan but still do all the cleanup stuff like stopping the recording ...
            self.rosbag.stop_recording()
            self.ask_next_test()
        
        else:
            # All subtests performed - move on to the scan
            self.sub_test_index = 0 

            self.get_logger().info("Performing after test scan...")
            # Scan after the grind 
            scan_call = self.scan_surface_trigger.call_async(RequestPCL.Request())
            scan_call.add_done_callback(self.second_scan_done_callback)


    def second_scan_done_callback(self, future):
        self.final_scan = future.result().pointcloud
        
        # Request lost volume computation
        req = RequestPCLVolumeDiff.Request()
        req.initial_pointcloud  = self.initial_scan
        req.final_pointcloud    = self.final_scan
        req.plate_thickness     = self.plate_thickness
        req.belt_width          = self.belt_width
        path = self.write_pcl_pair(self.initial_scan, self.final_scan)

        volume_call = self.calculate_volume_trigger.call_async(req)
        volume_call.add_done_callback(partial(self.volume_calc_done_callback, path))

    def volume_calc_done_callback(self, data_path, future):
        result = future.result()
        success = result.success

        # Convert volume to cubic mm and divide by the number of subtests that were performed without scanning
        vol_per_grind = float(result.volume_difference) * 1000**3 / self.repeat_test_count
        volume_msg = Float32Stamped(data= vol_per_grind, header=Header(stamp=self.get_clock().now().to_msg()))
        self.get_logger().info(f"Removed volume per grind: {vol_per_grind} mm3")
        self.publisher_volume.publish(volume_msg)

        if not success:
            self.get_logger().error(f"\n\nThe volume calculation seems to have failed due to:\n{result.message}\n")
            self.failure_publisher.publish(String(data=result.message))  # Leave a message so the recording is marked as a failed test
        

        self.write_pcl(result.difference_pointcloud, data_path / 'difference_pcl.ply')
        self.rosbag.stop_recording()
        
        self.test_index += 1     
        if self.test_index >= len(self.settings):
            self.get_logger().info("All queued tests have been executed")
            return 
               
        # Check whether the belt is worn down. If so - wait for replacement
        current_wear = self.read_initial_wear()
        if current_wear > self.belt_threshold:
            self.get_logger().info(f"The wear metric ({current_wear}) has exceeded the threshold of ({self.belt_threshold}). Please change the belt")
            self.belt_worn = True 
        else:
            self.ask_next_test()

    ############################################################################################################################################
    # Methods related to user inputs (chaning the belt and executing the next test)
    ############################################################################################################################################

    def ask_next_test(self):
        # Check whether the user wants to continue testing 
        remaining_tests = len(self.settings) - self.test_index
        self.get_logger().info(f"There are {remaining_tests} remaining tests to be run with {self.repeat_test_count} subtests each. Do you want to continue? - Use the 'continue_testing' topic'")        
        self.ready_for_next = True 

    def usr_changed_belt(self, _):
        # The user signaled that the belt has been changed
        # Change the tracked belt file. 
        if self.belt_worn:
            self.belt_worn = False 
            self.belt_change()

            # Prime the next belt 
            self.get_logger().info(f"Belt changed. Priming belt in {self.startup_delay} seconds")
            self.prime_belt_countdown = self.create_timer(self.startup_delay, self.prime_belt)    

    def usr_stop_testing(self, _):
        self.get_logger().info(f"The remaining tests will not be run. Exiting...")
    
    def usr_continue_testing(self, _):
        if self.ready_for_next:
            self.ready_for_next = False 
            self.test_start_countdown = self.create_timer(self.startup_delay, self.execute_test)
            self.get_logger().info(f"Next test starting in {self.startup_delay} seconds.")
        else:
            self.get_logger().info("Not ready for the next test yet.")     

    ############################################################################################################################################
    # Methods related to tracking the belt wear 
    ############################################################################################################################################

    def pub_wear(self):
        self.belt_pub_timer.cancel()
        self.belt_wear_publisher.publish(self.create_wear_msg())
        self.grind_area_publisher.publish(GrindArea(belt_width=self.belt_width, plate_thickness=self.plate_thickness))

    def update_wear_history(self, force, rpm, time, area):
        with open(self.belt_tracking_path, 'a') as f:
            f.write(f'{force},{rpm},{time},{area}\n')
        with open(self.performed_tests_path, 'a') as f:
            f.write(f'{force},{rpm},{time}\n')

    def read_initial_wear(self):
        wear_csv = pd.read_csv(self.belt_tracking_path, delimiter=',')
        return self.wear_metric_calculation(wear_csv['force'], wear_csv['rpm'], wear_csv['contact_time'])
    
    def create_wear_msg(self):
        wear_csv = pd.read_csv(self.belt_tracking_path, delimiter=',')
        msg = BeltWearHistory()
        msg.force           = list(wear_csv["force"].astype(float))
        msg.rpm             = list(wear_csv["rpm"].astype(float))
        msg.contact_time    = list(wear_csv["contact_time"].astype(float))
        msg.area            = list(wear_csv["area"].astype(float))
        msg.tracked_file    = str(self.belt_tracking_path)
        return msg 
        
    def belt_change(self):
        """
        Method to create a new belt wear tracing file once the belt is replaced.
        The file name format should be beltid_{id}_grit_{gritsize}.csv
        For example: beltid_5_grit_120.csv 
        """
        self.get_logger().info("Creating new belt wear tracking file,,,")
        current_belt_name_parts = self.belt_tracking_path.name.strip('.csv').split('_')
        new_belt_name_parts = copy.deepcopy(current_belt_name_parts)
        new_belt_name_parts[current_belt_name_parts.index('beltid') + 1] = str(int(new_belt_name_parts[current_belt_name_parts.index('beltid') + 1]) + 1)
        new_path = self.belt_tracking_path.parent / f"{'_'.join(new_belt_name_parts)}.csv" 

        if os.path.isfile(new_path):
            self.get_logger().info(f'The auto generated filename {new_path.name} already exists - generating a timestamped name instead...')
            new_path = self.belt_tracking_path.parent / f"belt_beltid_{datetime.now()}.csv"
        self.get_logger().info(f'Set the belt history path to {new_path}')

        self.belt_tracking_path = new_path
        if os.path.isfile(self.belt_tracking_path):
            # Ensure that no existing file is overwritten 
            raise ValueError(f"The autogenerated filepath {self.belt_tracking_path} already exists. ")

        with open(self.belt_tracking_path, "w") as f:
            f.write('force,rpm,contact_time,area\n')
            f.write('0.0,0.0,0.0,0.0\n')       

    def wear_metric_calculation(self, force, rpm, time):
        """
        This is the metric by which wear belt wear is measured 
        """
        return sum(force * rpm * time)

    ############################################################################################################################################
    # Methods related to setting the test parameters
    ############################################################################################################################################

    def create_setting_list(self, forces, rpms, contact_times):
        settings = []
        for i in range(max(len(forces), len(rpms), len(contact_times))):
            request                 = TestRequest.Request()
            request.force           = float(forces[i%len(forces)])
            request.rpm             = float(rpms[i%len(rpms)])
            request.contact_time    = float(contact_times[i%len(contact_times)])
            settings.append(request)
        return settings 

    def test_setting_validity(self):
        if len(self.force_settings) == 0 or len(self.rpm_settings) == 0 or len(self.pass_count_settings) or len(self.feed_rate_settings) == 0:
            raise ValueError("A value must be specified for 'force_settings', 'rpm_settings', 'pass_count_settings' and 'feed_rate_settings'")

        _unique_settings = set([len(self.force_settings), len(self.rpm_settings), len(self.contact_time_settings)])
        _unique_settings.discard(1)
        if len(_unique_settings) > 2:
            raise ValueError("All setting lists must be either the same length, or length 1 to keep the setting constant")

        negatives = [setting for setting in [*self.force_settings, *self.rpm_settings, *self.contact_time_settings] if setting < 0]
        if len(negatives) > 0:
            raise ValueError("Force, RPM and contact time should be positive")
        
        if max(self.feed_rate_settings) > self.feed_rate_threshold:
            raise ValueError("Feedrate above threshold")


    ############################################################################################################################################
    #   Methods related to storing the data
    ############################################################################################################################################

    def generate_rosbag_suffix(self):
        test_settings = self.settings[self.test_index]
        return f'_sample{self.sample_id}__f{test_settings.force}_rpm{test_settings.rpm}_grit{self.grit}_t{test_settings.contact_time}'

    def convert_ros_to_open3d(self, pcl_msg):
        # Extract the point cloud data from the ROS2 message
        
        loaded_array = np.frombuffer(pcl_msg.data, dtype=np.float32).reshape(-1, 3) 
        o3d_pcl = o3d.geometry.PointCloud()
        o3d_pcl.points = o3d.utility.Vector3dVector(loaded_array)
        return o3d_pcl

    def write_pcl(self, pointcloud: PointCloud2, path):
        pcl = self.convert_ros_to_open3d(pointcloud)
        if pcl.is_empty():
            pathtxt = str(path).strip('ply') + 'txt'
            with open(pathtxt, 'w') as f:
                f.write('Empty pointcloud')
        else:
            o3d.io.write_point_cloud(str(path), pcl)

    def write_pcl_pair(self, pre_grind: PointCloud2, post_grind: PointCloud2):
        current_time = datetime.now()
        timestamp = f'{str(current_time.date()).strip()}_{str(current_time.time()).strip().split(".")[0]}'
        path = self.record_path / f'pcl_{self.generate_rosbag_suffix()}_{timestamp}'
        if not path.exists():
            os.mkdir(path)

        # Write initial point cloud
        pcl_initial_path = path /'pre_grind_pointcloud.ply'
        self.get_logger().info(f"Saving initial pointcloud to: {pcl_initial_path}")
        self.write_pcl(pre_grind, pcl_initial_path)
        
        # Write postgrind point cloud
        pcl_postgrind_path = path / f'post_grind_pointcloud.ply'
        self.get_logger().info(f"Saving postgrind pointcloud to: {pcl_postgrind_path}")
        self.write_pcl(post_grind, pcl_postgrind_path)
        
        return path
    
def main(args=None):
    rclpy.init(args=args)

    test_coordinator = TestCoordinator()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(test_coordinator, executor=executor)
    except KeyboardInterrupt:
        pass 
    test_coordinator.destroy_node()
    
    # Avoid stack trace 
    try:
        rclpy.shutdown()
    except rclpy._rclpy_pybind11.RCLError:
        pass 


if __name__ == '__main__':
    main()
    