import rclpy 
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from rcl_interfaces.msg import ParameterDescriptor 
from data_gathering_msgs.srv import TestRequest,  RequestPCL, RequestPCLVolumeDiff
from data_gathering_msgs.msg import BeltWearHistory
from std_msgs.msg import Empty, String 
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger 

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
        self.declare_parameter("contact_time_settings", [], ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("grit", 120)
        self.declare_parameter("sample", "")
        self.declare_parameter("plate_thickness", 0.)

        self.declare_parameter("wear_threshold", 10e6)
        self.declare_parameter("data_path", "")
        self.declare_parameter("wear_tracking_path", "")
        self.declare_parameter("test_tracker_path", "")
        self.declare_parameter("record_path", "")

        self.force_settings         = self.get_parameter("force_settings").value
        self.rpm_settings           = self.get_parameter("rpm_settings").value
        self.contact_time_settings  = self.get_parameter("contact_time_settings").value
        self.grit                   = self.get_parameter("grit").value
        self.sample_id              = self.get_parameter("sample").value
        self.plate_thickness        = self.get_parameter("plate_thickness").value 

        self.belt_threshold         = self.get_parameter("wear_threshold").value
        self.data_path              = pathlib.Path(self.get_parameter('data_path').value)
        self.belt_tracking_path     = pathlib.Path(self.get_parameter("wear_tracking_path").value)
        self.performed_tests_path   = pathlib.Path(self.get_parameter("test_tracker_path").value)
        self.record_path            = pathlib.Path(self.get_parameter("record_path").value)

        # Set default paths if any parameter is empty
        if not self.data_path or self.data_path == pathlib.Path('.'):
            self.data_path = pathlib.Path.cwd() / 'src' / 'data_gathering' / 'data'
        if not self.belt_tracking_path or self.belt_tracking_path == pathlib.Path('.'):
            self.belt_tracking_path = self.data_path / 'belt_tracking.csv'
        if not self.performed_tests_path or self.performed_tests_path == pathlib.Path('.'):
            self.performed_tests_path = self.data_path / 'performed_tests.csv'
        if not self.record_path or self.record_path == pathlib.Path('.'):
            self.record_path = self.data_path / 'test_data'

        self.test_setting_validity()
        self.settings = self.create_setting_list()
        self.test_index = 0
                
        self.test_client            = self.create_client(TestRequest, "execute_test")
        self.user_stop_testing      = self.create_subscription(Empty, "stop_testing", self.usr_stop_testing, 1)
        self.user_continue_testing  = self.create_subscription(Empty, "continue_testing", self.usr_continue_testing, 1)
        self.user_changed_belt      = self.create_subscription(Empty, "changed_belt", self.usr_changed_belt, 1)
        self.failure_publisher      = self.create_publisher(String, 'test_failure', 1)
        self.belt_wear_publisher    = self.create_publisher(BeltWearHistory, 'belt_wear_history', 1)

        self.scan_surface_trigger     = self.create_client(RequestPCL, 'execute_loop')      
        self.calculate_volume_trigger = self.create_client(RequestPCLVolumeDiff, 'calculate_volume_lost')   

        self.ready_for_next = True # Flag to see whether a test is in progress right now or the next test can be started on user input

        current_wear = self.read_initial_wear()
        if current_wear > self.belt_threshold:
            raise ValueError(f"The wear threshold {self.belt_threshold} has been exceeded ({current_wear}). Please change the belt.")

        self.rosbag = RosbagRecorder(['-a'],  str(self.record_path), 'rosbag2', self.get_logger())

        self.startup_delay = 5
        self.get_logger().info(f"Test coordinator started -- Executing first test in {self.startup_delay} seconds")
        self.test_start_countdown = self.create_timer(self.startup_delay, self.execute_test)

    ############################################################################################################################################
    # Main pipeline callback chain
    ############################################################################################################################################
    
    def execute_test(self):
        self.test_start_countdown.cancel()
        self.ready_for_next = False 
        self.rosbag.start_recording(self.generate_rosbag_suffix())

        # Publish the current wear history to store it in the rosbag 
        self.belt_wear_publisher.publish(self.create_wear_msg())

        # Perform an initial scan if this is the first test
        if self.test_index == 0:
            scan_call = self.scan_surface_trigger.call_async(RequestPCL.Request())
            scan_call.add_done_callback(self.initial_scan_done_callback)
        else:
            # On subsequent tests the 'initial' is the 'final' of the previous test
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


        
        if not success:
            self.get_logger().error("The test seems to have failed")
            self.failure_publisher.publish(String(data=result.message))  # Leave a message so the recording is marked as a failed test
            self.rosbag.stop_recording()
            return
        
        # Scan after the grind 
        scan_call = self.scan_surface_trigger.call_async(RequestPCL.Request())
        scan_call.add_done_callback(self.second_scan_done_callback)

        self.update_wear_history(result.force, result.rpm, result.contact_time)

    def second_scan_done_callback(self, future):
        self.final_scan = future.result().pointcloud
        
        # Request lost volume computation
        req = RequestPCLVolumeDiff.Request()
        req.initial_pointcloud  = self.initial_scan
        req.final_pointcloud    = self.final_scan
        req.plate_thickness     = self.plate_thickness
        path = self.write_pcl_pair(self.initial_scan, self.final_scan)

        volume_call = self.calculate_volume_trigger.call_async(req)
        volume_call.add_done_callback(partial(self.volume_calc_done_callback, path))

    def volume_calc_done_callback(self, data_path, future):
        result = future.result()
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
        else:
            self.ask_next_test()

    ############################################################################################################################################
    # Methods related to user inputs (chaning the belt and executing the next test)
    ############################################################################################################################################

    def ask_next_test(self):
        # Check whether the user wants to continue testing 
        remaining_tests = len(self.settings) - self.test_index
        self.get_logger().info(f"There are {remaining_tests} remaining tests to be run. Do you want to continue? - Use the 'continue_testing' topic'")        
        self.ready_for_next = True 

    def usr_changed_belt(self, _):
        # The user signaled that the belt has been changed
        # Change the tracked belt file. 
        self.belt_change()

        # The worn belt has been changed. Move on to the next test.
        self.ask_next_test()

    def usr_stop_testing(self, _):
        self.get_logger().info(f"The remaining tests will not be run. Exiting...")
    
    def usr_continue_testing(self, _):
        if self.ready_for_next:
            self.test_start_countdown = self.create_timer(self.startup_delay, self.execute_test)
            self.get_logger().info(f"Next test starting in {self.startup_delay} seconds.")
        else:
            self.get_logger().info("Not ready for the next test yet.")     

    ############################################################################################################################################
    # Methods related to tracking the belt wear 
    ############################################################################################################################################

    def update_wear_history(self, force, rpm, time):
        with open(self.belt_tracking_path, 'a') as f:
            f.write(f'{force},{rpm},{time}\n')
        with open(self.performed_tests_path, 'a') as f:
            f.write(f'{force},{rpm},{time}\n')

    def read_initial_wear(self):
        wear_csv = pd.read_csv(self.belt_tracking_path, delimiter=',')
        return self.wear_metric_calculation(wear_csv['force'], wear_csv['rpm'], wear_csv['contact_time'])
    
    def create_wear_msg(self):
        wear_csv = pd.read_csv(self.belt_tracking_path, delimiter=',')
        msg = BeltWearHistory()
        msg.force           = list(wear_csv["force"])
        msg.rpm             = list(wear_csv["rpm"])
        msg.contact_time    = list(wear_csv["contact_time"])
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
            f.write('force,rpm,contact_time\n')
            f.write('0,0,0\n')       

    def wear_metric_calculation(self, force, rpm, time):
        """
        This is a metric by which wear belt wear is measured, it does not have an exact physical meaning. 
        """
        return sum(force * rpm * time)

    ############################################################################################################################################
    # Methods related to setting the test parameters
    ############################################################################################################################################

    def create_setting_list(self):
        settings = []
        for i in range(max(len(self.force_settings), len(self.rpm_settings), len(self.contact_time_settings))):
            request = TestRequest.Request()
            request.force           = float(self.force_settings[i%len(self.force_settings)])
            request.rpm             = float(self.rpm_settings[i%len(self.rpm_settings)])
            request.contact_time    = float(self.contact_time_settings[i%len(self.contact_time_settings)])
            settings.append(request)
        return settings 

    def test_setting_validity(self):
        if len(self.force_settings) == 0 or len(self.rpm_settings) == 0 or len(self.contact_time_settings) == 0:
            raise ValueError("A value must be specified for 'force_settings', 'rpm_settings' and 'contact_time_settings'")

        _unique_settings = set([len(self.force_settings), len(self.rpm_settings), len(self.contact_time_settings)])
        _unique_settings.discard(1)
        if len(_unique_settings) > 2:
            raise ValueError("All setting lists must be either the same length, or length 1 to keep the setting constant")

        negatives = [setting for setting in [*self.force_settings, *self.rpm_settings, *self.contact_time_settings] if setting < 0]
        if len(negatives) > 0:
            raise ValueError("Force, RPM and contact time should be positive")


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

    rclpy.spin(test_coordinator, executor=executor)
    test_coordinator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    