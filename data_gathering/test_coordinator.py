import rclpy 
from rclpy.node import Node
from rclpy.impl import rcutils_logger
from rclpy.executors import MultiThreadedExecutor

from rcl_interfaces.msg import ParameterDescriptor 
from data_gathering_msgs.srv import TestRequest 
from std_msgs.msg import Empty

import copy
import os 
import pathlib 
from datetime import datetime
import pandas as pd 

from data_gathering.rosbag_controller import RosbagRecorder

class TestCoordinator(Node):

    def __init__(self):
        super().__init__('test_coordinator')
        self.declare_parameter("force_settings",        [], ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("rpm_settings",          [], ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("contact_time_settings", [], ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("grit", 120)
        self.declare_parameter("sample", "")

        self.declare_parameter("wear_threshold", 10e6)
        self.declare_parameter("wear_tracking_file", "")

        self.force_settings         = self.get_parameter("force_settings").value
        self.rpm_settings           = self.get_parameter("rpm_settings").value
        self.contact_time_settings  = self.get_parameter("contact_time_settings").value
        self.grit                   = self.get_parameter("grit").value
        self.sample_id              = self.get_parameter("sample").value

        self.belt_threshold         = self.get_parameter("wear_threshold").value
        self.belt_tracking_path     = pathlib.Path(self.get_parameter("wear_tracking_file").value)

        self.test_setting_validity()
        self.settings = self.create_setting_list()
        self.test_index = 0
                
        self.test_client            = self.create_client(TestRequest, "execute_test")
        self.user_stop_testing      = self.create_subscription(Empty, "stop_testing", self.usr_stop_testing, 1)
        self.user_continue_testing  = self.create_subscription(Empty, "continue_testing", self.usr_continue_testing, 1)
        self.user_changed_belt      = self.create_subscription(Empty, "changed_belt", self.usr_changed_belt, 1)
        self.failure_publisher      = self.create_publisher(Empty, 'test_failure', 1)

        # Variables to be changed by user inputs 
        self._run_next = None
        self._belt_replaced = False
        self.user_input_check_rate = self.create_rate(1, self.get_clock())

        current_wear = self.read_initial_wear()
        if current_wear > self.belt_threshold:
            self.wait_for_belt_change(current_wear)

        self.rosbag = RosbagRecorder(['-a'],  'ros_bags', 'rosbag2', self.get_logger())

        self.startup_delay = 10
        self.get_logger().info(f"Test coordinator started -- Executing first test in {self.startup_delay} seconds")
        self.test_start_countdown = self.create_timer(self.startup_delay, self.execute_test)


    def execute_test(self):
        self.test_start_countdown.cancel()
        self.rosbag.start_recording(self.generate_rosbag_suffix())

        # TODO Request initial scan of the object 

        request = self.settings[self.test_index]
        call = self.test_client.call_async(request)
        call.add_done_callback(self.test_finished_callback)

    def test_finished_callback(self, result):
        success = result.success
        
        if not success:
            self.get_logger().error("The test seems to have failed")
            self.failure_publisher
            self.rosbag.stop_recording()
            return
        
        # TODO request callback from the LLS to calculate lost volume 
        
        self.rosbag.stop_recording()
        
        self.update_wear_history(result.force, result.rpm, result.contact_time)

        self.test_index += 1     
        remaining_tests = len(self.settings - self.test_index)
        if self.test_index >= len(self.settings):
            self.get_logger().info("All queued tests have been executed")
            return 
               
        # Check whether the belt is worn down. If so - wait for replacement
        current_wear = self.read_initial_wear()
        if current_wear > self.belt_threshold:
            self.wait_for_belt_change(current_wear)

        # Check whether the user wants to continue testing 
        self.get_logger().info(f"There are {remaining_tests} remaining tests to be run. Do you want to continue?")        
        while self._run_next is None:
            self.user_input_check_rate.sleep()
        if not self._run_next: 
            self.get_logger().info(f"The remaining {remaining_tests} tests will not be run. Exiting...")
            return 
        self._run_next = None 

        # Start the next test 
        self.test_start_countdown = self.create_timer(self.startup_delay, self.execute_test)
        self.get_logger().info(f"Next test starting in {self.startup_delay} seconds.")


    def update_wear_history(self, force, rpm, time):
        with open(self.belt_tracking_path, 'a') as f:
            f.write(f'{force},{rpm},{time}\n')

    def read_initial_wear(self):
        wear_csv = pd.read_csv(self.belt_tracking_path, delimiter=',')
        return self.wear_metric_calculation(wear_csv['force'], wear_csv['rpm'], wear_csv['contact_time'])
    
    def wait_for_belt_change(self, current_wear):
        self.get_logger().info(f"The wear metric ({current_wear} has exceeded the threshold of {self.belt_threshold}). Please change the belt")

        while not self._belt_replaced:
            self.user_input_check_rate.sleep()
            
        # Create a new file for tracking wear 
        self.belt_change()
        self._belt_replaced = False 
    

    def belt_change(self):
        # The belt name should have the following format:
            # beltid_{beltid}_grit_{grit}.csv
        current_belt_name_parts = self.belt_tracking_path.name.strip('.csv').split('_')
        new_belt_name_parts = copy.deepcopy(current_belt_name_parts)
        new_belt_name_parts[current_belt_name_parts.index('beltid') + 1] += 1
        new_path = self.belt_tracking_path.parent / f"{'_'.join(new_belt_name_parts)}.csv" 

        if os.path.isfile(new_path):
            self.get_logger().info(f'The auto generated filename {new_path.name} already exists')
            new_path = self.belt_tracking_path.parent / f"belt_{datetime.now()}.csv"
        self.get_logger().info(f'Set the belt history path to {new_path}')

        self.belt_tracking_path = new_path
        if os.path.isfile(self.belt_tracking_path):
            # Ensure that no existing file is overwritten 
            raise ValueError(f"The autogenerated filepath {self.belt_tracking_path}")

        with open(self.belt_tracking_path, "w") as f:
            f.write('force,rpm,contact_time\n')
            f.write('0,0,0\n')
            

    def wear_metric_calculation(self, force, rpm, time):
        return sum(force * rpm * time)

    def create_setting_list(self):
        settings = []
        for i in range(max(len(self.force_settings), len(self.rpm_settings), len(self.contact_time_settings))):
            request = TestRequest.Request()
            request.force = float(self.force_settings[i%len(self.force_settings)])
            request.rpm = float(self.rpm_settings[i%len(self.rpm_settings)])
            request.contact_time = float(self.contact_time_settings[i%len(self.contact_time_settings)])
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
        
    def usr_stop_testing(self, _):
        self._run_next = False
    
    def usr_continue_testing(self, _):
        self._run_next = True

    def usr_changed_belt(self, _):
        self._belt_replaced = True

    def generate_rosbag_suffix(self):
        test_settings = self.settings[self.test_index]
        return f'_sample{self.sample_id}__f{test_settings.force}_rpm{test_settings.rpm}_grit{self.grit}_t{test_settings.contact_time}'


    
def main(args=None):
    rclpy.init(args=args)

    test_coordinator = TestCoordinator()
    executor = MultiThreadedExecutor()

    rclpy.spin(test_coordinator, executor=executor)
    test_coordinator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    