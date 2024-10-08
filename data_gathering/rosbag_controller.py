import subprocess
import os
from pathlib import Path
from datetime import datetime


class RosbagRecorder:
    def __init__(self, topics, bag_directory, bag_prefix, logger):
        self.topics = topics
        self.bag_directory = bag_directory
        self.bag_prefix = bag_prefix
        self.bag_process = None
        self.logger = logger

        # Ensure bag directory exists
        if not os.path.isdir(self.bag_directory):
            os.makedirs(self.bag_directory, exist_ok=True)

    def start_recording(self, name_suffix):
        if self.bag_process is not None and self.bag_process.poll() is None:
            self.logger.warn('Recording is already running.')
            return False, 'Recording is already running.'

        # Generate a unique bag name with timestamp
        current_time = datetime.now()
        timestamp = f'{str(current_time.date()).strip()}_{str(current_time.time()).strip().split(".")[0]}'
        bag_name = f"{self.bag_prefix}_{name_suffix}_{timestamp}"
        bag_path = os.path.join(self.bag_directory, bag_name)

        # Build the ros2 bag record command
        cmd = ['ros2', 'bag', 'record', '-s', 'mcap', '-o', bag_path] + self.topics

        try:
            self.bag_process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self.logger.info(f'Started recording bag: {bag_path}')
            return True, f'Recording started: {bag_path}'
        except Exception as e:
            self.logger.error(f'Failed to start recording: {e}')
            return False, f'Failed to start recording: {e}'

    def stop_recording(self):
        if self.bag_process is None or self.bag_process.poll() is not None:
            self.logger.warn('No active recording to stop.')
            return False, 'No active recording to stop.'

        try:
            # Gracefully terminate the recording
            self.bag_process.terminate()
            self.bag_process.wait(timeout=5)
            self.logger.info('Stopped recording bag.')
            self.bag_process = None
            return True, 'Recording stopped.'
        except subprocess.TimeoutExpired:
            self.bag_process.kill()
            self.bag_process.wait()
            self.logger.warning('Recording process killed after timeout.')
            self.bag_process = None
            return False, 'Recording process killed after timeout.'
        except Exception as e:
            # 
            self.logger.error(f'Failed to stop recording: {e}')
            return False, f'Failed to stop recording: {e}'
