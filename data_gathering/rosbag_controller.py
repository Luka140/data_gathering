import subprocess
from pathlib import Path
from datetime import datetime
import threading
import time


class RosbagRecorder:
    def __init__(self, topics, bag_directory, bag_prefix, logger):
        self.topics = topics
        self.bag_directory = Path(bag_directory)
        self.bag_prefix = bag_prefix
        self.bag_process = None
        self.logger = logger

        self.bag_name = None

        # Ensure bag directory exists
        self.bag_directory.mkdir(parents=True, exist_ok=True)

    def start_recording(self, name_suffix):
        if self.bag_process is not None and self.bag_process.poll() is None:
            self.logger.warn('Recording is already running.')
            return False, 'Recording is already running.'

        # Generate a unique bag name with timestamp
        current_time = datetime.now()
        timestamp = current_time.strftime("%Y-%m-%d_%H-%M-%S")
        bag_name = f"{self.bag_prefix}_{name_suffix}_{timestamp}"
        bag_path = self.bag_directory / bag_name

        # Build the ros2 bag record command
        cmd = ['ros2', 'bag', 'record', '-s', 'mcap', '-o', str(bag_path)] + self.topics

        try:
            self.bag_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                universal_newlines=True
            )
            self.logger.info(f'Started recording bag: {bag_path}')
            self.bag_name = bag_name

            # Start a thread to monitor the subprocess output (optional, for debugging)
            threading.Thread(target=self._monitor_output, daemon=True).start()

            return True, f'Recording started: {bag_path}'
        except Exception as e:
            self.logger.error(f'Failed to start recording: {e}')
            return False, f'Failed to start recording: {e}'

    def _monitor_output(self):
        """Optional: Monitor the rosbag subprocess output for debugging."""
        if self.bag_process.stdout:
            for line in self.bag_process.stdout:
                self.logger.debug(f'rosbag: {line.strip()}')

    def stop_recording(self):
        if self.bag_process is None or self.bag_process.poll() is not None:
            self.logger.warn('No active recording to stop.')
            return False, 'No active recording to stop.'

        try:
            # Gracefully terminate the recording
            self.bag_process.terminate()
            try:
                self.bag_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.bag_process.kill()
                self.bag_process.wait()
                self.logger.warning('Recording process killed after timeout.')
                return False, 'Recording process killed after timeout.'

            self.logger.info('Stopped recording bag.')
            self.bag_process = None
            self.bag_name = None
            return True, 'Recording stopped.'
        except Exception as e:
            self.logger.error(f'Failed to stop recording: {e}')
            return False, f'Failed to stop recording: {e}'

    def wait_for_rosbag(self, timeout_sec=10.0):
        """Wait until the bag file is created and has some data."""
        if self.bag_name is None:
            self.logger.error('No bag recording in progress.')
            return False, 'No bag recording in progress.'

        bag_path = self.bag_directory / self.bag_name

        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if bag_path.exists():
                if bag_path.stat().st_size > 0:
                    self.logger.info(f'Bag file {bag_path} created and is recording.')
                    return True, 'Bag is ready.'
            time.sleep(0.5)

        self.logger.error(f'Timeout waiting for bag file {bag_path} to be ready.')
        return False, f'Timeout waiting for bag file {bag_path} to be ready.'
