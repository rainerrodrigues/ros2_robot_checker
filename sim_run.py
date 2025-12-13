import subprocess
import time
import os
import signal
import json

class SimulationRunner:
    def __init__(self, package_path, ros_version):
        self.package_path = package_path
        self.ros_version = ros_version # ROS/ROS2
        self.sim_proc = None
        self.recorder_proc = None

    def run_simulation(self):
        if self.ros_version == "ROS 2":
            launch_cmd = ['ros2', 'launch', 'ur_simulation_gazebo', 'ur_sim_control.launch.py']
            joint_topic = '/joint_states'
        else:
            launch_cmd = ['roslaunch', 'ur_gazebo', 'ur5_bringup.launch']
            joint_topic = '/joint_states'

        print(f"Starting {self.ros_version} Gazebo Simulation...")
        self.sim_proc = subprocess.Popen(launch_cmd)
        time.sleep(12) 

  
        print("Recording joint motions...")
        log_file = open("joint_motions.log", "w")
        record_cmd = ['ros2', 'topic', 'echo', joint_topic] if self.ros_version == "ROS 2" else ['rostopic', 'echo', joint_topic]
        self.recorder_proc = subprocess.Popen(record_cmd, stdout=log_file)

        print(f"Running submitted {self.ros_version} node...")
        try:
            if self.ros_version == "ROS 2":
                user_proc = subprocess.run(['ros2', 'run', 'user_pkg', 'move_arm_node'], timeout=45)
            else:
                user_proc = subprocess.run(['rosrun', 'user_pkg', 'move_arm_node'], timeout=45)
        except subprocess.TimeoutExpired:
            print("Simulation timed out.")

        self._capture_screenshot()
        success = self._verify_cube_position()
        
        print(f"Task Status: {'SUCCESS' if success else 'FAILURE'}")
        self.cleanup()

    def _capture_screenshot(self):
        subprocess.run(['gz', 'gui', '--screenshot'])

    def _verify_cube_position(self):
        """
    Checks if the 'cube' is within 10cm of the 'target_marker'
    Target coordinates from our world file: x=0.5, y=0.5
    """
    	try:
        	#Getting Gazebo CLI to capture
        	result = subprocess.run(
            	['gz', 'model', '--model-name', 'cube', '--pose'], 
            	capture_output=True, text=True
        	)
        
        	#Parsing output using string parsing
        	output = result.stdout
        	x = float(output.split('x:')[1].split(',')[0].strip())
       		y = float(output.split('y:')[1].split(',')[0].strip())
        
        	# Calculate Euclidean distance to target (0.5, 0.5)
        	target_x, target_y = 0.5, 0.5
        	distance = math.sqrt((x - target_x)**2 + (y - target_y)**2)
        
        	# Success if within 10cm (0.1 meters)
        	return distance < 0.1

    except Exception as e:
        print(f"Error verifying position: {e}")
        return False 

    def cleanup(self):
        if self.sim_proc:
            os.kill(self.sim_proc.pid, signal.SIGINT)
        if self.recorder_proc:
            os.kill(self.recorder_proc.pid, signal.SIGINT)

# Test
# runner = SimulationRunner("path/to/pkg", "ROS 2")
# runner.run_simulation()
