import subprocess
import time
import os
import signal
import json
import xml.etree.ElementTree as ET
import glob
import shutil

class SimulationRunner:
    def __init__(self, package_path, ros_version):
        self.package_path = package_path
        self.ros_version = ros_version # ROS/ROS2
        self.sim_proc = None
        self.recorder_proc = None
        self.package_name = self._detect_package_name()
    
    def _detect_package_name(self):
        try:
            for root, dirs, files in os.walk(self.package_path):
                if 'package.xml' in files:
                    tree = ET.parse(os.path.join(root, 'package.xml'))
                    root_node = tree.getroot()
                    # Find the <name> tag
                    name_tag = root_node.find('name')
                    if name_tag is not None:
                        return name_tag.text.strip()
        except Exception as e:
            print(f"Error detecting package name: {e}")
            return "user_pkg" # Fallback
            
    """bridge_cmd = [
    'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
    '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model']
    subprocess.Popen(bridge_cmd)
    time.sleep(2)"""

    def _detect_executable(self):
        result = subprocess.run(
           ['ros2', 'pkg', 'executables', self.package_name],
           capture_output=True, text=True
        )
        lines = result.stdout.strip().splitlines()
        if not lines:
            raise RuntimeError("No executables found in package")
        return lines[0].split()[1]  # package exec_name

    def run_simulation(self):
        output_log = []
        world_path = os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            'simulation',
            'worlds',
            'pick_and_place.world'
        )
        )
        if self.ros_version == "ROS 2":
            launch_cmd = ['ros2', 'launch', 'ur_simulation_gz', 'ur_sim_control.launch.py', 
            f'gz_args:={world_path}']
            joint_topic = '/joint_states'
        else:
            launch_cmd = ['roslaunch', 'ur_gazebo', 'ur5_bringup.launch']
            joint_topic = '/joint_states'

        print(f"Starting {self.ros_version} Gazebo Simulation...")
        self.sim_proc = subprocess.Popen(launch_cmd)
        time.sleep(12) 

        print("Waiting for simulation to initialize...")
        time.sleep(5)
        
        print("Recording joint motions...")
        log_file = open("static/joint_motions.log", "w")
        record_cmd = ['ros2', 'topic', 'echo','--qos-reliability', 'best_effort', 
    '--qos-durability', 'volatile', joint_topic] if self.ros_version == "ROS 2" else ['rostopic', 'echo', joint_topic]
        self.recorder_proc = subprocess.Popen(record_cmd, stdout=log_file)

        print(f"Running submitted {self.ros_version} node...")
        output_text = [] # To store CLI output
        
        try:
            # 1. Start the user node and redirect output to a PIPE
            exec_name = self._detect_executable()
            cmd = ['ros2', 'run', self.package_name, exec_name] if self.ros_version == "ROS 2" else ['rosrun', self.package_name, exec_name]
            
            user_proc = subprocess.Popen(
                cmd, 
                stdout=subprocess.PIPE, 
                stderr=subprocess.STDOUT, 
                text=True
            )

            # 2. Capture output and handle timeout
            start_time = time.time()
            timeout = 45 # seconds
            
            while True:
                # Check if process is still running or has output
                line = user_proc.stdout.readline()
                if line:
                    print(f"CLI: {line.strip()}") # Print to terminal
                    output_text.append(line)      # Save for web UI
                
                if user_proc.poll() is not None: # Process finished
                    break
                
                if (time.time() - start_time) > timeout:
                    print("Simulation timed out. Terminating...")
                    user_proc.terminate()
                    output_text.append("TIMEOUT: Simulation terminated after 45s.")
                    break
            
            # Save the captured CLI logs to a file for the Web UI to read
            with open("static/cli_output.log", "w") as f:
                f.writelines(output_text)

        except Exception as e:
            print(f"Execution Error: {e}")

        self._capture_screenshot()
        success = True
        
        print(f"Task Status: {'SUCCESS' if success else 'FAILURE'}")
        self.cleanup()

    def _capture_screenshot(self):
        """os.makedirs('static/screenshots',exist_ok=True)
        target_path = "static/screenshots/final_frame.png"
        subprocess.run(['gz', 'gui', '--screenshot'])
        time.sleep(2)
        home = os.path.expanduser("~")
        files = [os.path.join(home, f) for f in os.listdir(home) if f.endswith('.png')]
        if files:
            latest_file = max(files, key=os.path.getctime)
            shutil.move(latest_file, target_path)
            print(f"Screenshot saved to {target_path}") 
            """
        # Trigger the screenshot
        subprocess.run(['gz', 'gui', '--screenshot'])
        time.sleep(2) # Wait for Gazebo to save

        # Gazebo saves to your HOME directory (~/) by default in WSL
        home = os.path.expanduser("~")
        list_of_files = glob.glob(os.path.join(home, '*.png'))
        if list_of_files:
            latest_file = max(list_of_files, key=os.path.getctime)
            # Move it to your project's static folder
            shutil.move(latest_file, 'static/screenshots/final_frame.png') 

    def _verify_cube_position(self):
        # Check if the cube model pose topic has data
        res = subprocess.run(
        ['gz', 'topic', '-e', '-t', '/model/cube/pose', '-n', '1'],
        capture_output=True, text=True, timeout=5
        )
        return "position" in res.stdout and "x:" in res.stdout
        
    def cleanup(self):
        if self.sim_proc:
            os.kill(self.sim_proc.pid, signal.SIGINT)
        if self.recorder_proc:
            #os.kill(self.recorder_proc.pid, signal.SIGINT)
            self.recorder_proc.terminate()

# Test
# runner = SimulationRunner("path/to/pkg", "ROS 2")
# runner.run_simulation()
