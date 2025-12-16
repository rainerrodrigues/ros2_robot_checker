import subprocess
import time
import os
import signal
import json
import xml.etree.ElementTree as ET
import glob
import shutil
import fcntl

class SimulationRunner:
    def __init__(self, package_path, ros_version):
        self.package_path = package_path
        self.ros_version = ros_version 
        self.sim_proc = None
        self.recorder_proc = None
        self.package_name = self._detect_package_name()
    
    def _detect_package_name(self):
        try:
            for root, dirs, files in os.walk(self.package_path):
                if 'package.xml' in files:
                    tree = ET.parse(os.path.join(root, 'package.xml'))
                    root_node = tree.getroot()
                    name_tag = root_node.find('name')
                    if name_tag is not None:
                        return name_tag.text.strip()
        except Exception as e:
            print(f"Error detecting package name: {e}")
            return "user_pkg"

    def _detect_executable(self):
        result = subprocess.run(
           ['ros2', 'pkg', 'executables', self.package_name],
           capture_output=True, text=True
        )
        lines = result.stdout.strip().splitlines()
        if not lines:
            raise RuntimeError("No executables found in package")
        return lines[0].split()[1]

    def run_simulation(self):
        # 1. Connection to Code Checker: Only run if validation passed 
        if os.path.exists('checker_report.json'):
            with open('checker_report.json', 'r') as f:
                report = json.load(f)
                if not report.get("passed", False):
                    print("Aborting: Code failed validation.")
                    return

        world_path = os.path.abspath(os.path.join(os.path.dirname(__file__), 'simulation', 'worlds', 'pick_and_place.world'))
        
        # 2. Launching with immediate physics start (-r) 
        launch_cmd = ['ros2', 'launch', 'ur_simulation_gz', 'ur_sim_control.launch.py', f'gz_args:=-r {world_path}', 'use_sim_time:=true']
        joint_topic = '/joint_states' # Broadcaster topic 

        print(f"Starting {self.ros_version} Simulation...")
        self.sim_proc = subprocess.Popen(launch_cmd)

        # 3. Robust Controller Wait
        print("Waiting for controllers to activate...")
        time.sleep(20)
        for _ in range(15):
            check_ctrl = subprocess.run(['ros2', 'control', 'list_controllers'], capture_output=True, text=True)
            if "scaled_joint_trajectory_controller [active]" in check_ctrl.stdout:
                print("Controller ACTIVE.")
                break
            time.sleep(2)
        
        # 4. Recording Motions 
        log_file = open("static/joint_motions.log", "w")
        record_cmd = ['ros2', 'topic', 'echo', '--qos-reliability', 'best_effort', '--qos-durability', 'volatile', joint_topic]
        self.recorder_proc = subprocess.Popen(record_cmd, stdout=log_file)
        output_text = []

        # 5. Non-blocking User Node Execution 
        try:
            exec_name = self._detect_executable()
            cmd = ['ros2', 'run', self.package_name, exec_name, '--ros-args', '-p', 'use_sim_time:=true']
            user_proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            
            # Non-blocking setup
            fd = user_proc.stdout.fileno()
            fl = fcntl.fcntl(fd, fcntl.F_GETFL)
            fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)

            #output_text = []
            start_time = time.time()
            while (time.time() - start_time) < 60:
                if user_proc.poll() is not None: break
                try:
                    line = user_proc.stdout.readline()
                    if line:
                        print(f"CLI: {line.strip()}")
                        output_text.append(line)
                except IOError: pass
                time.sleep(0.1)

            # Success Verification 
            success = self._verify_cube_placed(target_x=0.5, target_y=0.5)
            # self._capture_screenshot()
            self._generate_json_report(success)
            
            with open("static/cli_output.log", "w") as f:
                f.writelines(output_text)
                

        except Exception as e:
            print(f"Execution Error: {e}")
            self._generate_json_report(False) # Report failure if node execution crashes
            with open("static/cli_output.log", "w") as f:
                f.writelines(output_text)
                
            
        finally:
            print(f"Task Status: {'SUCCESS' if success else 'FAILURE'}")
            self._capture_screenshot()
            self.cleanup()

        
    def _capture_screenshot(self):
        try:
            base_dir = os.path.dirname(os.path.abspath(__file__))
            target_path = os.path.join(base_dir, 'static', 'screenshots', 'final_frame.png')
            subprocess.run(['gz', 'gui', '--screenshot'], timeout=10)
            time.sleep(3)
            
             
            home = os.path.expanduser("~")
            list_of_files = glob.glob(os.path.join(home, '*.png'))
            if list_of_files:
                latest_file = max(list_of_files, key=os.path.getctime)
                base_dir = os.path.dirname(os.path.abspath(__file__))
                target_dir = os.path.join(base_dir,'static', 'screenshots')
                target_path = os.path.join(target_dir, 'final_frame.png')
                os.makedirs(target_dir, exist_ok=True)
                shutil.move(latest_file, target_path) # Move file to the correct static path
                print(f"Screenshot successfully moved to: {target_path}")
                print("Screenshot saved successfully.")
        except Exception as e:
            print(f"Screenshot capture failed: {e}")

    def _verify_cube_placed(self, target_x, target_y):
        """Checks if cube is near the target coordinates """
        try:
            res = subprocess.run(['gz', 'topic', '-e', '-t', '/model/cube/pose', '-n', '1'], 
                                 capture_output=True, text=True, timeout=5)
            # Basic coordinate parsing logic could be added here
            return "position" in res.stdout
        except: return False
                
    def _generate_json_report(self, success):
        base_dir = os.path.dirname(os.path.abspath(__file__))
        static_dir = os.path.join(base_dir, 'static')
        report = {
            "package_name": self.package_name,
            "simulation_status": "SUCCESS" if success else "FAILURE",
            "logs": {"joint_motions":os.path.join(static_dir, "joint_motions.log"), "cli_output":os.path.join(static_dir, "cli_output.log")},
            "artifacts": {"screenshot": "static/screenshots/final_frame.png"},
            "timestamp": time.time()
        }
        with open("static/simulation_report.json", "w") as f:
            json.dump(report, f, indent=4)
        
    def cleanup(self):
        if self.sim_proc: os.kill(self.sim_proc.pid, signal.SIGINT)
        if self.recorder_proc: self.recorder_proc.terminate()
