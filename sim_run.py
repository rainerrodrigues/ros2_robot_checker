import subprocess
import time
import os
import signal

class SimulationRunner:
	def __init__(self, package_path):
		self.package_path = package_path
		self.log_file = "sim_logs.txt"
		self.recorder_proc = None
		
	def run_simulation(self):
		print("Launching Gazebo...")
		sim_proc = subprocess.Popen(['ros2','launch','sim_pkg','ur5_task.launch.py'])
		
		time.sleep(10)
		
		print("Recording joint motions...")
		self.record_proc = subprocess.Popen(['ros2','topic','echo','/joint_states','>','joint_logs.txt'],shell=True)
		
		print("Running user node...")
		try:
			user_node_proc = subprocess.run(['ros2', 'run', 'user_package', 'user_node'], timeout=30)
            		print("User node execution finished.")
            		
            		self._check_success()
            		
            		subprocess.run(['gz','gui','--screenshot'])
            		
            		sim_proc.send_signal(signal.SIGINT)
            		self.recorder_proc.send_signal(signal.SIGINT)
            		
         def _check_sucess(self):
         #TODO
         	pass
         	
if __name__ == "__main__":
    runner = SimulationRunner("temp_ws/install/user_package")
    runner.run_simulation()
