import os
import zipfile
import subprocess
import json
import shutil
from catkin_pkg.package import parse_package_string

class ROSCodeChecker:
	def __init__(self, zip_path, extract_path='temp_ws'):
		self.zip_path = zip_path
		self.extract_path = extract_path
		self.report = { 
			"syntax_errors": [],
			"structure_errors": [],
			"ros_entities": {"publishers": 0, "subscribers": 0, "services":0},
			"safety_warnings": [],
			"passed":True,
			}
	
	def run_check(self):
		if not self._extract_zip()
			return self._finalize_report()
			
		self._check_ros_structure()
		
		self._check_code_details()
		
		return self._finalize_report()
		
	def _extract_zip(self):
		try:
			if os.path.exists(self.extract_path):
				shutil.rmtree(self.extract_path)
			with zipfile.ZipFile(self.zip_path,'r') as zip_ref:
				zip_ref.extractall(self.extract_path)
			return True
		except Exception as e:
			self.report["structure_errors"].append(f"Failed to unzip: {str(e)}")
			self.report["passed"] = False
			return False
			
	def _check_ros_structure(self):
		"""Checking if package.xml and CMakeLists.txt/setup.py exists"""
		pkg_xml_path = None
		for root, dirs, files in os.walk(self.extract_path):
			if 'package.xml' in files:
				pkg_xml_path = os.path.join(root, 'package.xml')
				break
			
		if not pkg_xml_path:
			self.report["structure_errors"].append("Missing package.xml")
			self.report["passed"] = False
		else:
			try:
				with open(pkg_xml_path,'r') as f:
					parse_package_string(f.read())
			except Exception as e:
				self.report["structure_errors"].append(f"Malformed package.xml: {str(e)}")
				
	def _check_code_details(self):
        """Syntax validation and entity detection [cite: 13, 15]"""
        	for root, dirs, files in os.walk(self.extract_path):
            		for file in files:
                		if file.endswith('.py'):
                    			file_path = os.path.join(root, file)
                    
                    	res = subprocess.run(['flake8', file_path], capture_output=True, text=True)
                    	if res.returncode != 0:
                        	self.report["syntax_errors"].append(f"{file}: {res.stdout}")
                        	self.report["passed"] = False

                    # ROS Entity Detection 
                    	with open(file_path, 'r') as f:
                        	content = f.read()
                        	self.report["ros_entities"]["publishers"] += content.count('create_publisher')
                        	self.report["ros_entities"]["subscribers"] += content.count('create_subscription')
                        	self.report["ros_entities"]["services"] += content.count('create_service')

                        # Simple Safety Check: Looking for sleep in loops 
                        	if 'while' in content and 'sleep' not in content:
                            		self.report["safety_warnings"].append(f"{file}: While loop detected without visible sleep.")

    	def _finalize_report(self):
        	with open('checker_report.json', 'w') as f:
            	json.dump(self.report, f, indent=4)
        	return self.report

# Testing
if __name__ == "__main__":
    checker = ROSCodeChecker('your_test_package.zip')
    final_results = checker.run_full_check()
    print(json.dumps(final_results, indent=4))

def check_syntax(file_path):
	# Use flake-8 on the file provided by the users
	result = subprocess.run(['flake8',file_path], capture_output=True, text=True)
	
	if result.returncode == 0:
		return {"status": "Passed", "errors": []}
	else:
		return {"status": "Failed", "errors": result.stdout.splitlines()}
		
