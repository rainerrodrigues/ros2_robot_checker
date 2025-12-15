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
            "ros_version": "Unknown",
            "syntax_errors": [],
            "structure_errors": [],
            "ros_entities": {"publishers": 0, "subscribers": 0, "services": 0, "actions": 0, "initialized": False},
            "safety_warnings": [],
            "passed": True
        }

    def run_full_check(self):
        if not self._extract_zip():
            return self._finalize_report()

        self._check_ros_structure()
        self._check_code_details()
        return self._finalize_report()

    def _extract_zip(self):
        try:
            if os.path.exists(self.extract_path):
                shutil.rmtree(self.extract_path)
            with zipfile.ZipFile(self.zip_path, 'r') as zip_ref:
                zip_ref.extractall(self.extract_path)
            return True
        except Exception as e:
            self.report["structure_errors"].append(f"Failed to unzip: {str(e)}") 
            self.report["passed"] = False
            return False

    def _check_ros_structure(self):
        pkg_xml_path = None
        has_cmake = False
        has_setup_py = False

        for root, dirs, files in os.walk(self.extract_path):
            if 'package.xml' in files:
                pkg_xml_path = os.path.join(root, 'package.xml')
            if 'CMakeLists.txt' in files:
                has_cmake = True
            if 'setup.py' in files:
                has_setup_py = True

        if not pkg_xml_path:
            self.report["structure_errors"].append("Missing package.xml")
            self.report["passed"] = False
        else:
            try:
                with open(pkg_xml_path, 'r') as f:
                    pkg = parse_package_string(f.read())
                    self.report["ros_version"] = "ROS 2" if has_setup_py or "ament_cmake" in [d.name for d in pkg.build_depends] else "ROS 1"
            except Exception as e:
                self.report["structure_errors"].append(f"Malformed package.xml: {str(e)}")

        if not (has_cmake or has_setup_py):
            self.report["structure_errors"].append("Missing build file (CMakeLists.txt or setup.py)")

    def _check_code_details(self):
        for root, dirs, files in os.walk(self.extract_path):
            for file in files:
                file_path = os.path.join(root, file)
                
                if file.endswith('.py'):
                    # Syntax Check
                    res = subprocess.run(['flake8', file_path], capture_output=True, text=True) 
                    if res.returncode != 0:
                        self.report["syntax_errors"].append(f"{file}: {res.stdout}")
                        self.report["passed"] = False
                    self._detect_entities(file_path, is_python=True)

                elif file.endswith(('.cpp', '.hpp', '.h')):
                    # Syntax Check
                    res = subprocess.run(['g++', '-fsyntax-only', file_path], capture_output=True, text=True)
                    if res.returncode != 0:
                        self.report["syntax_errors"].append(f"{file}: {res.stderr}")
                        self.report["passed"] = False
                    self._detect_entities(file_path, is_python=False)

    def _detect_entities(self, file_path, is_python):
        with open(file_path, 'r') as f:
            content = f.read()
            
            # Initialization Checks 
            if 'rclpy.init' in content or 'rospy.init_node' in content or 'ros::init' in content:
                self.report["ros_entities"]["initialized"] = True

            # Entity Detection 
            self.report["ros_entities"]["publishers"] += (content.count('create_publisher') + content.count('rospy.Publisher'))
            self.report["ros_entities"]["subscribers"] += (content.count('create_subscription') + content.count('rospy.Subscriber'))
            self.report["ros_entities"]["services"] += (content.count('create_service') + content.count('rospy.Service('))
            
            # Safety Heuristics 
            if "while" in content and "sleep" not in content and "rclpy.spin" not in content:
                self.report["safety_warnings"].append(f"{os.path.basename(file_path)}: Loop detected without sleep/spin (CPU safety risk).")
            
            if "joint" in content.lower() and ("> 3.14" in content or "> 6.28" in content):
                 self.report["safety_warnings"].append(f"{os.path.basename(file_path)}: Potential unsafe joint value detected.")

    def _finalize_report(self):
        # Generate JSON Report 
        with open('checker_report.json', 'w') as f:
            json.dump(self.report, f, indent=4) 
        
        # Generate Text Report 
        with open('checker_report.txt', 'w') as f:
            f.write(f"ROS VERSION: {self.report['ros_version']}\n")
            f.write(f"PASSED: {self.report['passed']}\n")
            f.write(f"ENTITIES: {self.report['ros_entities']}\n")
            f.write("ERRORS:\n" + "\n".join(self.report['syntax_errors'] + self.report['structure_errors']))
            f.write("\nWARNINGS:\n" + "\n".join(self.report['safety_warnings']))
            
        return self.report
