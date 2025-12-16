from flask import Flask, render_template, request, jsonify, send_from_directory
import os
from code_checker import ROSCodeChecker
from simulation_runner import SimulationRunner
import shutil
import glob
import time
import json

app = Flask(__name__)
UPLOAD_FOLDER = 'uploads'
STATIC_FOLDER = 'static'
os.makedirs(UPLOAD_FOLDER, exist_ok=True)
os.makedirs(os.path.join(STATIC_FOLDER, 'screenshots'), exist_ok=True)

# To store report
last_report = {}

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/upload', methods=['POST'])
def upload_file():
    global last_report
    if 'file' not in request.files:
        return jsonify({"error": "No file uploaded"}), 400
    
    file = request.files['file']
    zip_path = os.path.join(UPLOAD_FOLDER, file.filename)
    file.save(zip_path)
    
    # 1. Define where to extract
    extract_to = os.path.join(UPLOAD_FOLDER, "extracted_pkg")
    if os.path.exists(extract_to):
        shutil.rmtree(extract_to) # Clean previous uploads
    
    # 2. Run the checker (Ensure your checker unzips the file to 'extract_to')
    checker = ROSCodeChecker(zip_path, extract_to)
    last_report = checker.run_full_check()
    
    # 3. Store the path of the actual package folder for the SimulationRunner
    # We add the extraction path to the report so /run_sim knows where to look
    last_report['package_extract_path'] = extract_to
    
    return jsonify(last_report)

@app.route('/run_sim', methods=['POST'])
def run_sim():
    global last_report
    try:
        pkg_path = last_report.get('package_extract_path')
        ros_ver = last_report.get('ros_version', 'ROS 2')

        if not pkg_path:
            return jsonify({"error": "No validated package found","success": False,"status": "ERROR"}), 400

        # Run the simulation logic
        runner = SimulationRunner(pkg_path, ros_ver)
        runner.run_simulation()
        
        #Wait for files to sync
        time.sleep(1)
        
        success = False
        status_text = "FAILURE"
        if os.path.exists("static/simulation_report.json"):
            with open("static/simulation_report.json", "r") as f:
                sim_data = json.load(f)
                status_text = sim_data.get("simulation_status", "FAILURE")
                success = (status_text == "SUCCESS")

        # Read the generated logs
        cli_logs = "No CLI logs captured."
        if os.path.exists("static/cli_output.log"):
            with open("static/cli_output.log", "r") as f:
                cli_logs = f.read()
                
        joint_logs = "Joint motions not captured."
        if os.path.exists("static/joint_motions.log"):
            with open("static/joint_motions.log", "r") as f:
                joint_logs = f.read()

        return jsonify({
            "status": status_text,
            "success": success,
            "cli_output": cli_logs,
            "joint_output": joint_logs
        })
        
    except Exception as e:
        print(f"Simulation Error:{e}")
        return jsonify({"status": "Error", "success": False, "error": str(e)}), 200
        
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
