from flask import Flask, render_template, request, jsonify, send_from_directory
import os
from code_checker import ROSCodeChecker
from simulation_runner import SimulationRunner
import shutil
import glob

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
        # Pass the extracted folder path stored during upload
        pkg_path = last_report.get('package_extract_path')
        ros_ver = last_report.get('ros_version', 'ROS 2')
        screenshots_dir = os.path.join(STATIC_FOLDER, 'screenshots')
        screenshots = sorted(glob.glob(os.path.join(screenshots_dir, '*.png')))
        screenshot_urls = [    
            f"/static/screenshots/{os.path.basename(s)}"
            for s in screenshots
            ]
        if not pkg_path:
            return jsonify({"error": "Package path missing"}), 400

        runner = SimulationRunner(pkg_path, ros_ver)
        runner.run_simulation()
        with open("static/cli_output.log", "r") as f:
            cli_logs = f.read()
        
        return jsonify({
        "status": "Simulation Finished",
        "cli_output": cli_logs,
        "screenshots": screenshot_urls
        })
        
    except Exception as e:
        print(f"Simulation Error:{e}")
        return jsonify({"error": str(e)}), 500  

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
