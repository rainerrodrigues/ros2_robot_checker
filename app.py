from flask import Flask, render_template, request, jsonify, send_from_directory
import os
from code_checker import ROSCodeChecker
from simulation_runner import SimulationRunner

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
    filepath = os.path.join(UPLOAD_FOLDER, file.filename)
    file.save(filepath)
    
    #Run the code checker
    checker = ROSCodeChecker(filepath)
    last_report = checker.run_full_check()
    
    return jsonify(last_report)

@app.route('/run_sim', methods=['POST'])
def run_sim():
    runner = SimulationRunner(UPLOAD_FOLDER, last_report.get('ros_version', 'ROS 2'))
    
    try:
        runner.run_simulation()
        
        # 3. Return the paths so the frontend can display them
        return jsonify({
            "status": "Simulation Complete",
            "log_url": "/static/joint_motions.log",
            "screenshot_url": "/static/screenshots/final_frame.png"
        })
    except Exception as e:
        return jsonify({"error": f"Simulation failed: {str(e)}"}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
