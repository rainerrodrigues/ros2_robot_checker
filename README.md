## 🤖 ROS Code Checker and Simulation Tool Documentation (README)

This tool provides a web-based interface for validating and simulating user-uploaded ROS 2 robot packages. It checks code for syntax and safety, runs the package in a Gazebo simulation environment, and displays real-time logs and a final screenshot of the result.

### 1\. Setup Instructions

To run this tool, you need a functional ROS 2 environment with the necessary dependencies for running simulations and Python code execution.

#### Prerequisites

1.  **Operating System:** Ubuntu 22.04 LTS (Recommended)
2.  **ROS 2 Distribution:** ROS 2 Humble/Iron/Kilted (The tool is configured for ROS 2)
3.  **Gazebo (Ignition) Sim:** Installed and configured for the `ur_simulation_gz` package.
4.  **Python Packages:**
      * `Flask`
      * `python-rosdep2` (for checking ROS dependencies)
      * `fcntl`, `shutil`, `glob`, `json`, `xml.etree.ElementTree` (standard library dependencies used across `app.py` and `simulation_runner.py`)

#### Installation Steps

1.  **Install Python Dependencies:**

    ```bash
    pip install Flask
    # Add any other required non-standard libraries here (e.g., if needed for code_checker.py) and create a virtual environment while doing this
    ```

2.  **Configure ROS 2 Workspace:**
    Source your ROS 2 environment in your terminal session:

    ```bash
    source /opt/ros/<ROS_DISTRO>/setup.bash
    ```

3.  **Setup Directory Structure:**
    Ensure your project directory (`ros_robot_checker/`) has the following structure for static file serving and temporary storage:

    ```
    ros_robot_checker/
    ├── app.py
    ├── index.html
    ├── simulation_runner.py
    ├── code_checker.py (assumed)
    ├── static/
    │   ├── screenshots/  <-- Folder for final_frame.png (must exist)
    │   ├── joint_motions.log
    │   └── cli_output.log
    └── uploads/          <-- Folder for extracting user ZIP files
    ```

    *Note: The `static/screenshots` folder must be created manually before running the tool.*

### 2\. How to Run the Tool

The tool runs as a local Flask web server that handles package validation, simulation execution, and artifact generation.

#### Step 1: Start the Flask Application

Navigate to the project root directory and execute `app.py`:

```bash
python3 app.py
```

The server will start and typically run on `http://127.0.0.1:5000/`.

#### Step 2: Access the Web UI

Open your web browser and navigate to the local host address.

1.  **Upload Package:** Use the "Choose file" button to select a ROS 2 package zipped file (e.g., `ros2_correct_py_pkg.zip`).
2.  **Validate Code:** Click **"Validate Code"**. The backend processes the file, checks for syntax, structure, and safety violations, and displays the **Validation Report**.
      * If validation fails, errors are displayed in a red log area, and the "Run Simulation" button is hidden.
3.  **Run Simulation:** If validation passes, click **"🚀 Run Simulation"**.
      * The backend launches Gazebo and the necessary ROS components.
      * It executes the user's main node (`ros2 run`).
      * **Real-Time Display:** The `Simulation Results` section updates to show:
          * **Status Badge:** Green for `SUCCESS`, Red for `FAILURE`.
          * **CLI Execution Logs:** Output from the user's ROS node.
          * **Joint Motion Logs:** Data captured from the robot's joint state topic.
          * **Final Frame Preview:** A screenshot (`final_frame.png`) of the final state of the Gazebo world.

### 3\. Logs and Notes from Testing

The following notes document the critical software bugs encountered during development and the solutions implemented to achieve a stable and reliable execution flow.

| Error Type | Log Message / Observation | Root Cause & Resolution |
| :--- | :--- | :--- |
| **Python Crash** | `SyntaxError: invalid syntax` at `finally:` | **Cause:** Indentation error in the `try-except-finally` block in `simulation_runner.py`. **Resolution:** Corrected Python indentation structure. |
| **Python Crash** | `name 'time' is not defined` / `name 'json' is not defined` | **Cause:** Missing `import` statements in `app.py` or `simulation_runner.py`. **Resolution:** Added `import time` and `import json` to the top of the respective files to prevent crashes during serialization or delay handling. |
| **Frontend Failure** | CLI Logs and Status showing `"undefined"` | **Cause:** Python crash was preventing variables (`success`, `output_text`) from being initialized before the `except` block was hit, leading to incomplete JSON responses. **Resolution:** Initialized critical variables (`success = False`, `output_text = []`) *before* the main `try` block in `simulation_runner.py`. |
| **Critical File Failure** | `172.17.224.1 - - "GET /static/screenshots/final_frame.png?t=... HTTP/1.1" 404 -` | **Cause:** The `_capture_screenshot` method in `simulation_runner.py` was failing to move the file to the correct web-accessible path due to ambiguity in relative path construction. **Resolution:** Enforced the use of **absolute paths** when moving the captured file from the user's home directory to `[PROJECT_ROOT]/static/screenshots/final_frame.png` using `os.path.abspath(__file__)`. |
| **Display Redundancy** | Joint Motion Logs were initially accessed via an `iframe`, causing browser download prompts. | **Resolution:** Modified `app.py` to read the content of `joint_motions.log` into the JSON response (`data.joint_output`) and updated `index.html` to display the content in a dedicated `<textarea>` for reliable rendering. |
