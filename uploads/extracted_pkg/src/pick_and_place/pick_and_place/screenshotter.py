def main():
    import subprocess
    import time

    # Wait for Gazebo GUI
    time.sleep(6)

    subprocess.run([
        'gz', 'service',
        '-s', '/gui/screenshot',
        '--reqtype', 'gz.msgs.Empty',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '3000'
    ])

