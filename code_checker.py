import subprocess
import json

def check_syntax(file_path):
	# Use flake-8 on the file provided by the users
	result = subprocess.run(['flake8',file_path], capture_output=True, text=True)
	
	if result.returncode == 0:
		return {"status": "Passed", "errors": []}
	else:
		return {"status": "Failed", "errors": result.stdout.splitlines()}
		
# Testing
report = check_syntax('my_node.py')
print(json.dumps(report,indent=4))
