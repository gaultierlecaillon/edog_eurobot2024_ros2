import subprocess

response = subprocess.Popen(f"(echo '1pass4u!' | sudo -S ./aruco_detector.py)", stderr=subprocess.PIPE, stdout=subprocess.PIPE, shell=True)
output, errors = response.communicate()
