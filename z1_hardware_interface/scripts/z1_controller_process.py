import subprocess
import signal
import sys
import time
import os.path
from ament_index_python import get_package_share_directory


share_dir = get_package_share_directory("z1_hardware_interface")
work_dir = os.path.join(share_dir, "controller")
delay_secs = 4

proc = subprocess.Popen(["./z1_ctrl"], cwd=work_dir)



def handle_sigint(signum, frame):
    print(f"SIGINT received. Waiting {delay_secs} seconds before shutdown...")
    time.sleep(delay_secs)
    proc.send_signal(signal.SIGINT)
    print("Finished")
    sys.exit(0)



signal.signal(signal.SIGINT, handle_sigint)
try:
    proc.wait()
except KeyboardInterrupt:
    handle_sigint(signal.SIGINT, None)
