# Copyright 2025 IDRA, University of Trento
# Author: Matteo Dalle Vedove (matteodv99tn@gmail.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
