#!/usr/bin/env python3

import subprocess
import os
import logging



# Putanja do direktorija u kojem se nalaze skripte
package_directory = os.path.dirname(os.path.abspath(__file__))
scripts_directory = os.path.join(package_directory, "gcode_parser.py")

# Pokretanje gcode_parser.py skripte
subprocess.run(["python3", scripts_directory])

# Pokretanje move.py skripte
subprocess.run(["python3", os.path.join(package_directory, "move.py")])

