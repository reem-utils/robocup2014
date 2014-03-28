#! /usr/bin/env python

import sys
import os
import shutil

root = "/home/albert/fuerte_workspace/reem_at_iri/state_machines/gpsrSoar/real/gpsrSoar/SOAR/gp"
destination = "/home/albert/fuerte_workspace/reem_at_iri/state_machines/gpsrSoar/real/gpsrSoar/SOAR/gpsrSoarFile"

path = root
for path, subdirs, files in os.walk(root):
    for f in files:
    	if f.endswith(".soar"):
    		print os.path.join(path, f)
        	shutil.copy2(f, destination)