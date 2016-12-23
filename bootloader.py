#!/usr/bin/env python
"""
Bootloader.py

Boot-time application to check for software updates from USB

* Accesses mtab to find mounting point
* Currently no check-sum or package encryption

"""

## Modules
import os, sys, shutil
import pandas as pd
import re

## Constants
app_name = "chractor"
usb_dev = "/dev/sdc" # Regex pattern (probably) of the USB device we are looking for
mtab_path = "/etc/mtab"
app_path = os.path.join("/root", app_name)
usb_path = None

## Runtime
if __name__ == '__main__':
    
    # Try to load MTAB as table
    try:
        mtab_df = pd.read_table(mtab_path, sep=' ', header=None, names=('dev', 'path', 'permissions', '', '')) #!TODO don't know what the last two do
    except Exception as e:
        print "No MTAB!"
        exit(1)
    
    # Search for possible USB device(s)
    for dev,path in mtab_df['dev'].iteritems():
        res = re.match(usb_dev, dev)
        if res is not None:
            usb_path = path.replace('\\040', ' ') # MTAB replaces ' ' with '\040'
            break
    if usb_path is not None:
        mounted = os.path.isdir(usb_path)
    else:
        print "No USB found!"
        exit(1)

    # Check if mount path is valid
    if mounted is not None:
        os.path.exists(usb_path)
        load_path = os.path.join(usb_path, "app")
        app_present = os.path.exists(load_path)
    else:
        print "Unable to access USB mounting directory!"
        exit(1)

    # Check if app directory is present on USB drive
    if app_present:
        print "Copying application data ..."
        try:
            #shutil.copytree(load_path, app_path)
            print "Successfully copied application!"
        except Exception as e:
            print "Unable to copy application!"
    else:
        print "No application found!"
        exit(1)
