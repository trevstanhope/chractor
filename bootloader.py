#!/usr/bin/env python
"""
Bootloader.py

Boot-time application to check for software updates from USB
"""

## Modules
import os, sys, shutil

## Constants
app_name = "chractor"
usb_path = "/media/sdb"
app_path = os.path.join("/root", app_name)

## Runtime
if __name__ == '__main__':
    mounted = os.path.isdir(usb_path)
    if mounted:
        os.path.exists(usb_path)
        load_path = os.path.join(usb_path, "app")
        app_present = os.path.exists(load_path)
        if app_present:
            shutil.copytree(load_path, app_path)
