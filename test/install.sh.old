#!/bin/sh
# Chractor Installer
# WARNING: This software makes significant changes to the system behavior
# DISCLAIMER: This software is distributed with no warranty.

INSTALL_PATH="$PWD"
CONFIG_PATH="$PWD/conf"
BIN_PATH="$PWD/bin"
BUILD_PATH="$PWD/build"

# GRUB
read -p "Enable fast GRUB? [y/n] " ans
if [ $ans = y -o $ans = Y -o $ans = yes -o $ans = Yes -o $ans = YES ]
    then
        echo "Updating Custom Grub..."
        cp $CONFIG_PATH/grub /etc/default
        update-grub
fi
if [ $ans = n -o $ans = N -o $ans = no -o $ans = No -o $ans = NO ]
    then
        echo "Aborting..."
fi

# Network Setup
read -p "Setup AP network? [y/n] " ans
if [ $ans = y -o $ans = Y -o $ans = yes -o $ans = Yes -o $ans = YES ]
    then
        echo "Reconfiguring Network Interfaces..."
        apt-get install dnsmasq hostapd -y -qq
        apt-get install firmware-iwlwifi -y -qq
        apt-get install wireless-tools -y -qq
        cp $CONFIG_PATH/interfaces /etc/network
        cp $CONFIG_PATH/hostapd.conf /etc/hostapd/
        cp $CONFIG_PATH/dnsmasq.conf /etc/
fi
if [ $ans = n -o $ans = N -o $ans = no -o $ans = No -o $ans = NO ]
    then
        echo "Aborting..."
fi

# APT Packages
read -p "Update APT dependencies? [y/n] " ans
if [ $ans = y -o $ans = Y -o $ans = yes -o $ans = Yes -o $ans = YES ]
    then
        echo "Installing APT dependencies ..."
        cp $CONFIG_PATH/sources.list /etc/apt/
        apt-get update
        apt-get upgrade -y -qq
fi
if [ $ans = n -o $ans = N -o $ans = no -o $ans = No -o $ans = NO ]
    then
        echo "Aborting..."
fi

# Pip Packages
read -p "Update Python modules? [y/n] " ans
if [ $ans = y -o $ans = Y -o $ans = yes -o $ans = Yes -o $ans = YES ]
    then
        echo "Installing Python modules..."
        pip install -r requirements.txt
fi
if [ $ans = n -o $ans = N -o $ans = no -o $ans = No -o $ans = NO ]
    then
        echo "Aborting..."
fi

# Create Daemon
read -p "Start Daemon on boot? [y/n] " ans
if [ $ans = y -o $ans = Y -o $ans = yes -o $ans = Yes -o $ans = YES ]
    then
        echo "Adding daemon to init.d"
        cp $CONFIG_PATH/app /etc/init.d/
        chmod +x /etc/init.d/chractor
        chown root:root /etc/init.d/app
        update-rc.d app defaults
        update-rc.d app enable
fi
if [$ans = n -o $ans = N -o $ans = no -o $ans = No -o $ans = NO ]
    then
        echo "Skipping..."
fi

# Done Message
read -p "Installation Complete! Reboot now? [y/n] " ans
if [ $ans = y -o $ans = Y -o $ans = yes -o $ans = Yes -o $ans = YES ]
    then
    echo "Rebooting..."
    reboot
fi
if [ $ans = n -o $ans = N -o $ans = no -o $ans = No -o $ans = NO ]
    then
        echo "Aborting..."
fi

