#!/bin/bash

sudo cp ./script/handsfree-serial.rules /etc/udev/rules.d/
sudo cp ./orbbec/56-orbbec-usb.rules /etc/udev/rules.d/56-orbbec-usb.rules
sudo service udev reload
sudo service udev restart

sudo usermod -a -G dialout $USER

