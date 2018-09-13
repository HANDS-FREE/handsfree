#!/bin/bash

sudo cp ./script/handsfree-serial.rules /etc/udev/rules.d/
sudo service udev reload
sudo service udev restart

sudo usermod -a -G dialout $USER

