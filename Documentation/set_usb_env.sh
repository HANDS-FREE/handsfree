#!/bin/bash

sudo cp ./script/60-persistent-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload
