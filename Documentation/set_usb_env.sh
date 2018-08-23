#!/bin/bash

sudo cp ./handsfree-serial.rules /etc/udev/rules.d/
sudo service udev reload
sudo service udev restart
