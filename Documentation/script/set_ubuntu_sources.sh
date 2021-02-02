#!/bin/bash

version=`lsb_release -sc`
echo "Checking the Ubuntu version"
case $version in
  "trusty" | "xenial" | "bionic" | "focal")
  ;;
  *)
    echo "ERROR: This script will only work on Ubuntu Trusty(14.04) / Xenial(16.04) / Bionic(18.04) / Focal(20.04). Exit."
    exit 0
esac

sudo cp /etc/apt/sources.list /etc/apt/sources.list.backup

# 163
echo "
deb http://mirrors.163.com/ubuntu/ ${version} main restricted universe multiverse
deb http://mirrors.163.com/ubuntu/ ${version}-security main restricted universe multiverse
deb http://mirrors.163.com/ubuntu/ ${version}-updates main restricted universe multiverse
deb http://mirrors.163.com/ubuntu/ ${version}-proposed main restricted universe multiverse
deb http://mirrors.163.com/ubuntu/ ${version}-backports main restricted universe multiverse
deb-src http://mirrors.163.com/ubuntu/ ${version} main restricted universe multiverse
deb-src http://mirrors.163.com/ubuntu/ ${version}-security main restricted universe multiverse
deb-src http://mirrors.163.com/ubuntu/ ${version}-updates main restricted universe multiverse
deb-src http://mirrors.163.com/ubuntu/ ${version}-proposed main restricted universe multiverse
deb-src http://mirrors.163.com/ubuntu/ ${version}-backports main restricted universe multiverse
 " | sudo tee /etc/apt/sources.list

sleep 1
sudo apt-get update
sleep 1
sudo apt-get purge libappstream3
sleep 1
sudo apt-get update
sleep 1

