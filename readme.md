# Robot interfaces

## What it is

Interface to robot hardware

## Authors

Manuel Wuethrich

Felix Widmaier

## Copyrights

Copyright(c) 2018-2019 Max Planck Gesellschaft

## License

BSD 3-Clause License

## Steps to set up the pylon interface

1. Get the pylon Camera Software Suite (5.0.12) from [here](https://www.baslerweb.com/en/sales-support/downloads/software-downloads/pylon-5-0-12-linux-x86-64-bit/). Then follow the instructions in the INSTALL file of the pylon suite. Pasted below for reference (2, 3, 4):-

2. Change to the directory which contains this INSTALL file, e.g.:
       cd ~/pylon-5.0.0.1234-x86

3. Extract the corresponding SDK into /opt
       sudo tar -C /opt -xzf pylonSDK*.tar.gz

4. Install udev-rules to set up permissions for basler USB cameras
       ./setup-usb.sh

5. Now we need the python wrapper of this suite, which is from [here](https://github.com/basler/pypylon).

6. Install swig following the instructions [here](https://github.com/swig/swig/wiki/Getting-Started) (condensed below), and also run the tests for python. (the swig dir should be extracted in a location from where sudo has permission to copy files)

    ```bash
    sudo apt-get install build-essential libpcre3-dev
    tar -zxf swig-4.0.1.tar.gz
    cd swig-4.0.1
    ./configure && make && make check
    sudo make install
    ```

7. Then in your home directory (say),

    ```bash
    git clone https://github.com/basler/pypylon.git
    cd pypylon
    pip3 install .
    ```

### Steps to access pylon from inside the container

```bash
singularity shell -B /opt/pylon5 containers/blmc_ei.sif
```

### Steps to setup the webcam

Just connect to a USB 3 port and install guvcview following the instructions [here](http://ubuntuhandbook.org/index.php/2018/10/guvcview-2-0-6-released-install-ubuntu/).

### General productivity instructions

Touch a CATKIN_IGNORE in the following packages-
       * dbot
       * dbot_ros
       * fl
       * dbot_ros_msgs
       * blmc_ros_msgs
