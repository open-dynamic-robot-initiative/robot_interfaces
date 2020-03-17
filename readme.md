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

## Installing Dependencies

### Steps to set up the pylon interface

1. Get the pylon Camera Software Suite (5.0.12) from [here](https://www.baslerweb.com/en/sales-support/downloads/software-downloads/pylon-5-0-12-linux-x86-64-bit/). Then follow the instructions in the INSTALL file of the pylon suite. Pasted below for reference (2, 3, 4):-

2. Change to the directory which contains this INSTALL file, e.g.:
       cd ~/pylon-5.0.0.1234-x86

3. Extract the corresponding SDK into /opt
       sudo tar -C /opt -xzf pylonSDK*.tar.gz

4. Install udev-rules to set up permissions for basler USB cameras
       ./setup-usb.sh

### Steps to access pylon from inside the container

```bash
singularity shell -B /opt/pylon5 containers/blmc_ei.sif
```

-B binds the directory /opt/pylon5 to the container so that the directories and the files inside it are accessible from within the container

### Exceptions and how to deal with them

In case of a runtime exception, use a smaller USB 3.0 cable.
