# Localization via Wi-Fi through RSSI/BSSID Pair Training
This repo contains a variety of ROS2 packages for a system developed for Northeastern University's
ECEE 5554 - Robotic Sensing and Navigation class.  This was developed using ROS2 Jazzy.

The system serves to predict grid position based on collected WiFi RSSI-BSSID pairs after
an initial data collection and training.


| Package          | Description                                |  Owner |
| ---------------- | ------------------------------------------ | ------ |
| `wifi_driver`    | Reads from wifi card                       | Jack   |
| `wifi_filter`    | LPF and Kalman Filter                      | Ben    |
| `wifi_interface` | Custom wifi localization messages          | All    |
| `wifi_launch`    | Launch files for full system               | All    |
| `wifi_predict`   | Predicts position from wifi data           | Ananda |    





## Installation

Begin by creating your workspace such as `.../wifi_ws/`.

Next, place this repo in the source folder.  From the workspace directory, run
```bash
mkdir src && cd src && git clone https://github.com/dbcometto/wifi_localization.git
```

Once cloned, move the `bags` folders up one directory into the main workspace folder
```bash
mv bags/ ..
```

Additionally, certain packages must be installed.  Run
```bash
pip install -r requirements.txt
```
It may be necessary to add `--break-system-packages`, if desired.

Then you should be able to build and run the repo!


## Training the model

Once the data bags are moved into the parent workspace folder, running 
```bash
ros2 launch wifi_predict train.launch.py
```
will train the model.  It is currently set to work on Data Collection 2.



## Starting the System

For regular use, run
```bash
ros2 launch wifi_launch system.launch.py
```

For testing, run
```bash
ros2 launch wifi_launch system.launch.py use_sim:=true
```
and also play the desired bag using
```bash
ros2 bag play path/to/bag
```
