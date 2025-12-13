# wifi_localization
Repo for EECE 5554 RSN Final Project


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
