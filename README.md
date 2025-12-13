# wifi_localization
Repo for EECE 5554 RSN Final Project


| Package          | Description                                |  Owner |
| ---------------- | ------------------------------------------ | ------ |
| `wifi_driver`    | Reads from wifi card                       | Jack   |
| `wifi_filter`    | LPF and Kalman Filter                      | Ben    |
| `wifi_interface` | Custom wifi localization messages          | All    |
| `wifi_launch`    | Launch files for full system               | All    |
| `wifi_predict`   | Predicts position from wifi data           | Ananda |    


Note that Scikit Learn is required for the `analysis/predictor.py` script.

# Running this repo

To run this repo on collected data, please create your workspace `.../workspace/`

Place this repo in the source folder folder

`mkdir src`

`cd src`

`git clone ...`

Once cloned, please move the `analysis` and `bags` folders up one directory to the main workspace folder

`mv bags/ analysis/ ..`

Then you shoul dbe able to build and run the repo!