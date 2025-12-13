import rosbag2_py
from rclpy.serialization import deserialize_message
from wifi_interface.msg import WifiList, WifiMeasurement, WifiPosition

from scipy import signal, optimize, integrate
from scipy.spatial.transform import Rotation
from scipy.ndimage import binary_opening, binary_erosion
import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as signal
from geometry_msgs.msg import PoseWithCovarianceStamped

def grab_raw_data(uri = '/home/dbcometto/workspace/wifi_ws/src/wifi_localization/bags/0_0_0_11-17', storage_id='mcap', msg_type = WifiPosition):
    all_data = []

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(
        uri=uri,
        storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    while reader.has_next():
        msg = reader.read_next()

        if msg[0] == '/wifi_kf_position':
            data = deserialize_message(msg[1],msg_type)
            all_data.append(data)

    return all_data


def grab_data(uri = '/home/dbcometto/workspace/wifi_ws/src/wifi_localization/bags/0_0_0_11-17', storage_id='mcap', msg_type = PoseWithCovarianceStamped):
    all_data = []

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(
        uri=uri,
        storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    while reader.has_next():
        msg = reader.read_next()

        if msg[0] == '/pose':
            data = deserialize_message(msg[1],msg_type)
            all_data.append(data)

    return all_data

    





#==========================# Function #==========================#
def analyze_data(folder_path = '/home/dbcometto/workspace/wifi_ws/src/wifi_localization/bags',
                 fig_title  = "Default Dataset 0_0",
                 start_time = 0,
                 stop_time  = 1000):
    
    print(f"\n\n\n#=====# For: {fig_title} #=====#\n")


    fig1, axs1 = plt.subplots(figsize=(20,10),nrows=2,ncols=2,layout="constrained")
    # plt.subplots_adjust(bottom=0.5, right=0, top=0)
    fig1.suptitle(f"{fig_title}: x")

    fig2, axs2 = plt.subplots(figsize=(20,10),nrows=2,ncols=2,layout="constrained")
    # plt.subplots_adjust(bottom=0.5, right=0, top=0)
    fig2.suptitle(f"{fig_title}: y")

    x_color = "#CD7A7A"
    y_color = "#0D4019"
    z_color = "#87BAC5"
    extra_color1 = "#2ECCA3"
    extra_color2 = "#4E2745"

    test_x_list = ["0","0","1","2p5"]
    test_y_list = ["2","2p5","0","2"]

    for test_index in range(len(test_x_list)):
        test_x = test_x_list[test_index]
        test_y = test_y_list[test_index]

        data_path  = f"{folder_path}/final_test/{test_x}_{test_y}_test"

        try:
            true_x = int(test_x)
        except:
            true_x = 2.5

        try:
            true_y = int(test_y)
        except:
            true_y = 2.5
        

        estimate_data = grab_raw_data(uri=data_path)
        

        estimate_time = np.array([x.header.stamp.sec + x.header.stamp.nanosec*1e-9 for x in estimate_data])
        estimate_elapsed_time = estimate_time - min(estimate_time)

        estimate_indices = np.where((start_time <= estimate_elapsed_time) & (estimate_elapsed_time <= stop_time))[0]
        estimate_elapsed_time = estimate_elapsed_time[estimate_indices]

        estimate_x = np.array([x.x for x in estimate_data])
        estimate_y = np.array([x.y for x in estimate_data])



        output_data = grab_data(uri=data_path)

        output_time = np.array([x.header.stamp.sec + x.header.stamp.nanosec*1e-9 for x in output_data])
        output_elapsed_time = output_time - min(output_time)

        output_indices = np.where((start_time <= output_elapsed_time) & (output_elapsed_time <= stop_time))[0]
        output_elapsed_time = output_elapsed_time[output_indices]

        output_x = np.array([x.pose.pose.position.x for x in output_data])
        output_y = np.array([x.pose.pose.position.y for x in output_data])



        #=====================# Calculations #=====================#


    


        #=====================# Plotting #=====================#

    

        # x
        r = test_index // 2
        c = test_index % 2
        axs1[r,c].plot(estimate_elapsed_time,estimate_x,marker=".",linestyle='-',label="New Prediction",zorder=1)
        axs1[r,c].plot(output_elapsed_time,output_x,marker=".",linestyle='-',label="Estimate",zorder=1)
        axs1[r,c].plot([min(output_elapsed_time),max(output_elapsed_time)],[true_x,true_x],marker="",linestyle='--',label="True Position",zorder=1)

        axs1[r,c].set_title(f"x Position over Time for Trial {test_index}")
        axs1[r,c].set_xlabel("Time (s)")
        axs1[r,c].set_ylabel("Position (grid)")
        axs1[r,c].legend(loc="upper right")
        axs1[r,c].grid(True,alpha=0.7,zorder=0)
        axs1[r,c].set_axisbelow(True)
        axs1[r,c].set_ylim(0,3)

        # y
        r = test_index // 2
        c = test_index % 2
        axs2[r,c].plot(estimate_elapsed_time,estimate_y,marker=".",linestyle='-',label="New Prediction",zorder=1)
        axs2[r,c].plot(output_elapsed_time,output_y,marker=".",linestyle='-',label="Estimate",zorder=1)
        axs2[r,c].plot([min(output_elapsed_time),max(output_elapsed_time)],[true_y,true_y],marker="",linestyle='--',label="True Position",zorder=1)

        axs2[r,c].set_title(f"y Position over Time for Trial {test_index}")
        axs2[r,c].set_xlabel("Time (s)")
        axs2[r,c].set_ylabel("Position (grid)")
        axs2[r,c].legend(loc="upper right")
        axs2[r,c].grid(True,alpha=0.7,zorder=0)
        axs2[r,c].set_axisbelow(True)
        axs2[r,c].set_ylim(0,3)




    






#==========================# Data #==========================#

analyze_data(fig_title  = "Whole System Performance",
             start_time = 0,
             stop_time  = 10000)




plt.show()