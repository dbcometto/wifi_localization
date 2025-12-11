import rosbag2_py
from rclpy.serialization import deserialize_message
from wifi_interface.msg import WifiList, WifiMeasurement

from scipy import signal, optimize, integrate
from scipy.spatial.transform import Rotation
from scipy.ndimage import binary_opening, binary_erosion
import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as signal
from geometry_msgs.msg import PoseWithCovarianceStamped

def grab_raw_data(uri = '/home/dbcometto/workspace/wifi_ws/src/wifi_localization/bags/0_0_0_11-17', storage_id='mcap', msg_type = PoseWithCovarianceStamped):
    all_data = []

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(
        uri=uri,
        storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    while reader.has_next():
        msg = reader.read_next()

        if msg[0] == '/pose_estimate':
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

    data_path  = f"{folder_path}/ben_tests/kf_sim_test"
    

    estimate_data = grab_raw_data(uri=data_path)
    

    estimate_time = np.array([x.header.stamp.sec + x.header.stamp.nanosec*1e-9 for x in estimate_data])
    estimate_elapsed_time = estimate_time - min(estimate_time)

    estimate_indices = np.where((start_time <= estimate_elapsed_time) & (estimate_elapsed_time <= stop_time))[0]
    estimate_elapsed_time = estimate_elapsed_time[estimate_indices]

    estimate_x = np.array([x.pose.pose.position.x for x in estimate_data])
    estimate_y = np.array([x.pose.pose.position.x for x in estimate_data])



    output_data = grab_data(uri=data_path)

    output_time = np.array([x.header.stamp.sec + x.header.stamp.nanosec*1e-9 for x in output_data])
    output_elapsed_time = output_time - min(output_time)

    output_indices = np.where((start_time <= output_elapsed_time) & (output_elapsed_time <= stop_time))[0]
    output_elapsed_time = output_elapsed_time[output_indices]

    output_x = np.array([x.pose.pose.position.x for x in output_data])
    output_y = np.array([x.pose.pose.position.x for x in output_data])



    #=====================# Calculations #=====================#


  


    #=====================# Plotting #=====================#

    fig, axs = plt.subplots(figsize=(20,10),nrows=2,ncols=2,layout="constrained")
    # plt.subplots_adjust(bottom=0.5, right=0, top=0)
    fig.suptitle(fig_title)
    x_color = "#CD7A7A"
    y_color = "#0D4019"
    z_color = "#87BAC5"
    extra_color1 = "#2ECCA3"
    extra_color2 = "#4E2745"


    # Raw
    r = 0
    c = 0
    axs[r,c].plot(estimate_elapsed_time,estimate_x,marker=".",linestyle='-',label="New Prediction",zorder=1)
    axs[r,c].plot(output_elapsed_time,output_x,marker=".",linestyle='-',label="Estimate",zorder=1)
    axs[r,c].set_title("X Position over Time")
    axs[r,c].set_xlabel("Time (s)")
    axs[r,c].set_ylabel("Position (grid)")
    axs[r,c].legend(loc="upper right")
    axs[r,c].grid(True,alpha=0.7,zorder=0)
    axs[r,c].set_axisbelow(True)


    # # FFT
    # r = 0
    # c = 1
    # for j in range(np.shape(rssis)[0]):
    #     one_bssid_fft = fft_db[j,:]
    #     axs[r,c].plot(freqs,one_bssid_fft,marker=".",linestyle='-',label="BSSID {j}",zorder=1)

    # axs[r,c].set_title("FFT of RSSIS")
    # axs[r,c].set_xlabel("Frequency (Hz)")
    # axs[r,c].set_ylabel("dB")
    # # axs[r,c].legend(loc="upper left")
    # axs[r,c].grid(True,alpha=0.7,zorder=0)
    # axs[r,c].set_axisbelow(True)
    # axs[r,c].set_ylim(-100)







    # # Raw filt
    # r = 1
    # c = 0
    # for j in range(np.shape(rssis_filt)[0]):
    #     one_bssid_rssi = rssis_filt[j,:]
    #     axs[r,c].plot(elapsed_time,one_bssid_rssi,marker=".",linestyle='-',label="BSSID {j}",zorder=1)

    # axs[r,c].set_title("Filtered RSSIS over Time")
    # axs[r,c].set_xlabel("Time (s)")
    # axs[r,c].set_ylabel("RSSI (%)")
    # # axs[r,c].legend(loc="upper left")
    # axs[r,c].grid(True,alpha=0.7,zorder=0)
    # axs[r,c].set_axisbelow(True)


    # # FFT_filt
    # r = 1
    # c = 1
    # for j in range(np.shape(rssis)[0]):
    #     one_bssid_fft = fft_filt_db[j,:]
    #     axs[r,c].plot(freqs,one_bssid_fft,marker=".",linestyle='-',label="BSSID {j}",zorder=1)

    # axs[r,c].set_title("FFT of Filtered RSSIS")
    # axs[r,c].set_xlabel("Frequency (Hz)")
    # axs[r,c].set_ylabel("dB")
    # # axs[r,c].legend(loc="upper left")
    # axs[r,c].grid(True,alpha=0.7,zorder=0)
    # axs[r,c].set_axisbelow(True)
    # axs[r,c].set_ylim(-100)


    






#==========================# Data #==========================#

analyze_data(fig_title  = "Kalman Filter Performance",
             start_time = 0,
             stop_time  = 10000)




plt.show()