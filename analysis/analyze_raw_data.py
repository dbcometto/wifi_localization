import rosbag2_py
from rclpy.serialization import deserialize_message
from wifi_interface.msg import WifiList, WifiMeasurement

from scipy import signal, optimize, integrate
from scipy.spatial.transform import Rotation
from scipy.ndimage import binary_opening, binary_erosion
import numpy as np
import matplotlib.pyplot as plt
import allantools as at

def grab_data(uri = '/home/dbcometto/workspace/wifi_ws/src/wifi_localization/bags/0_0_0_11-17', storage_id='mcap', msg_type = WifiList):
    all_data = []

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(
        uri=uri,
        storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    while reader.has_next():
        msg = reader.read_next()

        # TODO: grab correct topics
        if msg[0] == '/wifi':
            data = deserialize_message(msg[1],msg_type)
            all_data.append(data)

    return all_data





#==========================# Function #==========================#
def analyze_data(data_path  = '/home/dbcometto/workspace/wifi_ws/src/wifi_localization/bags/0_0_0_11-17',
                 fig_title  = "Default Dataset 0-0-0",
                 start_time = 0,
                 stop_time  = 100):
    
    print(f"\n\n\n#=====# For: {fig_title} #=====#\n")


    # Grab data
    # Data was recorded in FLU, change to FRD
    my_data = grab_data(data_path)

    time = np.array([x.header.stamp.sec + x.header.stamp.nanosec*1e-9 for x in my_data])
    elapsed_time = time - min(time)

    indices = np.where((start_time <= elapsed_time) & (elapsed_time <= stop_time))[0]
    elapsed_time = elapsed_time[indices]

    # TODO: dict by bssid
    # wifi_data = {}
    # for x in my_data:
    #     bssid,
    #     wifi_data = np.array([x.measurements for x in my_data])[indices]





















    #=====================# Plotting #=====================#

    fig, axs = plt.subplots(figsize=(20,10),nrows=2,ncols=2,layout="constrained")
    # plt.subplots_adjust(bottom=0.5, right=0, top=0)
    fig.suptitle(fig_title)
    x_color = "#CD7A7A"
    y_color = "#0D4019"
    z_color = "#87BAC5"
    extra_color1 = "#2ECCA3"
    extra_color2 = "#4E2745"


    r = 0
    c = 0
    # axs[r,c].scatter(mag_x*scale,mag_y*scale,color=x_color,label="Uncalibrated Data (Scaled)",zorder=1)
    # # axs[r,c].scatter(cal_mag_x_unfixed,cal_mag_y_unfixed,c=elapsed_time,cmap="plasma",label="Calibrated Data Unfixed",zorder=2)
    # # axs[r,c].contour(X, Y, Z, levels=[0], cmap='viridis')
    # sc = axs[r,c].scatter(cal_mag_x,cal_mag_y,c=elapsed_time,label="Calibrated Data",zorder=3)
    axs[r,c].set_title("Magnetic Field")
    axs[r,c].set_xlabel("X")
    axs[r,c].set_ylabel("Y")
    axs[r,c].legend(loc="upper left")
    axs[r,c].grid(True,alpha=0.7,zorder=0)
    axs[r,c].set_axisbelow(True)
    # axs[r,c].set_aspect('equal')
    # axs[r,c].set_xlim(left=-7)

    # cbar = plt.colorbar(sc, ax=axs[r, c])
    # cbar.set_label(f"Elapsed Time (s)")


    # Yaw Heading cal vs uncal
    r = 1
    c = 0
    # axs[r,c].plot(elapsed_time,dead_roll,marker="^",linestyle='-',color=z_color,label="x",zorder=1)
    # axs[r,c].plot(elapsed_time,dead_pitch,marker="s",linestyle='-',color=y_color,label="y",zorder=2)
    # axs[r,c].plot(elapsed_time,np.unwrap(mag_yaw*180/np.pi,period=360),marker=".",linestyle='-',color=x_color,label="Yaw (Calibrated)",zorder=1)
    # axs[r,c].plot(elapsed_time,np.unwrap(mag_yaw_uncal*180/np.pi,period=360),marker=".",linestyle='-',color=y_color,label="Yaw (Uncalibrated)",zorder=2)
    # axs[r,c].plot(elapsed_time,best_yaw,marker=".",linestyle='-',color=z_color,label="z (Best)",zorder=3)
    # axs[r,c].plot(gps_elapsed_time,gps_yaw,marker=".",linestyle='-',label="z (GPS)",zorder=4)
    axs[r,c].set_title("Heading based on Magnetic Field")
    axs[r,c].set_xlabel("Time (s)")
    axs[r,c].set_ylabel("Angle (deg)")
    axs[r,c].legend(loc="upper right")
    axs[r,c].grid(True,alpha=0.7,zorder=0)
    axs[r,c].set_axisbelow(True)













#==========================# Data #==========================#

analyze_data(data_path  = '/home/dbcometto/workspace/wifi_ws/src/wifi_localization/bags/0_0_0_11-17',
             fig_title  = "Specific Data",
             start_time = 0,
             stop_time  = 10000)




plt.show()