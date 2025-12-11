import rosbag2_py
from rclpy.serialization import deserialize_message
from wifi_interface.msg import WifiList, WifiMeasurement

from scipy import signal, optimize, integrate
from scipy.spatial.transform import Rotation
from scipy.ndimage import binary_opening, binary_erosion
import numpy as np
import matplotlib.pyplot as plt
import allantools as at

def grab_raw_wifi_data(uri = '/home/dbcometto/workspace/wifi_ws/src/wifi_localization/bags/0_0_0_11-17', storage_id='mcap', msg_type = WifiList):
    all_data = []

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(
        uri=uri,
        storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    while reader.has_next():
        msg = reader.read_next()

        if msg[0] == '/wifi':
            data = deserialize_message(msg[1],msg_type)
            all_data.append(data)

    return all_data



def collect_data(data_path='/home/dbcometto/workspace/wifi_ws/src/wifi_localization/bags/0_0_0_11-17',start_time = 0, stop_time =100):
    my_data = grab_raw_wifi_data(data_path)

    time = np.array([x.header.stamp.sec + x.header.stamp.nanosec*1e-9 for x in my_data])
    elapsed_time = time - min(time)

    indices = np.where((start_time <= elapsed_time) & (elapsed_time <= stop_time))[0]
    elapsed_time = elapsed_time[indices]


    wifi_data = []
    for x in my_data:
        wifi_data.append({mx_list.bssid: (mx_list.rssi,mx_list.variance) for mx_list in x.measurements})

    

    # Data collection points
    steps = len(wifi_data)


    # Count and Collect BSSIDs

    unique_bssids = set()
    for x in wifi_data:
        unique_bssids.update(key for key in x.keys())

    unique_bssid_list = sorted(unique_bssids)
    unique_bssid_count = len(unique_bssid_list)
    
    
    # Organize into numpy array
    rssis = np.full((unique_bssid_count,steps),np.nan)

    for i in range(unique_bssid_count):
        for step in range(steps):
            try:
                rssis[i,step] = float(wifi_data[step][unique_bssid_list[i]][0])
            except:
                continue

    return elapsed_time,wifi_data,unique_bssid_list,unique_bssid_count,rssis

    





#==========================# Function #==========================#
def analyze_data(location_0 = '0_0_0',
                 location_1 = '7_0_0',
                 date = "11-17",
                 folder_path = '/home/dbcometto/workspace/wifi_ws/src/wifi_localization/bags',
                 fig_title  = "Default Dataset 0_0_0",
                 start_time = 0,
                 stop_time  = 100):
    
    print(f"\n\n\n#=====# For: {fig_title} #=====#\n")

    data_path_0  = f"{folder_path}/{location_0}_{date}"
    data_path_1  = f"{folder_path}/{location_1}_{date}"
    

    elapsed_time_0,wifi_data_0,unique_bssid_list_0,unique_bssid_count_0,rssis_0 = collect_data(data_path=data_path_0, start_time=start_time, stop_time=stop_time)


    # print(f"Data 0: {wifi_data_0[0]}")
    # print(f"# of Unique BSSIDs: {unique_bssid_count_0}")
    # print(f"rssis: {rssis_0}")


    elapsed_time_1,wifi_data_1,unique_bssid_list_1,unique_bssid_count_1,rssis_1 = collect_data(data_path=data_path_1, start_time=start_time, stop_time=stop_time)







    #=====================# Raw Comparison Plot #=====================#


    bssid_to_index_1 = {bssid: i for i, bssid in enumerate(unique_bssid_list_1)}

    # Build a mapping from index in 0 -> index in 1
    index_mapping = {}
    for i, bssid in enumerate(unique_bssid_list_0):
        if bssid in bssid_to_index_1:
            index_mapping[i] = bssid_to_index_1[bssid]


    # print(f"index_mapping: {index_mapping}")

    num_bssis = min(10,len(index_mapping))

    indices_0,indices_1 = zip(*index_mapping.items())

    # print(f"0 indices: {indices_0}")
    # print(f"1 indices: {indices_1}")




    #=====================# FFT #=====================#
    fs = 0.2
    k = len(rssis_0)
    df = fs/k
    freqs_0 = df*np.arange(k)
    fft_0 = np.abs(np.fft.fft(rssis_0))
    fft_db_0 = 20*np.log10(fft_0/np.max(fft_0,axis=1)[:,None])





    fig, axs = plt.subplots(figsize=(20,10),nrows=2,ncols=2,layout="constrained")
    # plt.subplots_adjust(bottom=0.5, right=0, top=0)
    fig.suptitle(fig_title)
    x_color = "#CD7A7A"
    y_color = "#0D4019"
    z_color = "#87BAC5"
    extra_color1 = "#2ECCA3"
    extra_color2 = "#4E2745"




    # Raw Comparison
    r = 0
    c = 0
    for i in range(num_bssis):
        j = indices_0[i]
        axs[r,c].plot(elapsed_time_0,rssis_0[j],marker="^",linestyle='-',label=f"0 - {unique_bssid_list_0[j]}",zorder=1)

    for i in range(num_bssis):
        j = indices_1[i]
        axs[r,c].plot(elapsed_time_1,rssis_1[j],marker=".",linestyle='--',label=f"1 - {unique_bssid_list_1[j]}",zorder=1)

    axs[r,c].set_title("RSSI by BSSID Compared")
    axs[r,c].set_xlabel("Elpased Time")
    axs[r,c].set_ylabel("RSSI")
    # axs[r,c].legend(loc="upper left")
    axs[r,c].grid(True,alpha=0.7,zorder=0)
    axs[r,c].set_axisbelow(True)




    # FFT of 0
    r = 0
    c = 1
    axs[r,c].plot(freqs_0,fft_db_0,marker=".",linestyle='--',label="RSSIs 0",zorder=1)

    axs[r,c].set_title("FFT of RSSIS for 0")
    axs[r,c].set_xlabel("Frequency (Hz)")
    axs[r,c].set_ylabel("dB")
    # axs[r,c].legend(loc="upper left")
    axs[r,c].grid(True,alpha=0.7,zorder=0)
    axs[r,c].set_axisbelow(True)

    





    






#==========================# Data #==========================#

analyze_data(location_0 = '0_0_0',
             location_1 = '7_0_0',
             fig_title  = "0_0_0 Data vs 7_0_0 Data",
             start_time = 0,
             stop_time  = 10000)




plt.show()