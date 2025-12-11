import rosbag2_py
from rclpy.serialization import deserialize_message
from wifi_interface.msg import WifiList, WifiMeasurement

from scipy import signal, optimize, integrate
from scipy.spatial.transform import Rotation
from scipy.ndimage import binary_opening, binary_erosion
import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as signal

from collections import deque

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
def analyze_data(location = '0_0',
                 date = "11-17",
                 folder_path = '/home/dbcometto/workspace/wifi_ws/src/wifi_localization/bags',
                 fig_title  = "Default Dataset 0_0",
                 start_time = 0,
                 stop_time  = 1000):
    
    print(f"\n\n\n#=====# For: {fig_title} #=====#\n")

    data_path  = f"{folder_path}/data_collection_2/{location}_dc2"
    

    elapsed_time,wifi_data,unique_bssid_list,unique_bssid_count,rssis = collect_data(data_path=data_path, start_time=start_time, stop_time=stop_time)

    # print(f"Data 0: {wifi_data_0[0]}")
    # print(f"# of Unique BSSIDs: {unique_bssid_count_0}")
    # print(f"rssis: {rssis}")


    all_elapsed_time = np.vstack([elapsed_time]*np.shape(rssis)[0])
    # print(f"time shape: {all_elapsed_time.shape}")






    #=====================# Calculations #=====================#


    num_rssis = 10
    rssis = rssis[0:num_rssis,:]
    rssis = np.nan_to_num(rssis,0)
    # rssis = rssis - np.mean(rssis, axis=1, keepdims=True)



    # FFT
    fs = 0.2
    k = np.shape(rssis)[1]
    # print(f"k: {k}")
    df = fs/k
    freqs = df*np.arange(k)
    fft = np.abs(np.fft.fft(rssis))
    fft_db = 20*np.log10(fft/np.max(fft,axis=1)[:,None])




    # Filtering

    num_taps = 3
    fc = 0.01
    window = "boxcar"
    taps = signal.firwin(num_taps, fc, window=window, pass_zero=True, fs=fs)
    denominator=1

    # rssis_filt = signal.lfilter(taps,denominator,rssis)
    rssis_filt = np.zeros_like(rssis)


    rssi_ques = [deque(maxlen=num_taps)]*num_rssis

    for h in range(np.shape(rssis)[0]):
        for j in range(np.shape(rssis)[1]):
            if not rssis[h,j] < 0.1:
                rssi_ques[h].append(rssis[h,j])

                rssis_filt[h,j] = sum(tap*rssi for tap,rssi in zip(taps,rssi_ques[h]))


    # FFT filtered
    fft_filt = np.abs(np.fft.fft(rssis_filt))
    fft_filt_db = 20*np.log10(fft_filt/np.max(fft_filt,axis=1)[:,None])












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
    for j in range(np.shape(rssis)[0]):
        one_bssid_rssi = rssis[j,:]
        axs[r,c].plot(elapsed_time,one_bssid_rssi,marker=".",linestyle='-',label="BSSID {j}",zorder=1)

    axs[r,c].set_title("RSSIS over Time")
    axs[r,c].set_xlabel("Time (s)")
    axs[r,c].set_ylabel("RSSI (%)")
    # axs[r,c].legend(loc="upper left")
    axs[r,c].grid(True,alpha=0.7,zorder=0)
    axs[r,c].set_axisbelow(True)


    # FFT
    r = 0
    c = 1
    for j in range(np.shape(rssis)[0]):
        one_bssid_fft = fft_db[j,:]
        axs[r,c].plot(freqs,one_bssid_fft,marker=".",linestyle='-',label="BSSID {j}",zorder=1)

    axs[r,c].set_title("FFT of RSSIS")
    axs[r,c].set_xlabel("Frequency (Hz)")
    axs[r,c].set_ylabel("dB")
    # axs[r,c].legend(loc="upper left")
    axs[r,c].grid(True,alpha=0.7,zorder=0)
    axs[r,c].set_axisbelow(True)
    axs[r,c].set_ylim(-100)







    # Raw filt
    r = 1
    c = 0
    for j in range(np.shape(rssis_filt)[0]):
        one_bssid_rssi = rssis_filt[j,:]
        axs[r,c].plot(elapsed_time,one_bssid_rssi,marker=".",linestyle='-',label="BSSID {j}",zorder=1)

    axs[r,c].set_title("Filtered RSSIS over Time")
    axs[r,c].set_xlabel("Time (s)")
    axs[r,c].set_ylabel("RSSI (%)")
    # axs[r,c].legend(loc="upper left")
    axs[r,c].grid(True,alpha=0.7,zorder=0)
    axs[r,c].set_axisbelow(True)


    # FFT_filt
    r = 1
    c = 1
    for j in range(np.shape(rssis)[0]):
        one_bssid_fft = fft_filt_db[j,:]
        axs[r,c].plot(freqs,one_bssid_fft,marker=".",linestyle='-',label="BSSID {j}",zorder=1)

    axs[r,c].set_title("FFT of Filtered RSSIS")
    axs[r,c].set_xlabel("Frequency (Hz)")
    axs[r,c].set_ylabel("dB")
    # axs[r,c].legend(loc="upper left")
    axs[r,c].grid(True,alpha=0.7,zorder=0)
    axs[r,c].set_axisbelow(True)
    axs[r,c].set_ylim(-100)

    





    






#==========================# Data #==========================#

analyze_data(location = '0_0',
             fig_title  = "Data Before and After Filtering",
             start_time = 0,
             stop_time  = 10000)




plt.show()