import rosbag2_py
from rclpy.serialization import deserialize_message
from wifi_interface.msg import WifiList, WifiMeasurement

from scipy import signal, optimize, integrate
from scipy.spatial.transform import Rotation
from scipy.ndimage import binary_opening, binary_erosion
import numpy as np
import matplotlib.pyplot as plt
import allantools as at

from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error
from sklearn.model_selection import train_test_split, cross_val_score, LeaveOneOut, LeaveOneGroupOut
from sklearn.preprocessing import StandardScaler


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
def analyze_data(date = "11-17",
                 folder_path = '/home/dbcometto/workspace/wifi_ws/src/wifi_localization/bags',
                 fig_title  = "Default Dataset 0_0_0",
                 start_time = 0,
                 stop_time  = 100):
    
    print(f"\n\n\n#=====# For: {fig_title} #=====#\n")

    point_count = 2
    data_paths = [f"{folder_path}/{x}_0_Classroom_{date}" for x in range(point_count)]
    
    all_data = []
    for path in data_paths:
        all_data.append(collect_data(data_path=path, start_time=start_time, stop_time=stop_time))

    # elapsed_time_0,wifi_data_0,unique_bssid_list_0,unique_bssid_count_0,rssis_0 = all_data[0]
    # print(rssis_0)

    

    bssid_list_list = [data[2] for data in all_data]
    master_bssid_list = sorted(set().union(*bssid_list_list))
    total_bssids = len(master_bssid_list)
    # print(f"Total bssids: {total_bssids}")

    rssis_list = [data[4] for data in all_data]
    avg_rssis_list = [np.nanmean(rssis,axis=1) for rssis in rssis_list]
    # print(f"Count points: {len(rssis_list)}")
    # print(f"Count avg points: {len(avg_rssis_list)}")
    # print(f"First average {avg_rssis_list[0]}")
    
    num_points_list = [rssis.shape[1] for rssis in rssis_list]
    # print(num_points_list)

    total_points = sum(num_points_list)

    # Put average rssi in correct row for bssid
    aligned_avg_rssis = np.full((point_count,total_bssids),0.0)
    for i, (avg_rssis, bssids) in enumerate(zip(avg_rssis_list, bssid_list_list)):
        for rssi, bssid in zip(avg_rssis, bssids):
            idx = master_bssid_list.index(bssid)  # find column index
            aligned_avg_rssis[i, idx] = rssi
    # print(aligned_avg_rssis)


    # Or put rssi in correct row for bssid
    aligned_rssis = np.full((total_points, total_bssids), 0.0)
    row_idx = 0
    for rssis, bssids in zip(rssis_list, bssid_list_list):
        num_samples = rssis.shape[1]  # number of samples (columns)
        for sample_idx in range(num_samples):
            for bssid_idx, bssid in enumerate(bssids):
                cidx = master_bssid_list.index(bssid)  # column in master list
                new_value = rssis[bssid_idx, sample_idx]
                aligned_rssis[row_idx + sample_idx, cidx] = new_value if not np.isnan(new_value) else 0.0
        row_idx += num_samples
    # print(aligned_avg_rssis)


    


    true_pos_avg = np.arange(point_count)
    # print(true_pos)
    true_pos_list = []
    for i,x in enumerate(num_points_list):
        true_pos_list.extend([i]*x)

    true_pos = np.array(true_pos_list)
    # print(true_pos)

    print(f"Data Shapes: data is {aligned_rssis.shape} and truth is {true_pos.shape}")


    


    #=====================# Prediction #=====================#
    X = aligned_rssis
    y = true_pos


    scaler = StandardScaler()
    X_scaled = scaler.fit_transform(X)

    X_train, X_test, y_train, y_test = train_test_split(
    X_scaled, y, 
    test_size=0.2,   # fraction of data to use as test set
    random_state=42, # seed for reproducibility
    shuffle=True     # shuffle before splitting
)

    model = LinearRegression()
    model.fit(X_train, y_train)

    y_pred = model.predict(X_test)
    mse = mean_squared_error(y_test, y_pred)
    print(f"Test/Train Split MSE: {mse:.3f}")

    scores = cross_val_score(model, X_scaled, y, cv=5, scoring='neg_mean_squared_error')
    print(f"5-fold CV MSE: {-np.mean(scores):.3f}")


    loo = LeaveOneOut()
    model = LinearRegression()
    scores = cross_val_score(model, X_scaled, y, cv=loo, scoring='neg_mean_squared_error')

    print(f"LOOCV MSE: {-np.mean(scores):.3f}")



    logo = LeaveOneGroupOut()
    scores = cross_val_score(model, X_scaled, y, cv=logo, groups=y, scoring='neg_mean_squared_error')
    print(f"Leave-One-Position-Out MSE: {-np.mean(scores):.3f}")


    final_model = LinearRegression()
    final_model.fit(X_scaled, y)
    coeffs = final_model.coef_
    # print(f"Final Model: {final_model.coef_}")






    # plt.figure(figsize=(15, 8))
    # plt.imshow(aligned_rssis, aspect='auto', cmap='coolwarm', interpolation='nearest')

    # # Draw horizontal lines to separate groups by truth value
    # for val in np.unique(true_pos):
    #     # find indices of samples with this truth value
    #     indices = np.where(true_pos == val)[0]
    #     # draw line above the first index of the next group
    #     if len(indices) > 0:
    #         plt.axhline(y=indices[0]-0.5, color='black', linewidth=1)

    # plt.colorbar(label='RSSI (Integer)')
    # plt.xlabel('BSSID Index')
    # plt.ylabel('Sample Index')
    # plt.title('RSSI Heatmap Separated by Truth Value')
    




    # coeff_norm = (coeffs - coeffs.min()) / (coeffs.max() - coeffs.min())
    coeff_norm = coeffs/np.max(np.abs(coeffs))


    fig, (ax_heat, ax_overlay) = plt.subplots(2, 1, figsize=(18, 8), 
                                          gridspec_kw={'height_ratios':[4,1]}, sharex=True)

    # ===== Heatmap =====
    im = ax_heat.imshow(aligned_rssis, aspect='auto', cmap='inferno', interpolation='nearest')
    for val in np.unique(true_pos):
        indices = np.where(true_pos == val)[0]
        if len(indices) > 0:
            ax_heat.axhline(y=indices[0]-0.5, color='black', linewidth=1)
    ax_heat.set_ylabel('Sample Index')
    ax_heat.set_title('RSSI Heatmap Separated by Truth Value')
    fig.colorbar(im, ax=ax_heat, label='RSSI (Integer)', fraction=0.03, pad=0.02)

    # ===== Overlay row =====
    overlay_im = ax_overlay.imshow(coeff_norm[np.newaxis, :], aspect='auto', cmap='coolwarm')
    ax_overlay.set_yticks([])
    ax_overlay.set_xlabel('BSSID Index')
    ax_overlay.set_title('Coefficient Strength Overlay')
    fig.colorbar(overlay_im, ax=ax_overlay, label='Normalized Coefficient', fraction=0.03, pad=0.02)







    fig, (ax_heat, ax_overlay) = plt.subplots(2, 1, figsize=(18, 8), 
                                          gridspec_kw={'height_ratios':[4,1]}, sharex=True)

    # ===== Heatmap =====
    im = ax_heat.imshow(np.diff(aligned_rssis,axis=0), aspect='auto', cmap='bwr', interpolation='nearest')
    for val in np.unique(true_pos):
        indices = np.where(true_pos == val)[0]
        if len(indices) > 0:
            ax_heat.axhline(y=indices[0]-0.5, color='black', linewidth=1)
    ax_heat.set_ylabel('Sample Index')
    ax_heat.set_title('Difference in RSSI Heatmap Separated by Truth Value')
    fig.colorbar(im, ax=ax_heat, label='RSSI Change (Integer)', fraction=0.03, pad=0.02)

    # ===== Overlay row =====
    overlay_im = ax_overlay.imshow(coeff_norm[np.newaxis, :], aspect='auto', cmap='coolwarm')
    ax_overlay.set_yticks([])
    ax_overlay.set_xlabel('BSSID Index')
    ax_overlay.set_title('Coefficient Strength Overlay')
    fig.colorbar(overlay_im, ax=ax_overlay, label='Normalized Coefficient', fraction=0.03, pad=0.02)

    plt.tight_layout()





    plt.tight_layout()

    






    # fig, axs = plt.subplots(figsize=(20,10),nrows=2,ncols=2,layout="constrained")
    # # plt.subplots_adjust(bottom=0.5, right=0, top=0)
    # fig.suptitle(fig_title)
    # x_color = "#CD7A7A"
    # y_color = "#0D4019"
    # z_color = "#87BAC5"
    # extra_color1 = "#2ECCA3"
    # extra_color2 = "#4E2745"

    # r = 0
    # c = 0
    # for i in range(num_bssis):
    #     j = indices_0[i]
    #     axs[r,c].plot(elapsed_time_0,rssis_0[j],marker="^",linestyle='-',label=f"0 - {unique_bssid_list_0[j]}",zorder=1)

    # for i in range(num_bssis):
    #     j = indices_1[i]
    #     axs[r,c].plot(elapsed_time_1,rssis_1[j],marker=".",linestyle='--',label=f"1 - {unique_bssid_list_1[j]}",zorder=1)

    # axs[r,c].set_title("RSSI by BSSID Compared")
    # axs[r,c].set_xlabel("Elpased Time")
    # axs[r,c].set_ylabel("RSSI")
    # # axs[r,c].legend(loc="upper left")
    # axs[r,c].grid(True,alpha=0.7,zorder=0)
    # axs[r,c].set_axisbelow(True)

    




#==========================# Data #==========================#

analyze_data(fig_title  = "test",
             start_time = 0,
             stop_time  = 10000)




plt.show()