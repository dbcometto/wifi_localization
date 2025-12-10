import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
from rosbags.highlevel import AnyReader

##################################
######=-  Helper Stuff  -=########
##################################


class NetworkInfo:
    pass

class NetworkInfo:
    def __init__(self, bssid: str, rssi: int, variance: float) -> NetworkInfo:
        self.bssid    = bssid
        self.rssi     = rssi
        self.variance = variance

    def __repr__(self):
        return(f"BSSID:{self.bssid} RSSI:{self.rssi} VAR:{self.variance}")
    
    def __eq__(self, other: NetworkInfo):
        if self.bssid == other.bssid:
            return True
        else:
            return False
        
    def __key(self):
        return((self.bssid, self.rssi, self.variance))
        
    def __hash__(self):
        return(hash(self.__key))

def bag_to_networks(bag_filepath: str) -> list[list[NetworkInfo]]:
    wifi_pings = []

    with AnyReader([Path(bag_filepath)]) as reader:
        connections = [target_topic for target_topic in reader.connections if target_topic.topic == "/wifi"]
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            wifi_pings.append([NetworkInfo(network.bssid, network.rssi, network.variance) for network in msg.measurements])

    return wifi_pings

def simplify_bag(bag: list[list[NetworkInfo]]) -> list[NetworkInfo]:
    aggregate_data = {}
    network_avgs = []

    for ping in bag:
        for network in ping:
            if (network.bssid not in aggregate_data.keys()):
                aggregate_data[network.bssid] = [[],[]]
            aggregate_data[network.bssid][0].append(network.rssi)
            aggregate_data[network.bssid][1].append(network.variance)
    
    for bssid in aggregate_data.keys():
        network_avgs.append(NetworkInfo(bssid, np.mean(aggregate_data[bssid][0]), np.mean(aggregate_data[bssid][1])))

    return network_avgs

def get_dc_matrix(simplified: bool) -> list[list[list[NetworkInfo]]]|list[list[list[list[NetworkInfo]]]]:
    matrix = []

    for x in range(4):
        row = []
        for y in range(4):
            value = bag_to_networks(f"../bags/data_collection_2/{x}_{y}_dc2")
            if simplified:
                value = simplify_bag(value)
            row.append(value)
        matrix.append(row)

    return matrix

def compare_simp_bags(bags: list[list[NetworkInfo]], value_to_compare: str):
    if len(bags) == 0:
        raise ValueError("Length of 'bags' passed into compare_simp_bags is 0!")

    comp_lambda = None
    is_reversed = False
    if value_to_compare == "rssi":
        comp_lambda = lambda rnet: rnet[0].rssi
        is_reversed = True
    elif value_to_compare == "variance":
        comp_lambda = lambda rnet: rnet[0].variance
    else:
        raise ValueError("Value of 'value_to_compare' passed into compare_simp_bags is not 'rssi' or 'variance'")

    net_names = []
    for bag in bags:
        names = []
        for network in bag:
            names.append(network.bssid)
        net_names.append(names)
    
    names_int = set(net_names[0])
    for names in net_names:
        names_int = names_int & set(names)

    trimmed_bags = []
    for bag in bags:
        trimmed_bag = []
        for network in names_int:
            for bssid in bag:
                if bssid.bssid == network:
                    trimmed_bag.append(bssid)
        trimmed_bags.append(sorted(trimmed_bag, key=lambda net: net.bssid, reverse=True))
    
    sorted_trimmed_bags = []
    for bag in trimmed_bags:
        _, sorted_bag = map(list, zip(*sorted(zip(trimmed_bags[0], bag), key=comp_lambda, reverse=is_reversed)))
        sorted_trimmed_bags.append(sorted_bag)

    return sorted_trimmed_bags

def prep_comp_data(comp_bags: list[list[NetworkInfo]]):
    heatmap_matrix = []
    for bag in comp_bags:
        heatmap_row = []
        for network in bag:
            heatmap_row.append(float(network.rssi))
        heatmap_matrix.append(heatmap_row)
    
    heatmap_matrix = np.array(heatmap_matrix)

    diff_heatmap_matrix = []
    for i in range(len(heatmap_matrix)):
        j = i - 1
        if i == 0:
            j = 0
        diff_heatmap_matrix.append(heatmap_matrix[i] - heatmap_matrix[j])

    diff_heatmap_matrix = np.array(diff_heatmap_matrix)

    network_labels = []
    for network in comp_bags[0]:
        network_labels.append(network.bssid)

    return (heatmap_matrix, diff_heatmap_matrix, network_labels)

def generate_heatmap(heatmap_data, y_labels, title, x_label, y_label, cbar_label, cmap):
    fig, ax = plt.subplots(figsize=(15, 5))

    # Plot the heatmap
    im = ax.imshow(heatmap_data, aspect=10.0, cmap=cmap)

    # Create colorbar
    cbar = ax.figure.colorbar(im, ax=ax)
    cbar.ax.set_ylabel(cbar_label, rotation=-90, va="bottom")

    # Show all ticks and label them with the respective list entries.
    # ax.set_xticks(range(heatmap_data.shape[1]), labels=x_labels,
    #               rotation=-30, ha="right", rotation_mode="anchor")
    ax.set_yticks(range(heatmap_data.shape[0]), labels=y_labels)

    # Let the horizontal axes labeling appear on top.
    ax.tick_params(top=True, bottom=False,
                   labeltop=True, labelbottom=False)

    # Turn spines off and create white grid.
    ax.spines[:].set_visible(False)

    ax.set_xticks(np.arange(heatmap_data.shape[1]+1)-.5, minor=True)
    ax.set_yticks(np.arange(heatmap_data.shape[0]+1)-.5, minor=True)
    ax.set_title(title)
    ax.set_xlabel(x_label)
    ax.set_ylabel(y_label)
    ax.tick_params(which="minor", bottom=False, left=False)

    return im, cbar


################################
######=-  Main Method  -=#######
################################

def main():
    data = get_dc_matrix(simplified=True)
    rssi_x_comp_data = compare_simp_bags([data[0][0],data[1][0],data[2][0],data[3][0]], "rssi")
    rssi_x_heatmap, rssi_x_diff_heatmap, bssid_labels = prep_comp_data(rssi_x_comp_data)
    rssi_y_comp_data = compare_simp_bags([data[0][0],data[0][1],data[0][2],data[0][3]], "rssi")
    rssi_y_heatmap, rssi_y_diff_heatmap, bssid_labels = prep_comp_data(rssi_y_comp_data)
    var_x_comp_data = compare_simp_bags([data[0][0],data[1][0],data[2][0],data[3][0]], "variance")
    var_x_heatmap, var_x_diff_heatmap, bssid_labels = prep_comp_data(var_x_comp_data)
    var_y_comp_data = compare_simp_bags([data[0][0],data[0][1],data[0][2],data[0][3]], "variance")
    var_y_heatmap, var_y_diff_heatmap, bssid_labels = prep_comp_data(var_y_comp_data)


    generate_heatmap(rssi_x_heatmap, ["0", "1", "2", "3"], "Average RSSI Values Across X Axis", "Networks Sorted by RSSI Strength", "X Position", "RSSI", "viridis")
    generate_heatmap(rssi_y_heatmap, ["0", "1", "2", "3"], "Average RSSI Values Across Y Axis", "Networks Sorted by RSSI Strength", "Y Position", "RSSI", "viridis")
    generate_heatmap(var_x_heatmap,  ["0", "1", "2", "3"], "Average RSSI Values Across X Axis", "Networks Sorted by RSSI Variance", "X Position", "RSSI", "viridis") 
    generate_heatmap(var_y_heatmap,  ["0", "1", "2", "3"], "Average RSSI Values Across Y Axis", "Networks Sorted by RSSI Variance", "Y Position", "RSSI", "viridis")
   
    generate_heatmap(rssi_x_diff_heatmap, ["0", "1", "2", "3"], "Difference of Average RSSI Values Across X Axis", "Networks Sorted by RSSI Strength", "X Position", "RSSI Differnece", "seismic")
    generate_heatmap(rssi_y_diff_heatmap, ["0", "1", "2", "3"], "Difference of Average RSSI Values Across Y Axis", "Networks Sorted by RSSI Strength", "Y Position", "RSSI Differnece", "seismic")
    generate_heatmap(var_x_diff_heatmap,  ["0", "1", "2", "3"], "Difference of Average RSSI Values Across X Axis", "Networks Sorted by RSSI Variance", "X Position", "RSSI Differnece", "seismic") 
    generate_heatmap(var_y_diff_heatmap,  ["0", "1", "2", "3"], "Difference of Average RSSI Values Across Y Axis", "Networks Sorted by RSSI Variance", "Y Position", "RSSI Differnece", "seismic")

    plt.show()

if __name__ == "__main__":
    main()