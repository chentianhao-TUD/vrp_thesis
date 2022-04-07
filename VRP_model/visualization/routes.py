import os
import read_xml
import csv
import re
import matplotlib.pyplot as plt
import networkx as nx
import math


N_FIELDS = 5
N_VEHICLES = 2
N_ROBOTS = 7
CONFIGURATION = "-".join([str(N_FIELDS), str(N_VEHICLES), str(N_ROBOTS), "100"])
INSTANCE = 0
ITERATION = "0"
VEHICLE_CAPACITY = 2
XML_FILE_NAME = CONFIGURATION + ".xml"
SOLUTION_FILE_NAME = "CRVRP_" + CONFIGURATION + "_" +str(INSTANCE) + "_" + ITERATION + ".SOL"
if VEHICLE_CAPACITY * N_VEHICLES >= N_ROBOTS:
    START_DEPOTS_ROBOTS = []
else:
    n_start_depots_robots = math.ceil((N_ROBOTS - VEHICLE_CAPACITY * N_VEHICLES) / VEHICLE_CAPACITY)
    START_DEPOTS_ROBOTS = list(range(1, n_start_depots_robots + 1))
n_START_DEPOTS_ROBOTS = len(START_DEPOTS_ROBOTS)

def get_actual_node_number_nDepotEQnRobots(node):
    if N_ROBOTS < node <= N_ROBOTS + N_FIELDS:
        actual_node = node - N_ROBOTS
    elif N_ROBOTS + N_FIELDS < node <= N_ROBOTS + N_FIELDS * 2:
        actual_node = node - N_ROBOTS - N_FIELDS
    else:
        actual_node = 0
    return actual_node


def get_actual_node_number(node):
    if n_START_DEPOTS_ROBOTS < node <= n_START_DEPOTS_ROBOTS + N_FIELDS:
        actual_node = node - n_START_DEPOTS_ROBOTS
    elif n_START_DEPOTS_ROBOTS + N_FIELDS < node <= n_START_DEPOTS_ROBOTS + N_FIELDS * 2:
        actual_node = node - n_START_DEPOTS_ROBOTS - N_FIELDS
    else:
        actual_node = 0
    return actual_node


def get_vars():
    path_solution = os.path.join("solutions", SOLUTION_FILE_NAME)
    with open(path_solution, 'r', newline='') as solution_csv_file:
        csv_reader = csv.reader(solution_csv_file, delimiter=" ")

        x = {}
        w = {}
        service_time = {}
        start_service = {}

        for row in csv_reader:
            if 'x' in row[0] and int(round(float(row[1]))) > 0.1:
                x[tuple(int(s) for s in re.findall(r'\d+', row[0]))] = int(round(float(row[1])))
            elif 'w' in row[0] and int(round(float(row[1]))) > 0.1:
                w[tuple(int(round(float(s))) for s in re.findall(r'\d+', row[0]))] = int(round(float(row[1])))
            elif 'service_time' in row[0] and float(row[1]) > 0.1:
                service_time[tuple(int(round(float(s))) for s in re.findall(r'\d+', row[0]))[0]] = float(row[1])
            elif 'start_service' in row[0] and float(row[1]) > 0.1:
                start_service[tuple(int(float(s)) for s in re.findall(r'\d+', row[0]))[0]] = float(row[1])

        return x, w, service_time, start_service


def main(node_number):
    path_input = os.path.join("instances", str(N_FIELDS), XML_FILE_NAME)
    path_solution = os.path.join("solutions", SOLUTION_FILE_NAME)
    Nodes, demand, mode_amount, loc_x, loc_y, SetOfHarvesters, SetOfTransporters = read_xml.read_xml_file(
        path_input, INSTANCE)

    shift = 0
    locX = [loc_x[0]] * (n_START_DEPOTS_ROBOTS + 1) + loc_x[1:-1] + [x + shift for x in loc_x[1:-1]]  + [loc_x[-1] + shift] * (n_START_DEPOTS_ROBOTS + 1)
    locY = [loc_y[0]] * (n_START_DEPOTS_ROBOTS + 1) + loc_y[1:-1] + [x + shift for x in loc_y[1:-1]]  + [loc_y[-1] + shift] * (n_START_DEPOTS_ROBOTS + 1)

    x, w, service_time, start_service = get_vars()
    x_arcs = list(x.keys())
    w_arcs = list(w.keys())


    all_nodes = list(range(0, 2 * n_START_DEPOTS_ROBOTS + 2 * N_FIELDS + 2))

    pos_value = [(locX[i], locY[i]) for i in all_nodes]
    pos = {i: pos_value[i] for i in all_nodes}

    plt.figure(1, dpi=600, figsize=(20, 20))
    fig, axes = plt.subplots(nrows=1, ncols=2)

    G1 = nx.MultiDiGraph()
    G1.add_nodes_from(all_nodes)
    G1.add_edges_from(w_arcs)

    annotate_set = set()
    for i in all_nodes:
        if i not in annotate_set:
            if i != 11:
                axes[0].annotate(get_actual_node_number(i), (locX[i]-1.5, locY[i]-1.5), fontsize=8)
                annotate_set.add(get_actual_node_number(i))

    nx.draw_networkx_edge_labels(G1, pos, label_pos=0.3, edge_labels=w, font_size=6, ax=axes[0])
    nx.draw_networkx_edges(G1, pos, width=2, arrowsize=8, ax=axes[0])
    nx.draw_networkx_nodes(G1, pos, node_size=80, ax=axes[0])


    G2 = nx.MultiDiGraph()
    G2.add_nodes_from(all_nodes)
    G2.add_edges_from(x_arcs)

    annotate_set = set()
    for i in all_nodes:
        if i not in annotate_set:
            axes[1].annotate(get_actual_node_number(i), (locX[i]-1.5, locY[i]-1.5), fontsize=8)
            annotate_set.add(get_actual_node_number(i))

    nx.draw_networkx_edge_labels(G2, pos, label_pos=0.3, edge_labels=x, font_size=5, ax=axes[1])
    nx.draw_networkx_edges(G2, pos, width=2, arrowsize=8, ax=axes[1])
    nx.draw_networkx_nodes(G2, pos, node_size=80, ax=axes[1])
    plt.savefig('robot_route.png', dpi=600)


if __name__ == '__main__':
    os.chdir('..')
    main(N_FIELDS)