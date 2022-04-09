import os
import read_xml
import csv
import re
import matplotlib.pyplot as plt
import networkx as nx
import math
import numpy as np


MODEL = "CRVRP"
N_FIELDS = 10
N_VEHICLES = 3
N_ROBOTS = 7
CONFIGURATION = "-".join([str(N_FIELDS), str(N_VEHICLES), str(N_ROBOTS), "100"])
INSTANCE = 0
ITERATION = "0"
VEHICLE_CAPACITY = 2
XML_FILE_NAME = CONFIGURATION + ".xml"
SOLUTION_FILE_NAME = "Results_CP_VRP_" + CONFIGURATION + "_" + str(INSTANCE) + ".csv" #Results_CP_VRP_5-4-11-100_4

if VEHICLE_CAPACITY * N_VEHICLES >= N_ROBOTS:
    START_DEPOTS_ROBOTS = []
else:
    n_start_depots_robots = math.ceil((N_ROBOTS - VEHICLE_CAPACITY * N_VEHICLES) / VEHICLE_CAPACITY)
    START_DEPOTS_ROBOTS = list(range(1, n_start_depots_robots + 1))
n_START_DEPOTS_ROBOTS = len(START_DEPOTS_ROBOTS)
END_DEPOT = (n_START_DEPOTS_ROBOTS) * 2 + N_FIELDS * 2 + 1


def get_departure_node(i):
    if n_start_depots_robots < i <= n_start_depots_robots + N_FIELDS:
        return i + N_FIELDS
    else:
        return i


def get_actual_node_number_nDepotEQnRobots(node):
    if N_ROBOTS < node <= N_ROBOTS + N_FIELDS:
        actual_node = node - N_ROBOTS
    elif N_ROBOTS + N_FIELDS < node <= N_ROBOTS + N_FIELDS * 2:
        actual_node = node - N_ROBOTS - N_FIELDS
    else:
        actual_node = 0
    return actual_node


def get_distance_matrix(xml_path):
    Nodes, demand, mode_amount, loc_x, loc_y, SetOfVehicles, SetOfRobots = read_xml.read_xml_file(xml_path, INSTANCE)
    n_depot = [0] + list(range(1, n_START_DEPOTS_ROBOTS+1))

    transport_capacity = 2

    start_depot_vehicles = 0
    if transport_capacity * len(SetOfVehicles) >= len(SetOfRobots):
        start_depots_robots = []
    else:
        n_start_depots_robots = math.ceil(
            (len(SetOfRobots) - transport_capacity * len(SetOfVehicles)) / transport_capacity)
        start_depots_robots = list(range(1, n_start_depots_robots + 1))

    start_depots = [start_depot_vehicles] + start_depots_robots

    arrival_nodes = list(range(start_depots[-1] + 1, start_depots[-1] + 1 + len(Nodes)))
    departure_nodes = list(range(arrival_nodes[-1] + 1, arrival_nodes[-1] + 1 + len(Nodes)))
    all_customer_nodes = arrival_nodes + departure_nodes

    end_depots_robots = list(range(len(start_depots) + len(all_customer_nodes),
                                   len(start_depots) + len(all_customer_nodes) + len(start_depots_robots)))
    end_depot_vehicles = len(all_customer_nodes) + len(start_depots_robots) + len(end_depots_robots) + 1
    end_depots = end_depots_robots + [end_depot_vehicles]

    all_nodes = start_depots + all_customer_nodes + end_depots

    extended_Arcs = [(i, j)
                     for i in all_nodes for j in all_nodes]

    locations_x = [loc_x[0]] * len(start_depots) + 2 * loc_x[1:-1] + [loc_x[-1]] * len(end_depots)
    locations_y = [loc_y[0]] * len(start_depots) + 2 * loc_y[1:-1] + [loc_y[-1]] * len(end_depots)

    distances = {(i, j): np.hypot(locations_x[i] - locations_x[j], locations_y[i] - locations_y[j])
                 for i, j in extended_Arcs}

    return distances

def get_actual_node_number(node):
    if n_START_DEPOTS_ROBOTS < node <= n_START_DEPOTS_ROBOTS + N_FIELDS:
        actual_node = node - n_START_DEPOTS_ROBOTS
    elif n_START_DEPOTS_ROBOTS + N_FIELDS < node <= n_START_DEPOTS_ROBOTS + N_FIELDS * 2:
        actual_node = node - n_START_DEPOTS_ROBOTS - N_FIELDS
    else:
        actual_node = 0
    return actual_node


def get_vars():
    path_solution = os.path.join("results", '10', 'CP_VRP', SOLUTION_FILE_NAME)
    with open(path_solution, 'r', newline='') as solution_csv_file:
        csv_reader = csv.reader(solution_csv_file, delimiter=";")

        x = {}
        w = {}
        v = {}
        service_time = {x: 0 for x in range(END_DEPOT + 1)}
        start_service = {x: 0 for x in range(n_start_depots_robots + 1)}

        for row in csv_reader:
            if 'x' in row[0] and int(round(float(row[1]))) > 0.1:
                x[tuple(int(s) for s in re.findall(r'\d+', row[0]))] = int(round(float(row[1])))
            elif 'w' in row[0] and int(round(float(row[1]))) > 0.1:
                w[tuple(int(round(float(s))) for s in re.findall(r'\d+', row[0]))] = int(round(float(row[1])))
            elif 'v[' in row[0] and int(round(float(row[1]))) == 1:
                v[tuple(int(round(float(s))) for s in re.findall(r'\d+', row[0]))] = int(round(float(row[1])))
            elif 'service_time' in row[0] and float(row[1]) > 0.1:
                service_time[tuple(int(round(float(s))) for s in re.findall(r'\d+', row[0]))[0]] = float(row[1])
            elif 'start_service' in row[0] and float(row[1]) > 0.1:
                start_service[tuple(int(float(s)) for s in re.findall(r'\d+', row[0]))[0]] = float(row[1])

        return x, w, service_time, start_service, v


def random_color():
    color = "%06x" % np.random.randint(0, 0xFFFFFF)
    return color


def random_color_alpha20(color):
    color_alpha = color + "66"
    return color_alpha


def random_color_alpha50(color):
    color_alpha = color + "80"
    return color_alpha


def main():
    xml_path = os.path.join("instances", str(N_FIELDS), XML_FILE_NAME)
    distances = get_distance_matrix(xml_path)


    plt.figure(2)

    fig, gnt = plt.subplots()

    x, w, service_time, start_service, v = get_vars()

    x_arcs = list(x.keys())
    w_arcs = list(w.keys())

    gnt.set_ylim(1, N_VEHICLES + N_ROBOTS + 1)
    gnt.set_xlim(0, start_service[END_DEPOT]+100)

    gnt.set_xlabel('time')
    gnt.set_ylabel('Transporter/Robot')

    ticks = [x for x in range(2, N_ROBOTS + N_VEHICLES + 3)]
    tick_labels = [str(x) for x in range(1, N_ROBOTS + N_VEHICLES + 2)]

    gnt.set_yticks(ticks)
    gnt.set_yticklabels(tick_labels)
    gnt.grid(True)

    # color preparation for visualization
    color_k_i = []
    color_x = []
    color_annotate = []
    for i in range(1 * 2 + n_START_DEPOTS_ROBOTS * 2 + N_FIELDS * 2 + 10):
        a = random_color()
        color_k_i.append("#" + random_color_alpha50(a))
        color_x.append("#" + random_color_alpha20(a))
        color_annotate.append("#" + a)

    # gantt: vehicle bars
    i = 0
    j = 0
    o = 1
    x_gantt_bars = dict(x)

    while sum(x_gantt_bars.values()) > 0:
        j += 1
        if (i, j) in x_gantt_bars.keys() and x_gantt_bars[i, j] > 0.1:
            if get_departure_node(i) == j:
            # if (i, j) in v.keys():
                gnt.broken_barh([(start_service[i] + service_time[i], distances[i, j])], (o + 0.7, 0.6), facecolors=(color_x[get_actual_node_number(j)]),
                                label="f" + str(i) + "->" + str(j))
            else:
                gnt.broken_barh([(start_service[i], distances[i, j])], (o + 0.7, 0.6), facecolors=(color_x[get_actual_node_number(j)]),
                                label="f" + str(i) + "->" + str(j))

            x_gantt_bars[i, j] -= 1
            if j == END_DEPOT:
                i = 0
                o += 1
            else:
                i = j
            j = 0
        else:
            continue

        # gantt: robot bars
    i = 0
    j = 0
    o = N_VEHICLES + 1
    w_gantt_bars = dict(w)
    while sum(w_gantt_bars.values()) > 0:
        j += 1
        if (i, j) in w_gantt_bars and w_gantt_bars[i, j] > 0.1:
            gnt.broken_barh([(start_service[i], distances[i, j])], (o + 0.7, 0.6),
                            facecolors=(color_x[get_actual_node_number(j)]), label="f" + str(i) + "->" + str(j))

            #gnt.broken_barh([(start_service[i] + service_time[i], distances[i, j])], (o + 0.7, 0.6),
            #               facecolors=(color_x[get_actual_node_number(j)]), label="f" + str(i) + "->" + str(j))

            if get_departure_node(i) == j:
                gnt.broken_barh([(start_service[i], service_time[i])], (o + 0.55, 0.9), facecolors=(color_k_i[get_actual_node_number(i)]), label="f" + str(i))

                #if get_actual_node_number(j) > 0 and j <= len(START_DEPOTS_ROBOTS) + N_FIELDS:
                gnt.annotate(str(get_actual_node_number(i)), (start_service[i] + service_time[i]/2, o + 0.8), fontsize=10, #
                              color=color_annotate[get_actual_node_number(i)])

            w_gantt_bars[i, j] -= 1
            if j >= END_DEPOT - n_start_depots_robots:
                #print(sum(w_gantt_bars[0, a] for a in range(1, END_DEPOT + 1) if (0, a) in w_gantt_bars.keys()))
                i = 0
                o += 1
            else:
                i = j
            j = 0
        elif j > END_DEPOT:
            i += 1
            j = 0
        else:
            continue

    plt.savefig("CP_VRP_gantt_10_0.png")
    plt.close()


if __name__ == '__main__':
    os.chdir('..')
    main()