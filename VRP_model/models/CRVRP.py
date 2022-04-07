import os
import sys


import read_xml
import numpy as np
import timeit
from gurobipy import *
import csv
import pandas as pd
from enum import Enum
from random import seed
from random import randint
import copy


_THREADS = 2
_TIMELIMIT = 60 * 1


class ModelType(Enum):
    CRVRP = 1


# region INPUT

def generate_input_data(path, instance):
    Nodes, demand, mode_amount, loc_x, loc_y, SetOfVehicles, SetOfRobots = read_xml.read_xml_file(
        path, instance)

    transport_capacity = 2

    modes = list(range(1, transport_capacity+1))

    start_depot_vehicles = 0
    if transport_capacity * len(SetOfVehicles) >= len(SetOfRobots):
        start_depots_robots = []
    else:
        n_start_depots_robots = math.ceil((len(SetOfRobots) - transport_capacity * len(SetOfVehicles)) / transport_capacity)
        start_depots_robots = list(range(1, n_start_depots_robots + 1))

    start_depots = [start_depot_vehicles] + start_depots_robots

    arrival_nodes = list(range(start_depots[-1]+1, start_depots[-1]+1 + len(Nodes)))
    departure_nodes = list(range(arrival_nodes[-1]+1, arrival_nodes[-1]+1 + len(Nodes)))
    all_customer_nodes = arrival_nodes + departure_nodes

    end_depots_robots = list(range(len(start_depots) + len(all_customer_nodes), len(start_depots) + len(all_customer_nodes) + len(start_depots_robots)))
    end_depot_vehicles = len(all_customer_nodes) + len(start_depots_robots) + len(end_depots_robots) + 1
    end_depots = end_depots_robots + [end_depot_vehicles]

    all_nodes = start_depots + all_customer_nodes + end_depots

    all_nodes_but_arrival = [x for x in all_nodes if x not in arrival_nodes]
    all_nodes_but_departure = [x for x in all_nodes if x not in departure_nodes]

    arrival_to_departure_nodes = {
        i: i+len(Nodes) for i in arrival_nodes
    }

    departure_to_arrival_nodes = {
        i: i-len(Nodes) for i in departure_nodes
    }
    corresponding_nodes = {**arrival_to_departure_nodes, **departure_to_arrival_nodes}
    all_but_corresponding_nodes = {}

    for i in all_nodes:
        if i in corresponding_nodes.keys():
            all_but_corresponding_nodes[i] = [x for x in all_nodes if x != corresponding_nodes[i] if x != i]
        else:
            all_but_corresponding_nodes[i] = [x for x in all_nodes if x != i]

    arrival_departure_arcs = [(i, j) for i in arrival_nodes for j in departure_nodes if arrival_nodes.index(i) == departure_nodes.index(j)]

    demand.update((x, y * 10) for x, y in demand.items())

    demands = [0] * len(start_depots) + list(demand.values())[1:-1] + [0] * len(Nodes) + [0] * len(end_depots)

    coop_worktime_factor = {(j, m): 1 / m
                            for j in arrival_nodes
                            for m in modes}

    extended_Arcs = [(i, j)
                     for i in all_nodes for j in all_nodes]

    locations_x = [loc_x[0]] * len(start_depots) + 2 * loc_x[1:-1] + [loc_x[-1]] * len(end_depots)
    locations_y = [loc_y[0]] * len(start_depots) + 2 * loc_y[1:-1] + [loc_y[-1]] * len(end_depots)

    distances = {(i, j): np.hypot(locations_x[i] - locations_x[j], locations_y[i] - locations_y[j])
                 for i, j in extended_Arcs}

    prohibited_arcs_vehicles = {(i, j): True
    if i == end_depot_vehicles
       or j == start_depot_vehicles
       or i == j
    else False  for (i, j) in extended_Arcs}

    prohibited_arcs_robots = {(i, j): True
    if i in end_depots
       or j in start_depots
       or i == j
    else False for (i, j) in extended_Arcs}

    forbidden_arcs_departure_to_arrival = {(i, j): True
                                if i == arrival_to_departure_nodes[j]
                                else False
                                for i in departure_nodes for j in arrival_nodes}

    prohibited_arcs_vehicles = {**prohibited_arcs_vehicles, **forbidden_arcs_departure_to_arrival}
    prohibited_arcs_robots = {**prohibited_arcs_robots, **forbidden_arcs_departure_to_arrival}


    big_M = 999999

    data = {
        "original_customer_nodes": Nodes,
        "all_nodes": all_nodes,
        "arrival_nodes": arrival_nodes,
        "departure_nodes": departure_nodes,
        "arrival_to_departure_nodes": arrival_to_departure_nodes,
        "departure_to_arrival_nodes": departure_to_arrival_nodes,
        "all_but_corresponding_nodes": all_but_corresponding_nodes,
        "all_customer_nodes": all_customer_nodes,
        "all_nodes_but_arrival": all_nodes_but_arrival,
        "all_nodes_but_departure": all_nodes_but_departure,
        "arrival_departure_arcs": arrival_departure_arcs,
        "start_depot_vehicles": start_depot_vehicles,
        "start_depots_robots": start_depots_robots,
        "start_depots": start_depots,
        "end_depots_robots": end_depots_robots,
        "end_depot_vehicles": end_depot_vehicles,
        "end_depots": end_depots,
        "extended_arcs": extended_Arcs,
        "prohibited_arcs_vehicles": prohibited_arcs_vehicles,
        "prohibited_arcs_robots": prohibited_arcs_robots,
        "vehicles": SetOfVehicles,
        "robots": SetOfRobots,
        "distances": distances,
        "demands": demands,
        "transport_capacity": transport_capacity,
        "modes": modes,
        "coop_worktime_factor": coop_worktime_factor,
        "M": big_M,
        "corresponding_nodes": corresponding_nodes}

    return data
# endregion

# region APPROXIMATE MAKESPAN

"""
def build_model_integer_no_split_fixed_supporting(data):

    return mdl, variables"""

"""
def approximate_makespan(data):
    mdl_fixed_support, variables_fixed_support = build_model_integer_no_split_fixed_supporting(
        data)
    # mdl_fixed_support.setParam("OutputFlag", 0)
    mdl_fixed_support.setParam("Threads", _THREADS)
    mdl_fixed_support.setParam("TimeLimit", 1 * 60)
    # mdl_fixed_support.setParam("Presolve", 2)
    mdl_fixed_support.setParam("Cuts", 0)
    mdl_fixed_support.setParam("ScaleFlag", 1)
    mdl_fixed_support.setParam("NormAdjust", 0)
    mdl_fixed_support.optimize()

    approximated_makespan = mdl_fixed_support.ObjVal
    runtime_approximation = mdl_fixed_support.Runtime

    start_solution = {"x": {k: v.X for k, v in variables_fixed_support["x"].items(
    )}, "w": {k: v.X for k, v in variables_fixed_support["w"].items()}}

    return start_solution, approximated_makespan, runtime_approximation
"""
# endregion

# region INTEGER MODEL


def build_model_CRVRP(data):
    mdl = Model('CRVRP')

    # set distance to end_depot to 0 for objective function latest service
    """for key, value in data["distances"].items():
        if key[1] in data["end_depots"]:
            data["distances"][key] = 0.0"""

    # x[k, i,j] = 1, if a vehicle travels directly from i to j
    x = mdl.addVars(data["all_nodes"], data["all_nodes"],
                    name='x', vtype=GRB.BINARY)

    # w[i,j], number of robots on arc (i,j)
    w = mdl.addVars(data["all_nodes"], data["all_nodes"], lb=0, ub=data["transport_capacity"],
                    name='w', vtype=GRB.INTEGER)

    # y[i,j] = 1, if any robot travels on arc arc (i,j)
    v = mdl.addVars(data["all_nodes"], data["all_nodes"], lb=0, name="v", vtype=GRB.BINARY)

    # y[j, m] = 1, if field j is serviced by primary vehicle in mode m
    y = mdl.addVars(data["arrival_nodes"], data["modes"], name='y', vtype=GRB.BINARY)

    # q[k, j] = 1, if field j is visited by a primary vehicle
    # q = mdl.addVars(data["vehicles"], data["all_nodes"], name='q', vtype=GRB.BINARY)

    # s[j], service time vertex j is worked in mode m
    service_time = mdl.addVars(data["all_nodes"], lb=0, name='service_time', vtype=GRB.CONTINUOUS)

    # t[j], latest start of service at node j
    start_service = mdl.addVars(
        data["all_nodes"], lb=0, ub=data["big_M"], name='start_service', vtype=GRB.CONTINUOUS)

    mdl.update()

    variables = {"x": x, "w": w, "y": y, "serivce_time": service_time, "start_service": start_service}

    ### SET BOUNDS ###
    [x[i, j].setAttr("UB", 0) for (i, j) in data["extended_arcs"]
     if data["prohibited_arcs_vehicles"][(i, j)] == True or i == j]

    [w[i, j].setAttr("UB", 0) for (i, j) in data["extended_arcs"]
     if data["prohibited_arcs_robots"][(i, j)] == True or i == j]

    [v[i, j].setAttr("UB", 0) for (i, j) in data["extended_arcs"]
     if data["prohibited_arcs_robots"][(i, j)] == True or i == j]

    [v[i, j].setAttr("LB", 1) for (i, j) in data["extended_arcs"]
     if (i, j) in data["arrival_departure_arcs"]]

    [service_time[j].setAttr("UB", 0.0) for j in data["departure_nodes"]]
    [service_time[j].setAttr("UB", 0.0) for j in data["start_depots"]]
    [service_time[j].setAttr("UB", 0.0) for j in data["end_depots"]]

    start_service[data["start_depot_vehicles"]].setAttr("UB", 0.0)

    # Symmetry Breaking at robot depots

    """[x[i, j].setAttr("UB", 0) for i in data["start_depots_robots"] for j in data["start_depots_robots"]
     if j > i+data["transport_capacity"]-1 or i >= j]

    [x[i, j].setAttr("UB", 0) for i in data["end_depots_robots"] for j in data["end_depots_robots"]
     if j > i+data["transport_capacity"]-1 or i >= j]"""

    """[x[i, j].setAttr("UB", 0) for i in data["start_depots_robots"] for j in data["start_depots_robots"]
     if j > i+1 or i >= j]

    [x[i, j].setAttr("UB", 0) for i in data["end_depots_robots"] for j in data["end_depots_robots"]
     if j > i+-1 or i >= j]"""
    [x[i, j].setAttr("UB", 0) for i in data["start_depots"] for j in data["start_depots"]]
    [x[i, j].setAttr("UB", 0) for i in data["end_depots"] for j in data["end_depots"]]


    # max_distance_customer = max(
    #     data["distances"], key=lambda key: data["distances"][key])[1]

    # q[1, max_distance_customer].setAttr("LB", 1)


    ### OBJECTIVE ###

    mdl.setObjective(start_service[data["end_depot_vehicles"]], GRB.MINIMIZE)
    # mdl.setObjectiveN(start_service[data["end_depot_vehicles"]], index=0, priority=1)
    # mdl.setObjectiveN(quicksum(start_service[j] for j in data["all_nodes"]), index=1, priority=0)

    ### VEHICLES ###

    mdl.addConstr((quicksum(x[data["start_depot_vehicles"], j] for j in data["all_nodes"][1:]) <= len(data["vehicles"])),
                   "MaxNumberVehiclesStart")

    mdl.addConstrs((quicksum(x[i, j] for j in data["all_nodes"][1:]) == 1
                    for i in data["all_nodes"][1:-1]),
                   "OutFlowVehicles")

    mdl.addConstrs((quicksum(x[h, i] for h in data["all_nodes"][:-1]) == 1
                    for i in data["all_nodes"][1:-1]),
                   "InFlowVehicles")

    mdl.addConstrs((start_service[j] >= start_service[i] + data["distances"][(i, j)] * x[i, j] - data["M"] * (1 - x[i, j])
                    for i in data["all_nodes"][:-1]
                    for j in data["all_nodes"][1:]),
                   "NodeArrivalVehicles")

    # mdl.addConstrs(start_service[data["end_depot_vehicles"]] >= start_service[i] + data["distances"][i, data["end_depot_vehicles"]] for i in data["all_nodes"][:1])
    # mdl.addConstr(start_service[13] >= start_service[9] + data["distances"][9, 13])

    # mdl.addConstr(w[0, 5] == 2)
    # mdl.addConstr(w[0, 2] == 2)
    """mdl.addConstr(x[3, 1] == 1)
    mdl.addConstr(x[3, 8] == 0)
    mdl.addConstr(x[1, 3] == 1)
    mdl.addConstr(x[0, 4] == 1)"""
    # mdl.addConstr(x[10, 15] == 0)

    # mdl.addConstrs(x[i, j] == 0 for i in data["start_depots"] for j in data["end_depots"])

    """mdl.addConstr(x[0, 1] == 1)
    mdl.addConstr(x[1, 2] == 1)
    mdl.addConstr(x[2, 6] == 1)

    mdl.addConstr(x[0, 3] == 1)
    mdl.addConstr(x[3, 4] == 1)
    mdl.addConstr(x[4, 7] == 1)

    mdl.addConstr(x[6, 5] == 1)
    mdl.addConstr(x[5, 8] == 1)"""


    ### ROBOTS ###

    """mdl.addConstrs((quicksum(w[i, j] for j in data["all_nodes"][1:]) - quicksum(w[h, i] for h in data["start_depots"]) == 1
                    for i in data["start_depots_robots"]),
                   "EachRobotsfromOneStartDepot")"""

    mdl.addConstr((quicksum(w[data["start_depot_vehicles"], j] for j in data["all_nodes"][1:]) <= min(len(data["vehicles"]) * data["transport_capacity"], len(data["robots"]))),
                  "RobotsStartAtVehicleDepot")

    mdl.addConstr((quicksum(w[i, j] for i in data["start_depots"] for j in data["all_nodes"]) <= len(data["robots"])),       # len(data["start_depots"])
                  "TotalRobotStarts")
    """mdl.addConstr((quicksum(w[i, j] for i in data["start_depots"] for j in data["all_nodes"][len(data["start_depots"]):]) <= len(data["robots"])),
                  "TotalRobotStarts")"""

    if len(data["start_depots_robots"]) > 0:    # valid inequality: all slots are occupied when vehicles leave the start depot for the first time
        mdl.addConstr((quicksum(w[i, j] for j in data["all_nodes"][1:] for i in data["start_depots_robots"]) <= len(data["robots"]) - data["transport_capacity"] * len(data["vehicles"])),
                      "RobotsStartAtRobotDepot")

    mdl.addConstr((quicksum(w[i, data["end_depot_vehicles"]] for i in data["all_nodes"][:-1]) <= min(len(data["vehicles"]) * data["transport_capacity"], len(data["robots"]))),
                  "RobotsFinishAtVehicleDepot")

    mdl.addConstr((quicksum(w[i, j] for i in data["all_nodes"][:-len(data["end_depots"])] for j in data["end_depots"]) <= len(data["robots"])),
                  "TotalRobotFinishes")



    """if len(data["end_depots_robots"]) > 0:
        mdl.addConstr((quicksum(w[i, j] for j in data["all_nodes"][1:] for i in data["start_depots_robots"]) <= len(data["robots"]) - data["transport_capacity"] * data["vehicles"]),
                      "RobotsStartAtRobotDepot")"""

    """mdl.addConstrs((quicksum(w[h, i] for h in data["all_nodes"][1:]) - quicksum(w[i, j] for j in data["start_depots"]) == 1
                    for i in data["end_depots_robots"]),
                   "EachRobotToOneEndDepot")"""

    mdl.addConstrs((quicksum(w[h, i] for h in data["all_nodes"][:-1]) == quicksum(w[i, j] for j in data["all_nodes"][1:])
                    for i in data["all_customer_nodes"]),
                   "FlowConservationRobots")

    """mdl.addConstrs((quicksum(w[i, j] for i in data["all_nodes"][:-1]) == w[j, data["arrival_to_departure_nodes"][j]] 
                    for j in data["arrival_nodes"]),
                   "FlowConversationAtArrival")"""

    mdl.addConstrs((w[i, j] <= v[i, j] * data["transport_capacity"]
                    for i in data["all_nodes"][:-1]
                    for j in data["all_nodes"][1:]),
                   "MaxNumberRobotsOnArc")

    mdl.addConstrs((w[i, j] >= v[i, j]
                    for i in data["all_nodes"][:-1]
                    for j in data["all_nodes"][1:]),
                   "ArcUsedByRobot")    # valid inequality

    # surplus constraint?
    """mdl.addConstrs((start_service[j] >= start_service[i] + data["distances"][(i, j)] * v[i, j] -
                data["M"] * (1 - v[i, j])
                    for i in data["all_nodes"][:-1]
                    for j in data["all_nodes"][1:]),           # for j in data["all_but_corresponding_nodes"][i]),
                   "StartServiceRobots")"""

    mdl.addConstrs((start_service[data["arrival_to_departure_nodes"][i]] >= start_service[i] + service_time[i]
                    for i in data["arrival_nodes"]),
                   "RobotsEndService")

    mdl.addConstrs((w[i, j] <= x[i, j] * data["transport_capacity"]
                    for i in data["all_nodes"][:-1]
                    for j in data["all_but_corresponding_nodes"][i]),
                   "VehicleRobotSynchronization1")


    ### OPERATING MODES ###

    mdl.addConstrs((quicksum(m * y[i, m] for m in data["modes"]) == w[i, data["arrival_to_departure_nodes"][i]]
                    for i in data["arrival_nodes"]),
                   "SettingMode")


    mdl.addConstrs((service_time[j] == quicksum(y[j, m] * data["demands"][j] / m for m in data["modes"])
                    for j in data["arrival_nodes"]),
                   "SettingServiceTime")


    ### VALID INEQUALITIES ###

    mdl.addConstr((start_service[data["end_depot_vehicles"]] >= (quicksum(w[i, j] * data["distances"][(i, j)] for i in data["all_nodes"][:-1] for j in data["all_nodes"][1:])
                                                        + quicksum(y[j, m] * data["demands"][j] / m for j in data["arrival_nodes"] for m in data["modes"])) / len(data["robots"])),
                  "AverageTimeRobots")  # I

    mdl.addConstr((start_service[data["end_depot_vehicles"]] >= (quicksum(x[i, j] * data["distances"][(i, j)] for i in data["all_nodes"][:-1] for j in data["all_nodes"][1:])) / len(data["vehicles"])),
                  "AverageTimeVehicles")  # I

    mdl.addConstrs((start_service[data["end_depot_vehicles"]] >= start_service[i] + quicksum((data["distances"][(i, j)] + data["distances"][(j, data["end_depot_vehicles"])]) * x[i, j] for j in data["all_nodes"][1:])
                    for i in data["all_nodes"][:-1]),
                   "LowerBoundRoutingVehicle")

    mdl.addConstrs((start_service[j] >= start_service[data["start_depot_vehicles"]] + quicksum((data["distances"][(data["start_depot_vehicles"], i)] + data["distances"][(i, j)]) * x[i, j] for i in data["all_nodes"][:-1])
                    for j in data["all_nodes"][1:]),
                   "LowerBoundArrivalVehicle")

    mdl.addConstrs((quicksum(w[i, j] for i in data["all_nodes"][:-1]) >= 1
                    for j in data["arrival_nodes"]),
                   "VisitArrivalNodesNotEmpty")

    mdl.addConstrs((quicksum(w[i, j] for j in data["all_but_corresponding_nodes"][i]) <= data["transport_capacity"] - 1
                    for i in data["arrival_nodes"]),
                   "LeaveArrivalNodesNotFullyLoaded")

    mdl.addConstrs((quicksum(w[i, j] for i in data["all_but_corresponding_nodes"][j]) <= data["transport_capacity"] - 1
                    for j in data["departure_nodes"]),
                   "VisitDepartureNodesNotFullyLoaded")

    mdl.addConstrs((quicksum(w[i, j] for j in data["all_nodes"][1:]) >= 1
                    for i in data["departure_nodes"]),
                   "LeaveDepartureNodesNotEmpty")

    ### SYNCHRONIZATION BREAKING CONSTRAINTS ###



    return mdl, variables



# region GUROBI MODEL

def solve_relaxed_model(mdl):
    mdl.update()
    presolved_mdl = mdl.presolve()
    presolved_mdl.write("Presolved_Model_3.LP")
    presolved_mdl = presolved_mdl.relax()
    presolved_mdl.optimize()
    presolved_mdl.write("Relaxed_Solution_3.SOL")


def tune_model_parameters(mdl):
    mdl.Params.tuneResults = 10
    mdl.setParam("TuneTrials", 5)
    mdl.setParam("TuneTimeLimit", 12 * 3600)

    mdl.tune()

    if mdl.tuneResultCount > 0:
        for i in range(10):
            mdl.getTuneResult(i)
            mdl.write('tune' + str(i) + '.prm')

    mdl.getTuneResult(0)


def set_parameters(mdl, seed, log_file):
    mdl.setParam("LogFile", os.path.join("logs", log_file))
    mdl.setParam("Seed", seed)
    mdl.setParam("Threads", _THREADS)
    mdl.setParam("TimeLimit", _TIMELIMIT)
    mdl.setParam("Symmetry", 2)
    mdl.setParam("Cuts", 2)
    mdl.setParam("PreDepRow", 1)
    mdl.setParam("ScaleFlag", 1)
    mdl.setParam("NormAdjust", 0)


def set_start_solution(variables, start_solution):
    for k, v in start_solution["x"].items():
        variables["x"][k].setAttr("Start", v)
    for k, v in start_solution["w"].items():
        variables["w"][k].setAttr("Start", v)


def solve_IIS(mdl):
    mdl.computeIIS()
    if mdl.IISMinimal:
        print('IIS is minimal\n')
    else:
        print('IIS is not minimal\n')
    print('\nThe following constraint(s) cannot be satisfied:')
    for c in mdl.getConstrs():
        if c.IISConstr:
            print('%s' % c.constrName)
    mdl.write("model.ilp")


def build_model(type_of_model, data):
    if type_of_model == ModelType.CRVRP:
        return build_model_CRVRP(data)



def solve(mdl, log_file, seed):
    set_parameters(mdl, seed, log_file)
    mdl.optimize()
    status = mdl.status

    return status

# endregion

"""
def represent_routes(variables, data):
    # primary route string
    primary_route_string = {k: '0' for k in data["primary_vehicles"]}
    primary_starting_time_string = {k: '0' for k in data["primary_vehicles"]}
    primary_service_time_string = {k: '0.0' for k in data["primary_vehicles"]}
    primary_driving_time_string = {k: '' for k in data["primary_vehicles"]}
    for k in data["primary_vehicles"]:
        i = 0
        j = data["nodes_of_primary"][k][1]
        while j < len(data["all_nodes"]) - 1:
            if variables['x'][i, j].x >= 0.9:
                primary_route_string[k] += (
                        '-' + str(j - len(data["original_customer_nodes"]) * (k - 1)))
                primary_starting_time_string[k] += ('-' + str(variables['t'][j].x))
                primary_service_time_string[k] += ('-' + str(sum(variables['s'][j, m].x for m in data["modes"][j])))
                primary_driving_time_string[k] += (str(data["distances"][(i, j)]) + '-')
                i = j
                j = data["nodes_of_primary"][k][1]
            else:
                j += 1

        primary_route_string[k] += ('-' + str(len(data["original_customer_nodes"]) + 1))
        primary_starting_time_string[k] += ('-' + str(variables['t'][j].x))
        primary_service_time_string[k] += ('-' + str(sum(variables['s'][j, m].x for m in data["modes"][j])))
        primary_driving_time_string[k] += (str(data["distances"][(i, j)]))

    # secondary route string
    supporting_route_string = {k: '0' for k in data["supporting_vehicles"]}
    supporting_starting_time_string = {k: '0' for k in data["supporting_vehicles"]}
    supporting_service_time_string = {k: '0.0' for k in data["supporting_vehicles"]}
    supporting_driving_time_string = {k: '' for k in data["supporting_vehicles"]}

    w = {(i, j): variables['w'][i, j].x for (i, j) in variables['w']}

    for k in data["supporting_vehicles"]:
        i = 0
        j = 1
        while j < len(data["all_nodes"]) - 1:
            if w[i, j] > 0.9:
                primary_vehicle_number = int(j / len(data["original_customer_nodes"]) + 0.9)
                supporting_route_string[k] += (
                            '-' + str(j - len(data["original_customer_nodes"]) * (primary_vehicle_number - 1)))
                supporting_starting_time_string[k] += ('-' + str(variables['t'][j].x))
                supporting_service_time_string[k] += ('-' + str(sum(variables['s'][j, m].x for m in data["modes"][j])))
                supporting_driving_time_string[k] += (str(data["distances"][(i, j)]) + '-')
                w[i, j] -= 1
                i = j
                j = 1
            else:
                j += 1
        supporting_route_string[k] += ('-' + str(len(data["original_customer_nodes"]) + 1))
        supporting_starting_time_string[k] += ('-' + str(variables['t'][j].x))
        supporting_service_time_string[k] += ('-' + str(sum(variables['s'][j, m].x for m in data["modes"][j])))
        supporting_driving_time_string[k] += (str(data["distances"][(i, j)]))

    result_representations = [primary_route_string,
                              primary_starting_time_string,
                              primary_service_time_string,
                              primary_driving_time_string, supporting_route_string,
                              supporting_starting_time_string,
                              supporting_service_time_string,
                              supporting_driving_time_string]

    return result_representations
"""

def main(number_customer, selected_model_type):
    # directory = os.path.join("instances", "test_instances")
    directory = os.path.join("instances", str(number_customer))
    for entry in os.scandir(directory):
        if entry.path.endswith(".xml") and entry.is_file():
            instance_configuration = entry.path[len(directory) +
                                                1:len(entry.path) - 4]
            print(instance_configuration)

            all_results = list()

            file_name = "_".join(['Results', selected_model_type.name, instance_configuration,
                                  "LastService", "VI1+Tuning_bigM_fix"]) + ".csv"
            result_file = os.path.join('results', str(
                number_customer), selected_model_type.name, file_name)

            with open(result_file, 'w', newline='') as result_csv_file:
                writer = csv.writer(result_csv_file, delimiter=";")
                path = os.path.join(directory, instance_configuration + '.xml')
                instance_amount = read_xml.count_instances(path)


                for instance in range(instance_amount):
                    data = generate_input_data(path, instance)

                    approximated_makespan = 0
                    runtime_approximation = 0

                    # start_solution, approximated_makespan, runtime_approximation = approximate_makespan(data)

                    # data["big_M"] = int(approximated_makespan + 1)
                    data["big_M"] = 10000

                    mdl, variables = build_model(selected_model_type, data)

                    lp_file = "_".join(
                        [selected_model_type.name, instance_configuration, str(instance)]) + ".LP"
                    mdl.write(os.path.join("lp_files", lp_file))

                    # solve_relaxed_model(mdl)

                    # set_start_solution(variables, start_solution)

                    # tune_model_parameters(mdl)

                    for iteration in range(1):
                        log_file = "_".join(
                            [selected_model_type.name, instance_configuration, str(instance), str(iteration)]) + ".LOG"
                        seed = randint(0, 2000000000)

                        status = solve(mdl, log_file, seed)

                        if status == GRB.INFEASIBLE:
                            solve_IIS(mdl)
                        else:

                            """tmp_result = [seed, selected_model_type.name, instance_configuration, instance, iteration,
                                          mdl.ObjVal, approximated_makespan, mdl.Runtime, runtime_approximation,
                                          mdl.MIPGap, mdl.NodeCount, mdl.IterCount]"""
                            tmp_result = [seed, selected_model_type.name, instance_configuration, instance, iteration,
                                          mdl.ObjVal, approximated_makespan, mdl.Runtime, runtime_approximation,
                                          mdl.NodeCount, mdl.IterCount] # Multi Objective
                            all_results.append(tmp_result)

                            writer.writerow(tmp_result)
                            result_csv_file.flush()

                            """
                            result_representation = represent_routes(variables, data)
                            result_representation_file = "_".join([selected_model_type.name, instance_configuration,
                                                                  str(instance), str(iteration)]) + ".txt"
                            with open("routes/" + result_representation_file, 'w') as f:
                                for item in result_representation:
                                    f.write("%s\n" % item)
                            """

                            if mdl.SolCount > 0:
                                sol_file = "_".join([selected_model_type.name, instance_configuration, str(
                                    instance), str(iteration)]) + ".SOL"
                                mdl.write(os.path.join("solutions", sol_file))

                                # for var_dict in variables.values():
                                #     for k, v in var_dict.items():
                                #         if v.x > 0.001:
                                #             print(v.varName + ": " + str(v.x))

                        mdl.reset()


if __name__ == '__main__':
    os.chdir('..')
    main(5, ModelType.CRVRP)
