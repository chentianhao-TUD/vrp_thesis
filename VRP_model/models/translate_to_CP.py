#! /usr/bin/env python
from ortools.sat.python import cp_model
from CRVRP import generate_input_data
import os
import math
import csv
import read_xml


def build_cp_model(data):
    model = cp_model.CpModel()
    # [start variables]
    x = {}
    w = {}
    v = {}
    y = {}
    service_time = {}
    start_service = {}

    for i in data["all_nodes"]:
        for j in data["all_nodes"]:
            x[i, j] = model.NewBoolVar(f'vehicle {i} -> {j}')  # 1
            w[i, j] = model.NewIntVar(0, data["transport_capacity"], f'robots {i} -> {j}')  # 2
            v[i, j] = model.NewBoolVar(f'AtLeastOne_robots {i} -> {j}')  # 3

    for j in data["arrival_nodes"]:
        for m in data["modes"]:
            y[j, m] = model.NewBoolVar(f'arrival_nodes{j} served in mode{m}')  # 4

    for i in data["all_nodes"]:
        service_time[i] = model.NewIntVar(0, data["M"], f'service time at node{i}')  # 5
        start_service[i] = model.NewIntVar(0, data["M"], f'start service at node{i}')  # 6

    variables = {"x": x, "w": w, "v": v, "y": y, "service_time": service_time, "start_service": start_service}

    # [end variables]

    # [start set bounds]

    for i, j in data["extended_arcs"]:
        if data["prohibited_arcs_vehicles"][i, j] == True or i == j:
            model.Add(x[i, j] == 0)  # 7

    for i in data["start_depots"]:
        for j in data["start_depots"]:
            model.Add(x[i, j] == 0)  # 8 ~GL.29

    for i in data["end_depots"]:
        for j in data["end_depots"]:
            model.Add(x[i, j] == 0)  # 8 ~GL.30

    for (i, j) in data["extended_arcs"]:
        if data["prohibited_arcs_robots"][i, j] == True or i == j:
            model.Add(w[i, j] == 0)  # 9
            model.Add(v[i, j] == 0)  # 10
        elif (i, j) in data["arrival_departure_arcs"]:
            model.Add(v[i, j] == 1)  # 11

    for j in data["all_nodes"]:
        if j in data["start_depots"] or j in data["end_depots"] or j in data["departure_nodes"]:
            model.Add(service_time[j] == 0)  # 12

    model.Add(start_service[0] == 0)

    # [end set bounds]

    # [start vehicle constraints]

    model.Add(sum(x[0, j] for j in data["all_nodes"][1:]) <= len(data["vehicles"]))  # 15 ~GL.2

    for i in data["all_nodes"][1:-1]:
        model.Add(sum(x[i, j] for j in data["all_nodes"][1:]) == 1)  # 16 ~GL.3
        model.Add(sum(x[h, i] for h in data["all_nodes"][:-1]) == 1)  # 17 ~GL.4

    for i in data["all_nodes"][:-1]:
        for j in data["all_nodes"][1:]:
            # model.Add(start_service[j] >= start_service[i] + data["distances"][i, j]).OnlyEnforceIf(x[i, j])  # 18 ~GL.5
            model.Add(start_service[j] >= start_service[i] + data["distances"][i, j] * x[i, j] -
                      data["M"] * (1 - x[i, j]))  # 18 ~GL.5

    # [end vehicle constraints]

    # [start robot constraints]

    model.Add(sum(w[0, j] for j in data["all_nodes"][1:]) <= min(len(data["vehicles"]) * data["transport_capacity"],
                                                                 len(data["robots"])))  # 19 ~GL.6

    # 20 ~GL.7
    model.Add(sum(w[i, j] for i in data["start_depots"] for j in data["all_nodes"][1:]) <= len(data["robots"]))

    if len(data["start_depots_robots"]) > 0:
        model.Add(sum(w[i, j] for i in data["start_depots_robots"] for j in data["all_nodes"][1:]) <=
                  len(data["robots"]) - len(data["vehicles"]) * data["transport_capacity"])  # 21 ~GL.28

    model.Add(sum(w[i, data["end_depot_vehicles"]] for i in data["all_nodes"][:-1]) <=
              min(len(data["vehicles"] * data["transport_capacity"]), len(data["robots"])))  # 22 ~GL.8

    model.Add(sum(w[i, j] for i in data["all_nodes"][:-len(data["end_depots"])]for j in data["end_depots"]) <=
              len(data["robots"]))  # 23 ~GL.9

    # 24 ~GL.10
    for i in data["all_customer_nodes"]:
        model.Add(sum(w[h, i] for h in data["all_nodes"][:-1]) == sum(w[i, j] for j in data["all_nodes"][1:]))

    for i in data["all_nodes"][:-1]:
        for j in data["all_nodes"][1:]:
            model.Add(w[i, j] <= v[i, j] * data["transport_capacity"])  # 25 ~GL.11
            model.Add(w[i, j] >= v[i, j])  # 26 ~GL.12

    for i in data["arrival_nodes"]:
        model.Add(start_service[data["arrival_to_departure_nodes"][i]] >= start_service[i] + service_time[i])  # 27 ~GL.13

    for i in data["all_nodes"][:-1]:
        for j in data["all_but_corresponding_nodes"][i]:
            model.Add(w[i, j] == 0).OnlyEnforceIf(x[i, j].Not())
            model.Add(w[i, j] <= data["transport_capacity"]).OnlyEnforceIf(x[i, j])  # 28 ~GL.14

    for i in data["arrival_nodes"]:
        model.Add(sum(y[i, m] for m in data["modes"]) == 1)
        model.Add(sum(m * y[i, m] for m in data["modes"]) == w[i, data["arrival_to_departure_nodes"][i]])  # 29 ~GL.15

    # 30 ~GL.16
    help_v_service_time = {}
    for j in data["arrival_nodes"]:
        for m in data["modes"]:
            help_v_service_time[j, m] = model.NewIntVar(0, data["M"], f'help service time {j} mode {m}')
    for j in data["arrival_nodes"]:
        for m in data["modes"]:
            model.AddDivisionEquality(help_v_service_time[j, m], y[j, m] * data["demands"][j], m)
    for j in data["arrival_nodes"]:
        # model.AddDivisionEquality(service_time[j], data["demands"][j], m).OnlyEnforceIf(y[j, m])
        # model.Add(service_time[j] == math.ceil(data["demands"][j] / m)).OnlyEnforceIf(y[j, m])
        model.Add(service_time[j] == sum(help_v_service_time[j, m] for m in data["modes"]))

    # 39 ~GL.17
    for j in data["arrival_nodes"]:
        model.AddExactlyOne(v[j, data["corresponding_nodes"][j]])

    # 40 ~GL.18+19
    for j in data["departure_nodes"]:
        model.Add(v[j, data["corresponding_nodes"][j]] == 0)
        model.Add(x[j, data["corresponding_nodes"][j]] == 0)

    # [end robot constraints]

    # [start valid inequalities]

    # 31 ~GL.20
    help_v_1 = {}
    help_v_2 = {}

    for i in data["all_nodes"][:-1]:
        for j in data["all_nodes"][1:]:
            help_v_1[i, j] = model.NewIntVar(0, data["M"], f'help v 1 {i} {j}')

    for i in data["all_nodes"][:-1]:
        for j in data["all_nodes"][1:]:
            model.AddDivisionEquality(help_v_1[i, j], w[i, j] * data["distances"][i, j], len(data["robots"]))

    for i in data["all_nodes"]:
        help_v_2[i] = model.NewIntVar(0, data["M"], f'help v 2 {i}')

    for i in data["all_nodes"]:
        model.AddDivisionEquality(help_v_2[i], service_time[i], len(data["robots"]))

    model.Add(start_service[data["end_depot_vehicles"]] >=
              (sum(help_v_1[i, j] for i in data["all_nodes"][:-1] for j in data["all_nodes"][1:]) +
              sum(help_v_2[h] for h in data["arrival_nodes"])))

    # 32 ~GL.21
    help_v_3 = {}

    for i in data["all_nodes"][:-1]:
        for j in data["all_nodes"][1:]:
            help_v_3[i, j] = model.NewIntVar(0, data["M"], f'help v 1 {i} {j}')

    for i in data["all_nodes"][:-1]:
        for j in data["all_nodes"][1:]:
            model.AddDivisionEquality(help_v_3[i, j], x[i, j] * data["distances"][i, j], len(data["vehicles"]))

    model.Add(start_service[data["end_depot_vehicles"]] >=
              sum(help_v_3[i, j] for i in data["all_nodes"][:-1] for j in data["all_nodes"][1:]))

    # 33 ~GL.22
    for i in data["all_nodes"][:-1]:
        model.Add(start_service[data["end_depot_vehicles"]] >= start_service[i] +
                  sum((data["distances"][i, j] + data["distances"][j, data["end_depot_vehicles"]]) * x[i, j]
                  for j in data["all_nodes"][1:]))

    # 34 ~GL.23
    for j in data["all_nodes"][1:]:
        model.Add(start_service[j] >= start_service[data["start_depot_vehicles"]] +
                  sum((data["distances"][data["start_depot_vehicles"], i] + data["distances"][i, j]) * x[i, j]
                      for i in data["all_nodes"][:-1]))

    # 35 ~GL.24
    for j in data["arrival_nodes"]:
        model.Add(sum(w[i, j] for i in data["all_nodes"][:-1]) >= 1)

    # 36 ~GL.25
    for i in data["arrival_nodes"]:
        model.Add(sum(w[i, j] for j in data["all_but_corresponding_nodes"][i]) <= data["transport_capacity"] - 1)

    # 37 ~GL.26
    for j in data["departure_nodes"]:
        model.Add(sum(w[i, j] for i in data["all_but_corresponding_nodes"][j]) <= data["transport_capacity"] - 1)

    # 38 ~GL.27
    for i in data["departure_nodes"]:
        model.Add(sum(w[i, j] for j in data["all_nodes"][1:]) >= 1)

    ### SYNCHRONIZATION BREAKING CONSTRAINTS ###
    # 39 ~additional equations
    for j in data["start_depots"]:
        for i in data["start_depots"]:
            if i < j:
                model.Add(start_service[j] >= start_service[i])

    for j in data["end_depots"]:
        for i in data["end_depots"]:
            if i < j:
                model.Add(start_service[j] >= start_service[i])


    # [end valid inequalities]

    # [start objective]

    model.Minimize(start_service[data["end_depot_vehicles"]])  # 14 ~GL.1

    # [end objective]

    return model, variables


def main(number_customer):
    # directory = os.path.join("instances", "test_instances")
    directory = os.path.join("instances", str(number_customer))
    for entry in os.scandir(directory):
        if entry.path.endswith(".xml") and entry.is_file():
            instance_configuration = entry.path[len(directory) +
                                                1:len(entry.path) - 4]
            print(instance_configuration)
            path = os.path.join(directory, instance_configuration + '.xml')
            instance_amount = read_xml.count_instances(path)
            for instance in range(instance_amount):
                file_name = "_".join(['Results', 'CP_VRP', instance_configuration, str(instance)]) + ".csv"
                result_file = os.path.join('results', str(number_customer), 'CP_VRP', file_name)
                with open(result_file, 'w', newline='') as result_csv_file:
                    writer = csv.writer(result_csv_file, delimiter=";")

                    data = generate_input_data(path, instance)
                    data["distances"].update((key, math.ceil(value)) for key, value in data["distances"].items())
                    model, variables = build_cp_model(data)

                    # [start call solver]
                    solver = cp_model.CpSolver()
                    status = solver.Solve(model)
                    solver.parameters.log_search_progress = True
                    if status == cp_model.OPTIMAL:
                        print(solver.StatusName())
                        print(solver.ObjectiveValue())
                        ins = [f'instance {instance}']
                        writer.writerow(ins)
                        obj = ['objective', str(solver.ObjectiveValue())]
                        writer.writerow(obj)
                        time = ['runtime', str(solver.WallTime())]
                        writer.writerow(time)
                        for i, j in data["extended_arcs"]:
                            if solver.Value(variables["x"][i, j]):
                                print('x', variables["x"][i, j])
                                x = [f'x[{i}, {j}]', str(solver.Value(variables["x"][i, j]))]
                                writer.writerow(x)

                        for i, j in data["extended_arcs"]:
                            if solver.Value(variables["w"][i, j]):
                                print('w', variables["w"][i, j])
                                w = [f'w[{i}, {j}]', str(solver.Value(variables["w"][i, j]))]
                                writer.writerow(w)
                        for i, j in data["extended_arcs"]:
                            if solver.Value(variables["v"][i, j]):
                                print('v', variables["v"][i, j])
                                v = [f'v[{i}, {j}]', str(solver.Value(variables["v"][i, j]))]
                                writer.writerow(v)
                        for j in data["arrival_nodes"]:
                            for m in data["modes"]:
                                if solver.Value(variables["y"][j, m]):
                                    print(variables["y"][j, m])
                                    y = [f'y[{j}, {m}]', str(solver.Value(variables["y"][j, m]))]
                                    writer.writerow(y)
                        for j in data["all_nodes"]:
                            if solver.Value(variables["service_time"][j]):
                                print(variables["service_time"][j], solver.Value(variables["service_time"][j]))
                                service_time = [f'service_time[{j}]', str(solver.Value(variables["service_time"][j]))]
                                writer.writerow(service_time)
                        for j in data["all_nodes"]:
                            if solver.Value(variables["start_service"][j]):
                                print(variables["start_service"][j], solver.Value(variables["start_service"][j]))
                                start_service = [f'start_service[{j}]', str(solver.Value(variables["start_service"][j]))]
                                writer.writerow(start_service)
                        result_csv_file.flush()
                    else:
                        print(solver.StatusName())

                    print('\n')



if __name__ == '__main__':
    os.chdir('..')
    main(10)
