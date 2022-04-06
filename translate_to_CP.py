#! /usr/bin/env python

from ortools.sat.python import cp_model
from CRVRP import generate_input_data
import os
import math

os.chdir('..')
path = 'instances/5/5-2-7-100.xml'
data = generate_input_data(path, 1)
data["distances"].update((key, round(value)) for key, value in data["distances"].items())
# print(data["distances"])
# print(data)

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

# [end variables]

# [start set bounds]

for i, j in data["extended_arcs"]:
    if data["prohibited_arcs_vehicles"][i, j] == True or i == j:
        model.Add(x[i, j] == 0)  # 7

for i in data["start_depots"]:
    for j in data["start_depots"]:
        model.Add(x[i, j] == 0)  # 8

for i in data["end_depots"]:
    for j in data["end_depots"]:
        model.Add(x[i, j] == 0)  # 8

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

model.Add(sum(x[0, j] for j in data["all_nodes"][1:]) <= len(data["vehicles"]))  # 15

for i in data["all_nodes"][1:-1]:
    model.Add(sum(x[i, j] for j in data["all_nodes"][1:]) == 1)  # 16
    model.Add(sum(x[h, i] for h in data["all_nodes"][:-1]) == 1)  # 17

for i in data["all_nodes"][:-1]:
    for j in data["all_nodes"][1:]:
        model.Add(start_service[j] >= start_service[i] + data["distances"][i, j]).OnlyEnforceIf(x[i, j])  # 18

# [end vehicle constraints]

# [start robot constraints]

model.Add(sum(w[0, j] for j in data["all_nodes"][1:]) <= min(len(data["vehicles"]) * data["transport_capacity"],
                                                             len(data["robots"])))  # 19

for i in data["start_depots"]:
    model.Add(sum(w[i, j] for j in data["all_nodes"][1:]) <= len(data["robots"]))  # 20

if len(data["start_depots_robots"]) > 0:
    for i in data["start_depots_robots"]:
        model.Add(sum(w[i, j] for j in data["all_nodes"][1:]) <= len(data["robots"]) -
                  len(data["vehicles"]) * data["transport_capacity"])  # 21

model.Add(sum(w[i, data["end_depot_vehicles"]] for i in data["all_nodes"][:-1]) <=
          min(len(data["vehicles"] * data["transport_capacity"]), len(data["robots"])))  # 22

for i in data["all_nodes"][:-len(data["end_depots"])]:
    model.Add(sum(w[i, j] for j in data["end_depots"]) <= len(data["robots"]))  # 23

for i in data["all_customer_nodes"]:
    model.Add(sum(w[h, i] for h in data["all_nodes"][:-1]) == sum(w[i, j] for j in data["all_nodes"][1:]))  # 24

for i in data["all_nodes"][:-1]:
    for j in data["all_nodes"][1:]:
        model.Add(w[i, j] <= v[i, j] * data["transport_capacity"])  # 25
        model.Add(w[i, j] >= v[i, j])  # 26

for i in data["arrival_nodes"]:
    model.Add(start_service[data["arrival_to_departure_nodes"][i]] >= start_service[i] + service_time[i])  # 27

for i in data["all_nodes"][:-1]:
    for j in data["all_but_corresponding_nodes"][i]:
        model.Add(w[i, j] <= data["transport_capacity"]).OnlyEnforceIf(x[i, j])  # 28

for i in data["arrival_nodes"]:
    model.Add(sum(y[i, m] for m in data["modes"]) == 1)
    model.Add(sum(m * y[i, m] for m in data["modes"]) == w[i, data["arrival_to_departure_nodes"][i]])  # 29

for j in data["arrival_nodes"]:
    for m in data["modes"]:
        model.Add(service_time[j] >= math.ceil(data["demands"][j] / m)).OnlyEnforceIf(y[j, m])  # 30

# [end robot constraints]

# [start objective]

# model.Minimize(start_service[data["end_depot_vehicles"]])  # 14

# [end objective]


# [start call solver]

solver = cp_model.CpSolver()
status = solver.Solve(model)

if status == cp_model.OPTIMAL:
    print(solver.StatusName())
    print(solver.ObjectiveValue())
    for i, j in data["extended_arcs"]:
        if solver.Value(x[i, j]):
            print('x', x[i, j])
    for i, j in data["extended_arcs"]:
        if solver.Value(w[i, j]):
            print('w', w[i, j], solver.Value(w[i, j]))
    for i, j in data["extended_arcs"]:
        if solver.Value(v[i, j]):
            print('v', v[i, j])
    for j in data["arrival_nodes"]:
        for m in data["modes"]:
            if solver.Value(y[j, m]):
                print(y[j, m])
    for j in data["all_nodes"]:
        if solver.Value(service_time[j]):
            print(service_time[j], solver.Value(service_time[j]))
    for j in data["all_nodes"]:
        if solver.Value(start_service[j]):
            print(start_service[j], solver.Value(start_service[j]))
else:
    print(solver.StatusName())
