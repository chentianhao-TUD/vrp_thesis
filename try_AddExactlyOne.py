#!/usr/bin/env python
from ortools.sat.python import cp_model
import collections
from itertools import permutations

# [start input data]
Nodes = [0, 1, 2, 3]
Vehicles = [0, 1]
distance = {
    (0, 1): 63, (1, 0): 63, (0, 2): 67, (2, 0): 67,
    (1, 2): 50, (2, 1): 50, (0, 3): 100, (3, 0): 100,
    (1, 3): 162, (3, 1): 162, (2, 3): 161, (3, 2): 161
}
demand = [0, 300, 500, 700]
# [end input data]

# [start initiate model and solver]
model = cp_model.CpModel()
solver = cp_model.CpSolver()
# [end initiate model and solver]

x = {}
for i in Nodes:
    for j in Nodes:
        for k in Vehicles:
            x[(k, i, j)] = model.NewBoolVar(f'v{k} {i} {j}')

for i in Nodes:
    for k in Vehicles:
        model.AddAtLeastOne(x[(k, i, j)] for j in Nodes)

for i in Nodes:
    model.AddExactlyOne(x[(k, i, 0)] for k in Vehicles)
# print(x.values())

start_service = {}
service_time = {}
distance_var = {}
for i in Nodes:
    service_time[i] = demand[i]
    start_service[i] = model.NewIntVar(0, 9999, f'start time at Node{i}')

for i in Nodes:
    for j in Nodes:
        distance_var[(i, j)] = model.NewIntVar(0, 9999, f'distance between Node {i} {j}')

for i, j in permutations(Nodes, 2):
    model.Add(distance_var[(i, j)] == distance[(i, j)])

print(service_time)
print(start_service)

model.Add(start_service[0] == 0)
for i in Nodes:
    for j in Nodes:
        for k in Vehicles:
            if j != 0:
                model.Add(start_service[j] >= start_service[i] + distance_var[(i, j)] + service_time[i]).OnlyEnforceIf(
                    x[(k, i, j)])

for i in Nodes:
    for j in Nodes:
        for k in Vehicles:
            print('possible')
