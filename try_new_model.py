from ortools.sat.python import cp_model
from itertools import permutations
import numpy as np

# [start input data]
Nodes = [0, 1, 2, 3]
distance = {(0, 1): 63, (1, 0): 63, (0, 2): 67, (2, 0): 67, (1, 2): 50, (2, 1): 50, (0, 3): 100, (3, 0): 100, (1, 3): 162, (3, 1): 162, (2, 3): 161, (3, 2): 161}
demand = [0, 300, 500, 700]
Vehicle = [0, 1]
# [end input data]

# [start initiate model and solver]
model = cp_model.CpModel()
solver = cp_model.CpSolver()
# [end initiate model and solver]

single_K = {}
any_k = {}
time_back_to_depot = {}
start_time = {}
service_time = demand


for k in Vehicle:
    for i in Nodes:
        start_time[i] = model.NewIntVar(0, 9999, f'start time at node{i}')
        for j in Nodes:
            single_K[k, i, j] = model.NewBoolVar(f'v{k} {i} -> {j}')
            any_k[i, j] = model.NewBoolVar(f'v {i} -> {j}')
    time_back_to_depot[k] = model.NewIntVar(0, 9999, f'v{k} back to depot')

for k in Vehicle:
    for i in Nodes:
        for j in Nodes:
            model.Add(any_k[i, j] == 1).OnlyEnforceIf(single_K[k, i, j])
            model.Add(single_K[k, i, j] == 0).OnlyEnforceIf(any_k[i, j].Not())
for i in Nodes:
    for j in Nodes:
        if i == j:
            model.Add(any_k[i, j] == 0)
        for k in Vehicle:
            if i == j:
                model.Add(single_K[k, i, j] == 0)

for j in Nodes:
    model.AddAtLeastOne(single_K[k, i, j] for k in Vehicle for i in Nodes)
    for k in Vehicle:
        model.Add(sum(single_K[k, i, j] for i in Nodes) == sum(single_K[k, j, i] for i in Nodes))

for k in Vehicle:
    model.AddExactlyOne(single_K[k, i, 0] for i in Nodes)


for k in Vehicle:
    model.AddExactlyOne(single_K[k, 0, j] for j in Nodes[1:])

model.Add(start_time[0] == 0)

for i in Nodes:
    for j in Nodes[1:]:
        if i != j:
            model.Add(start_time[j] >= start_time[i] + service_time[i] + distance[i, j]).OnlyEnforceIf(
                any_k[i, j])

for k in Vehicle:
    for i in Nodes[1:]:
        model.Add(time_back_to_depot[k] == start_time[i] + service_time[i] + distance[i, 0]).OnlyEnforceIf(
            single_K[k, i, 0])


obj = model.NewIntVar(0, 9999, 'obj')
model.AddMaxEquality(obj, time_back_to_depot.values())
model.Minimize(obj)

status = solver.Solve(model)
if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
    print(solver.ObjectiveValue())
    for k in Vehicle:
        if solver.Value(time_back_to_depot[k]):
            print(f'v{k}', solver.Value(time_back_to_depot[k]))
    for i in Nodes:
        for j in Nodes:
            for k in Vehicle:
                if solver.Value(single_K[k, i, j]):
                    print(single_K[k, i, j])
else:
    print(solver.StatusName())

