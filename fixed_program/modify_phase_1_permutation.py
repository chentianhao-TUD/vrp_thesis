#! /usr/bin/env python

from ortools.sat.python import cp_model
from itertools import permutations

# [start input data]
Nodes = [0, 1, 2, 3]
distance = {(0, 1): 63, (1, 0): 63, (0, 2): 67, (2, 0): 67, (1, 2): 50, (2, 1): 50, (0, 3): 100, (3, 0): 100,
            (1, 3): 162, (3, 1): 162, (2, 3): 161, (3, 2): 161}
demand = [0, 300, 500, 700]
Vehicle = [0, 1, 2]
# [end input data]

# [start initiate model and solver]
model = cp_model.CpModel()
solver = cp_model.CpSolver()
# [end initiate model and solver]

# [start create variables]
single_K = {}  # BoolVar, single_K[k, i, j] =1 if a primary vehicle k travels from i to j
any_k = {}  # BoolVar, any_k[i, j] = 1 if there is at least one primary vehicle travels from i to j
time_back_to_depot = {}  # IntVar, time_back_to_depot[k] is the time that a vehicle k returns to depot
start_time = {}  # IntVar, start_time[i] is the time that a vehicle k arrives at node i
service_time = demand  # in this basic model, the service_time[i] is a constant value
makespan = model.NewIntVar(0, 9999, 'makespan')  # the time that the last vehicle returns to depot

for k in Vehicle:
    for i in Nodes:
        start_time[i] = model.NewIntVar(0, 9999, f'start time at node{i}')
    time_back_to_depot[k] = model.NewIntVar(0, 9999, f'v{k} back to depot')
for k in Vehicle:
    for i, j in permutations(Nodes, 2):
        single_K[k, i, j] = model.NewBoolVar(f'v{k} {i} -> {j}')
        any_k[i, j] = model.NewBoolVar(f'v {i} -> {j}')

# [end create variables]

# [start add constraints]
# link any_k to single_k
for k in Vehicle:
    for i, j in permutations(Nodes, 2):
            model.Add(any_k[i, j] == 1).OnlyEnforceIf(single_K[k, i, j])
            model.Add(single_K[k, i, j] == 0).OnlyEnforceIf(any_k[i, j].Not())

for i, j in permutations(Nodes, 2):
    model.AddAtLeastOne(single_K[k, i, j] for k in Vehicle).OnlyEnforceIf(any_k[i, j])

# each nodes should be visited at least once
# and if a node j is visited by vehicle k, k should also leave j
for j in Nodes:
    model.AddAtLeastOne(single_K[k, i, j] for k in Vehicle for i in Nodes if i != j)

for k in Vehicle:
    for h in Nodes:
        model.Add(sum(single_K[k, i, h] for i in Nodes if i != h) == sum(single_K[k, h, j] for j in Nodes if j != h))

# every vehicle should return to depot
for k in Vehicle:
    model.AddExactlyOne(single_K[k, i, 0] for i in Nodes[1:])

# every vehicle should leave depot
for k in Vehicle:
    model.AddExactlyOne(single_K[k, 0, j] for j in Nodes[1:])

# start time at depot is 0
model.Add(start_time[0] == 0)

# start time at j, is big M necessary?
for i in Nodes:
    for j in Nodes[1:]:
        if i != j:
            model.Add(start_time[j] >= start_time[i] + service_time[i] + distance[i, j]).OnlyEnforceIf(
                any_k[i, j])

# the time that a vehicle k returns to the depot
for k in Vehicle:
    for i in Nodes[1:]:
        model.Add(time_back_to_depot[k] >= start_time[i] + service_time[i] + distance[i, 0]).OnlyEnforceIf(
            single_K[k, i, 0])

# link makespan to the maximum of the values in time_back_to_depot
model.AddMaxEquality(makespan, time_back_to_depot.values())
model.Minimize(makespan)
# [end add constraints]

# [start call the solver and prints out solutions]
status = solver.Solve(model)
if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
    print(solver.StatusName())
    print(solver.ObjectiveValue())
    for k in Vehicle:
        if solver.Value(time_back_to_depot[k]):
            print(f'v{k}', solver.Value(time_back_to_depot[k]))
    for i, j in permutations(Nodes, 2):
        for k in Vehicle:
            if solver.Value(single_K[k, i, j]):
                print(single_K[k, i, j])
else:
    print(solver.StatusName())
# [end call the solver and prints out solutions]
