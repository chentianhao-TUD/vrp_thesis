#! /usr/bin/env python

from ortools.sat.python import cp_model
from itertools import permutations

# [start input data]
Nodes = [0, 1, 2, 3]
distance = {(0, 1): 63, (1, 0): 63, (0, 2): 67, (2, 0): 67, (1, 2): 50, (2, 1): 50, (0, 3): 100, (3, 0): 100,
            (1, 3): 162, (3, 1): 162, (2, 3): 161, (3, 2): 161}
demand = [0, 300, 500, 700]
mode_amount = [0, 3, 4, 6]
Vehicle = [0, 1, 2]
Robot = [0, 1, 2, 3, 4, 5, 6, 7, 8]
Capacity = 3
# [end input data]

# [start initiate model and solver]
model = cp_model.CpModel()
solver = cp_model.CpSolver()
# [end initiate model and solver]

# [start create variables]
single_K = {} # BoolVar, single_K[k, i, j] =1 if a primary vehicle k travels from i to j
any_K = {}  # BoolVar, any_k[i, j] = 1 if there is at least one primary vehicle travels from i to j
single_R = {}
any_R = {}
time_back_to_depot = {}  # IntVar, time_back_to_depot[k] is the time that a vehicle k returns to depot
time_back_to_depot_R = {}
start_time = {}  # IntVar, start_time[i] is the time that a vehicle k arrives at node i
start_time_R = {}
service_time = {}  # in this basic model, the service_time[i] is a constant value
mode = {}
makespan = model.NewIntVar(0, 9999, 'makespan')  # the time that the last vehicle returns to depot

for k in Vehicle:
    for i in Nodes:
        start_time[i] = model.NewIntVar(0, 9999, f'start time at node{i}')
    time_back_to_depot[k] = model.NewIntVar(0, 9999, f'v{k} back to depot')
for k in Vehicle:
    for i, j in permutations(Nodes, 2):
        single_K[k, i, j] = model.NewBoolVar(f'v{k} {i} -> {j}')
        any_K[i, j] = model.NewBoolVar(f'v {i} -> {j}')

for r in Robot:
    time_back_to_depot_R[r] = model.NewIntVar(0, 9999, f'r{r} back to depot')
    for i, j in permutations(Nodes, 2):
        single_R[r, i, j] = model.NewBoolVar(f'r{r} {i} -> {j}')
        any_R[i, j] = model.NewBoolVar(f'r {i} -> {j}')

for r in Robot:
    for i in Nodes:
        start_time_R[r] = model.NewIntVar(0, 9999, f'start time R at node{i}')

for i in Nodes:
    mode[i] = model.NewIntVar(1, len(Robot), f'mode at node{i}')
    service_time[i] = model.NewIntVar(0, demand[i], f'service time at node{i}')
# [end create variables]

# [start add constraints]
# link any_k to single_k

for i, j in permutations(Nodes, 2):
    for k in Vehicle:
        model.Add(any_K[i, j] == 1).OnlyEnforceIf(single_K[k, i, j])
        model.Add(single_K[k, i, j] == 0).OnlyEnforceIf(any_K[i, j].Not())
    for r in Robot:
        model.Add(any_R[i, j] == 1).OnlyEnforceIf(single_R[r, i, j])
        model.Add(single_R[r, i, j] == 0).OnlyEnforceIf(any_R[i, j].Not())
    model.AddAtLeastOne(single_K[k, i, j] for k in Vehicle).OnlyEnforceIf(any_R[i, j])
    model.AddAtLeastOne(single_K[k, i, j] for k in Vehicle).OnlyEnforceIf(any_K[i, j])
    model.AddAtLeastOne(single_R[r, i, j] for r in Robot).OnlyEnforceIf(any_R[i, j])
    model.AddImplication(any_R[i, j], any_K[i, j])
# each node should be visited at least once
# and if a node j is visited by vehicle k, k should also leave j
for j in Nodes:
    model.AddAtLeastOne(single_K[k, i, j] for k in Vehicle for i in Nodes if i != j)
    model.AddAtLeastOne(single_R[r, i, j] for r in Robot for i in Nodes if i != j)

for h in Nodes:
    for k in Vehicle:
        model.Add(sum(single_K[k, i, h] for i in Nodes if i != h) == sum(single_K[k, h, j] for j in Nodes if j != h))
    for r in Robot:
        model.Add(sum(single_R[r, i, h] for i in Nodes if i != h) == sum(single_R[r, h, j] for j in Nodes if j != h))

# every vehicle should return to depot
for k in Vehicle:
    model.AddExactlyOne(single_K[k, i, 0] for i in Nodes[1:])
for r in Robot:
    model.AddExactlyOne(single_R[r, i, 0] for i in Nodes[1:])

# every vehicle should leave depot
for k in Vehicle:
    model.AddExactlyOne(single_K[k, 0, j] for j in Nodes[1:])
for r in Robot:
    model.AddExactlyOne(single_R[r, 0, j] for j in Nodes[1:])

# start time at depot is 0
model.Add(start_time[0] == 0)
model.Add(start_time_R[0] == 0)

# the number of robots travel from i to j is smaller than the number of vehicles * Capacity
for i, j in permutations(Nodes, 2):
    model.Add(sum(single_R[r, i, j] for r in Robot) <= sum(single_K[k, i, j] * Capacity for k in Vehicle)).OnlyEnforceIf(
        any_R[i, j])
# service time at j depends on mode at j
# mode j is smaller than min(mode_amount[j], sum(single_K[k, i, j] * Capacity)
# sum(single_R[r, i, j]) = mode j) if any_R
for i, j in permutations(Nodes, 2):
    model.Add(sum(single_R[r, i, j] for r in Robot) == mode[j]).OnlyEnforceIf(any_R[i, j])
    model.Add(mode[j] <= sum(single_K[k, i, j] for k in Vehicle) * Capacity).OnlyEnforceIf(any_K[i, j])

for j in Nodes[1:]:
    model.AddDivisionEquality(service_time[j], demand[j], mode[j])
    model.Add(mode[j] <= mode_amount[j])
model.Add(service_time[0] == 0)

# start time at j, is big M necessary?
for i, j in permutations(Nodes, 2):
    if j != 0:
        model.Add(start_time[j] >= start_time[i] + distance[i, j]).OnlyEnforceIf(any_K[i, j])
        model.Add(start_time_R[j] >= start_time_R[i] + service_time[i] + distance[i, j]).OnlyEnforceIf(any_R[i, j])
for j in Nodes:
    model.Add(start_time[j] == start_time_R[j])
# the time that a vehicle k returns to the depot
for k in Vehicle:
    for i in Nodes[1:]:
        model.Add(time_back_to_depot[k] >= start_time[i] + service_time[i] + distance[i, 0]).OnlyEnforceIf(
            single_K[k, i, 0])
for r in Robot:
    for i in Nodes[1:]:
        model.Add(time_back_to_depot_R[r] >= start_time_R[i] + service_time[i] + distance[i, 0]).OnlyEnforceIf(
            single_R[r, i, 0])

# link makespan to the maximum of the values in time_back_to_depot
max_K = model.NewIntVar(0, 9999, f'max K')
max_R = model.NewIntVar(0, 9999, f'max R')
model.AddMaxEquality(max_K, time_back_to_depot.values())
model.AddMaxEquality(max_R, time_back_to_depot_R.values())
max = [max_K, max_R]
model.AddMaxEquality(makespan, max)
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
    for r in Robot:
        if solver.Value(time_back_to_depot_R[r]):
            print(f'r{r}', solver.Value(time_back_to_depot_R[r]))

    # for i, j in permutations(Nodes, 2):
    #     for k in Vehicle:
    #         if solver.Value(single_K[k, i, j]):
    #             print(single_K[k, i, j])
    # for i, j in permutations(Nodes, 2):
    #     for r in Robot:
    #         if solver.Value(single_R[r, i, j]):
    #             print(single_R[r, i, j])
    # for r in Robot:
    #     if solver.Value(time_back_to_depot_R[r]):
    #         print(f'Robot{r} return to depot', solver.Value(time_back_to_depot_R[r]))
    for i, j in permutations(Nodes, 2):
        if solver.Value(any_R[i, j]):
            print(sum(solver.Value(single_R[r, i, j])for r in Robot), f'robots {i} -> {j}')
        if solver.Value(any_K[i, j]):
            print(sum(solver.Value(single_K[k, i, j])for k in Vehicle), f'vehicles {i} -> {j}')

else:
    print(solver.StatusName())
# [end call the solver and prints out solutions]
