#! /usr/bin/env python

from ortools.sat.python import cp_model
import math

# [start input data]
Nodes = [0, 1, 2, 3]
distance = {(0, 1): 63, (1, 0): 63, (0, 2): 67, (2, 0): 67, (1, 2): 50, (2, 1): 50, (0, 3): 100, (3, 0): 100,
            (1, 3): 162, (3, 1): 162, (2, 3): 161, (3, 2): 161}
demand = [0, 300, 500, 700]
mode_amount = [0, 2, 3, 4]
Vehicle = [0, 1]
Robot = [0, 1, 2, 3, 4, 5]
Capacity = 2

# [end input data]

# [start initiate model and solver]
model = cp_model.CpModel()
solver = cp_model.CpSolver()
# [end initiate model and solver]

# [start create variables]
single_K = {}  # BoolVar, single_K[k, i, j] =1 if a primary vehicle k travels from i to j
any_K = {}  # BoolVar, any_k[i, j] = 1 if there is at least one primary vehicle travels from i to j
single_R = {}
any_R = {}
time_back_to_depot_K = {}  # IntVar, time_back_to_depot[k] is the time that a vehicle k returns to depot
time_back_to_depot_R = {}
mode = {}
service_time ={}
start_time_K = {}  # IntVar, start_time[i] is the time that a vehicle k arrives at node i
start_time_R = {}
makespan = model.NewIntVar(0, 9999, 'makespan')  # the time that the last vehicle returns to depot

for k in Vehicle:
    for i in Nodes:
        start_time_K[i] = model.NewIntVar(0, 9999, f'start time v{k} at node{i}')
        for j in Nodes:
            single_K[k, i, j] = model.NewBoolVar(f'v{k} {i} -> {j}')
            any_K[i, j] = model.NewBoolVar(f'v {i} -> {j}')
    time_back_to_depot_K[k] = model.NewIntVar(0, 9999, f'v{k} back to depot')

for r in Robot:
    for i in Nodes:
        start_time_R[i] = model.NewIntVar(0, 9999, f'start time r{r} at node{i}')
        for j in Nodes:
            single_R[r, i, j] = model.NewBoolVar(f'r{r} {i} -> {j}')
            any_R[i, j] = model.NewBoolVar(f'r {i} -> {j}')
    time_back_to_depot_R[r] = model.NewIntVar(0, 9999, f'r{r} back to depot')

for i in Nodes:
    service_time[i] = model.NewIntVar(0, 9999, f'service time at node{i}')


# [end create variables]

# [start add constraints]
# link any_k to single_k
for k in Vehicle:
    for i in Nodes:
        for j in Nodes:
            model.Add(any_K[i, j] == 1).OnlyEnforceIf(single_K[k, i, j])
            model.Add(single_K[k, i, j] == 0).OnlyEnforceIf(any_K[i, j].Not())

for r in Robot:
    for i in Nodes:
        for j in Nodes:
            model.Add(any_R[i, j] == 1).OnlyEnforceIf(single_R[r, i, j])
            model.Add(single_R[r, i, j] == 0).OnlyEnforceIf(any_R[i, j].Not())

for i in Nodes:
    for j in Nodes:
        model.AddImplication(any_R[i, j], any_K[i, j])
        model.AddImplication(any_K[i, j], any_R[i, j])
        model.Add(sum(single_R[r, i, j] for r in Robot) <= mode_amount[j])
        model.Add(sum(single_R[r, i, j] for r in Robot) <= Capacity *
                  sum(single_K[k, i, j] for k in Vehicle))
# no vehicle or robot travels between two identical nodes
for i in Nodes:
    for j in Nodes:
        if i == j:
            model.Add(any_K[i, j] == 0)
            model.Add(any_R[i, j] == 0)
        for k in Vehicle:
            if i == j:
                model.Add(single_K[k, i, j] == 0)
        for r in Robot:
            if i == j:
                model.Add(single_R[r, i, j] == 0)
# each nodes should be visited at least once
# and if a node j is visited by vehicle k, k should also leave j
for j in Nodes:
    model.AddAtLeastOne(single_K[k, i, j] for k in Vehicle for i in Nodes)
    model.AddAtLeastOne(single_R[r, i, j] for r in Robot for i in Nodes)
    for k in Vehicle:
        model.Add(sum(single_K[k, i, j] for i in Nodes) == sum(single_K[k, j, i] for i in Nodes))
    for r in Robot:
        model.Add(sum(single_R[r, i, j] for i in Nodes) == sum(single_R[r, j, i] for i in Nodes))
# every vehicle should return to depot
for k in Vehicle:
    model.AddExactlyOne(single_K[k, i, 0] for i in Nodes)
for r in Robot:
    model.AddExactlyOne(single_R[r, i, 0] for i in Nodes)
# every vehicle should leave depot
for k in Vehicle:
    model.AddExactlyOne(single_K[k, 0, j] for j in Nodes[1:])
for r in Robot:
    model.AddExactlyOne(single_R[r, 0, j] for j in Nodes[1:])
# start time at depot is 0
model.Add(start_time_K[0] == 0)
model.Add(start_time_R[0] == 0)
# mode
for i in Nodes:
    mode[i] = model.NewIntVar(0, mode_amount[i], f'mode at node{i}')
print(mode)
service_time_domain = {}
for i in Nodes:
    for value in list(range(mode_amount[i]))[1:]:
        service_time_domain[i] = []
        service_time_domain[i].append(math.ceil(demand[i] / value))

# service time depends on mode
# for i in Nodes:
#     model.AddDivisionEquality(service_time[i], demand[i], mode[i])
    # model.Add(service_time[i] == demand[i] / mode[i])
# start time at j, is big M necessary?
for i in Nodes:
    for j in Nodes[1:]:
        if i != j:
            model.Add(start_time_K[j] >= start_time_K[i] + distance[i, j]).OnlyEnforceIf(
                any_K[i, j])
            model.Add(start_time_R[j] >= start_time_R[i] + service_time[i] + distance[i, j]).OnlyEnforceIf(
                any_R[i, j])
            model.Add(start_time_R[j] <= start_time_K[j])

# the time that a vehicle k returns to the depot
for k in Vehicle:
    for i in Nodes[1:]:
        model.Add(time_back_to_depot_K[k] >= start_time_K[i] + distance[i, 0]).OnlyEnforceIf(
            single_K[k, i, 0])

for r in Robot:
    for i in Nodes[1:]:
        model.Add(time_back_to_depot_R[r] >= start_time_R[i] + service_time[i] + distance[i, 0]).\
            OnlyEnforceIf(single_R[r, i, 0])
# link makespan to the maximum of the values in time_back_to_depot
max_K = model.NewIntVar(0, 9999, f'max k')
max_R = model.NewIntVar(0, 9999, f'max r')
max_sum = [max_R, max_K]
model.AddMaxEquality(max_K, time_back_to_depot_K.values())
model.AddMaxEquality(max_R, time_back_to_depot_R.values())
model.AddMaxEquality(makespan, max_sum)
model.Minimize(makespan)
# [end add constraints]

# [start call the solver and prints out solutions]
status = solver.Solve(model)
if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
    print(solver.StatusName())
    print(solver.ObjectiveValue())
    for k in Vehicle:
        if solver.Value(time_back_to_depot_K[k]):
            print(f'v{k}', solver.Value(time_back_to_depot_K[k]))
    for r in Robot:
        if solver.Value(time_back_to_depot_R[r]):
            print(f'v{r}', solver.Value(time_back_to_depot_R[r]))

    for i in Nodes:
        for j in Nodes:
            for k in Vehicle:
                if solver.Value(single_K[k, i, j]):
                    print(single_K[k, i, j])
            for r in Robot:
                if solver.Value(single_R[r, i, j]):
                    print(single_R[r, i, j])

else:
    print(solver.StatusName())
# [end call the solver and prints out solutions]
