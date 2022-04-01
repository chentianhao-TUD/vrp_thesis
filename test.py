from ortools.sat.python import cp_model
import math
from itertools import permutations
# [start input data]
Nodes = [0, 1, 2, 3]
distance = {(0, 1): 63, (1, 0): 63, (0, 2): 67, (2, 0): 67, (1, 2): 50, (2, 1): 50, (0, 3): 100, (3, 0): 100,
            (1, 3): 162, (3, 1): 162, (2, 3): 161, (3, 2): 161}
demand = [0, 300, 500, 700]
mode_amount = [0, 2, 3, 4]
Vehicle = [0, 1]
Robot = [0, 1, 2, 3, 4, 5]
Capacity = 2

model = cp_model.CpModel()
solver = cp_model.CpSolver()

single_K = {}  # BoolVar, single_K[k, i, j] =1 if a primary vehicle k travels from i to j
any_K = {}  # BoolVar, any_k[i, j] = 1 if there is at least one primary vehicle travels from i to j
single_R = {}
any_R = {}
start_time_K = {}  # IntVar, start_time[i] is the time that a vehicle k arrives at node i
start_time_R = {}
time_back_to_depot_K = {}  # IntVar, time_back_to_depot[k] is the time that a vehicle k returns to depot
time_back_to_depot_R = {}
makespan = model.NewIntVar(0, 9999, 'makespan')  # the time that the last vehicle returns to depot

for i, j in permutations(Nodes, 2):
    for k in Vehicle:
        single_K[k, i, j] = model.NewBoolVar(f'v{k} {i} -> {j}')
    for r in Robot:
        single_R[r, i, j] = model.NewBoolVar(f'r{r} {i} -> {j}')
    any_R[i, j] = model.NewBoolVar(f'any r {i} -> {j}')
    any_K[i, j] = model.NewBoolVar(f'any k {i} -> {j}')

for i in Nodes:
    for k in Vehicle:
        start_time_K[i] = model.NewIntVar(0, 9999, f'start time k at node{i}')
    for r in Vehicle:
        start_time_R[i] = model.NewIntVar(0, 9999, f'start time r at node {i}')
for i, j in permutations(Nodes, 2):
    for k in Vehicle:
        model.AddImplication(single_K[k, i, j], any_K[i, j])
        model.AddImplication(any_K[i, j].Not(), single_K[k, i, j].Not())
    for r in Robot:
        model.AddImplication(single_R[r, i, j], any_R[i, j])
        model.AddImplication(any_R[i, j].Not(), single_R[r, i, j].Not())
    model.AddImplication(any_R[i, j], any_K[i, j])
    model.AddImplication(any_K[i, j].Not(), any_R[i, j].Not())

# each nodes should be visited at least once
# and if a node j is visited by vehicle k, k should also leave j
for j in Nodes[1:]:
    model.AddAtLeastOne(single_R[r, i, j] for r in Robot for i in Nodes if i != j)
    for k in Vehicle:
        model.Add(sum(single_K[k, i, j] for i in Nodes if i != j) == sum(single_K[k, j, i] for i in Nodes if i != j))
    for r in Robot:
        model.Add(sum(single_R[r, i, j] for i in Nodes if i != j) == sum(single_R[r, j, i] for i in Nodes if i != j))

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
model.Add(start_time_R[0] == 0)
model.Add(start_time_K[0] == 0)

# start time at j, is big M necessary?
for i, j in permutations(Nodes, 2):
    model.Add(start_time_R[j] >= start_time_R[i] + demand[i] + distance[i, j]).OnlyEnforceIf(
                any_R[i, j])
    model.Add(start_time_K[j] >= start_time_K[i] + distance[i, j]).OnlyEnforceIf(
                any_K[i, j])

# the time that a vehicle k returns to the depot

for k in Vehicle:
    for i in Nodes[1:]:
        time_back_to_depot_K[k] = model.NewIntVar(0, 9999, f'v{k} back to depot')
        model.Add(time_back_to_depot_K[k] >= start_time_K[i] + distance[i, 0]).\
            OnlyEnforceIf(single_K[k, i, 0])
for r in Robot:
    for i in Nodes[1:]:
        time_back_to_depot_R[r] = model.NewIntVar(0, 9999, f'r{r} back to depot')
        model.Add(time_back_to_depot_R[r] >= start_time_R[i] + demand[i] + distance[i, 0]).OnlyEnforceIf(
            single_R[r, i, 0])

print(time_back_to_depot_R, time_back_to_depot_K)
status = solver.Solve(model)
if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
    print(solver.StatusName())
    print(solver.ObjectiveValue())
    for i, j in permutations(Nodes, 2):
        for k in Vehicle:
            if solver.Value(single_K[k, i, j]):
                print(single_K[k, i, j])
        for r in Robot:
            if solver.Value(single_R[r, i, j]):
                print(single_R[r, i, j])
else:
    print(solver.StatusName())
