from ortools.sat.python import cp_model
from itertools import permutations
import numpy as np

# [start input data]
SetOfVehicle = [0, 1]
Nodes = [0, 1, 2, 3]
distance = {(0, 1): 63, (1, 0): 63, (0, 2): 67, (2, 0): 67, (1, 2): 50, (2, 1): 50, (0, 3): 100, (3, 0): 100, (1, 3): 162, (3, 1): 162, (2, 3): 161, (3, 2): 161}
demand = [0, 300, 500, 700]
# [end input data]

# [start initiate model and solver]
model = cp_model.CpModel()
solver = cp_model.CpSolver()
# [end initiate model and solver]

# [start create decision variables]
# ?? x put in dictionary or a np.array??
x_var = np.empty([len(SetOfVehicle), len(Nodes), len(Nodes)], dtype=object)
distance_var = np.empty([len(Nodes), len(Nodes)], dtype=object)
service_time = np.empty(len(Nodes), dtype=object)
start_service = np.empty(len(Nodes), dtype=object)
mode = np.empty(len(Nodes), dtype=object)
arcs = [[] for i in range(len(SetOfVehicle))]
for i in Nodes:
    service_time[i] = model.NewIntVar(0, 10000, f'service time at node {i}')
    start_service[i] = model.NewIntVar(0, 10000, f'start service time at node {i}')
    mode[i] = model.NewIntVar(1, len(SetOfVehicle), f'mode at Node{i}')
    for j in Nodes:
        distance_var[i, j] = model.NewIntVar(0, 10000, f'distance {i} -> {j}')
        for k in SetOfVehicle:
            x_var[k, i, j] = model.NewBoolVar(f'x{k} {i} -> {j}')

for i, j in permutations(Nodes, 2):
    for k in SetOfVehicle:
        arcs[k].append((i, j, x_var[k, i, j]))
print('x_var\n', x_var, '\n')
print('distance_var \n', distance_var, '\n')
print('service time \n', service_time, '\n')
print('start service \n', start_service, '\n')
print('arcs\n', arcs, '\n')
# [end create variables]

# [start add constraints]
# 1, the distance between two identical nodes is 0
#    the distance between two different nodes equals to the input data
# 2, x_var on i = j equals to 0
# 3, start_service[j]
# 4, all node must be visited once and left once
# 5, service time at node i equals to input data
model.Add(start_service[0] == 0)

for i in Nodes:
    for j in Nodes[1:]:
        model.Add(mode[j] == sum(x_var[:, i, j]))
print(mode)

for k in SetOfVehicle:
    model.AddCircuit(arcs[k])
for i in Nodes:
    for j in Nodes:
        if i != j:
            model.Add(distance_var[i, j] == distance[i, j])
        else:
            model.Add(distance_var[i, j] == 0)

for i in Nodes:
    for j in Nodes:
        for k in SetOfVehicle:
            if i == j:
                model.Add(x_var[k, i, j] == 0)

for i in Nodes:
    for j in Nodes[1:]:
        model.Add(start_service[j] >= start_service[i] + distance_var[i, j] + service_time[i]).OnlyEnforceIf(x_var[:, i, j])

for i in Nodes:
    model.Add(service_time[i] == demand[i])
    for k in SetOfVehicle:
        model.Add(sum(x_var[k, :, i]) >= 1)
        model.Add(sum(x_var[k, i, :]) >= 1)
        print(x_var[k, :, i])
        print(x_var[k, i, :])



# [end add constraints]

# [start objective function]

max_start_service = model.NewIntVar(0, 10000, f'max start time')
model.AddMaxEquality(max_start_service, start_service)
last_node = 0
for i, j in permutations(Nodes, 2):
    if j == 0:
        last_node = i
print(last_node)
# makespan = max_start_service + distance_var[last_node, 0] + service_time[last_node]
# model.Minimize(makespan)
# [start call the solver]
solution_printer = cp_model.VarArraySolutionPrinter(start_service)
status = solver.Solve(model, solution_printer)
solver.parameters.enumerate_all_solutions = True
# [end call the solver]
route = []
if status == cp_model.FEASIBLE:
    print('feasible')
elif status == cp_model.OPTIMAL:
    print('optimal')
    for i in Nodes:
        for j in Nodes:
            for k in SetOfVehicle:
                if solver.Value(x_var[k, i, j]):
                    print(f'V{k} {i} -> {j}')
                    print('start time of service at {} is'.format(j), solver.Value(start_service[j]))
                    print('service time at node {} is'.format(j), solver.Value(service_time[j]))
    # node = 0
    # count = 0
    # print(node, end="")
    # while count < 4:
    #     for i in Nodes:
    #         if i != node and solver.Value(x_var[0, node, i]):
    #             print(f" -> {i}", end="")
    #             node = i
    #             break
    #     else:
    #         break
    #     count += 1


else:
    print('failed')
print('\n')
# print(solver.Value(makespan))
print(last_node)
