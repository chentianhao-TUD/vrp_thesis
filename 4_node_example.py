from ortools.sat.python import cp_model
import numpy as np
from itertools import permutations

# input data, initiate two array to store variable x and distance
Nodes = [0, 1, 2, 3]
distance = {(0, 1): 63, (1, 0): 63, (0, 2): 67, (2, 0): 67, (1, 2): 50, (2, 1): 50, (0, 3): 100, (3, 0): 100, (1, 3): 162, (3, 1): 162, (2, 3): 161, (3, 2): 161}
demand = [0, 300, 500, 700]
x_var = {}
distance_var = {}
arcs = []
start_service = np.empty(4, dtype=object)
service_time = np.empty(4, dtype=object)


# create a model, assign boolean variable to the array x_var and integer variable to distance_var
# if i =j, no vehicle travel between the same node --> boolean value = 0, distance between them is set to 0
# if i != j, put distance into the array
model = cp_model.CpModel()
for i, j in permutations(Nodes, 2):
    x_var[0, i, j] = model.NewBoolVar(f'v0 {i} -> {j}')
    distance_var[0, i, j] = model.NewIntVar(0, 1000, "distance from {} to {}".format(i, j))
    arcs.append((i, j, x_var[0, i, j]))
    model.Add(distance_var[0, i, j] == distance[(i, j)]).OnlyEnforceIf(x_var[0, i, j])

model.AddCircuit(arcs)
print('x_var\n', x_var, '\n')
print('arcs\n', arcs, '\n')
print('distance var\n', distance_var, '\n')

for j in Nodes:
    service_time[j] = model.NewIntVar(0, 10000, 'service time at node {}'.format(j))
    start_service[j] = model.NewIntVar(0, 10000, 'start service time at node {}'.format(j))
    model.Add(service_time[j] == demand[j])  # link modes with robots later

model.Add(start_service[0] == 0)
print('service time is', service_time)

# if a node is visited, the car must leave this node
count = 0
tem_1 = []
tem_2 = []

while count < 4:
    for key in x_var.keys():
        if key[2] == count:
            tem_1.append(x_var[key])
        if key[1] == count:
            tem_2.append(x_var[key])
    model.Add(sum(tem_1) == 1)
    model.Add(sum(tem_2) == 1)
    print(tem_1)
    print(tem_2)
    tem_1.clear()
    tem_2.clear()
    count += 1

# calculation of start service at a node
# for i, j in permutations(Nodes, 2):
#     model.Add(start_service[j] >= start_service[i] + distance_var[0, i, j] + service_time[i]).OnlyEnforceIf(x_var[0, i, j])


max_start_service = model.NewIntVar(0, 10000, 'start time on last node')
model.AddMaxEquality(max_start_service, start_service)
# makespan = max_start_service + distance_var[0, route[len(Nodes) - 1], 0]
#
# model.Minimize(makespan)
solution_printer = cp_model.VarArraySolutionPrinter(start_service)
solver = cp_model.CpSolver()
status = solver.Solve(model, solution_printer)
# status = solver.SearchForAllSolutions(model, solution_printer)
# turn on the log if needed
# solver.parameters.log_search_progress = True
# solver.Solve(model)
solver.parameters.enumerate_all_solutions = True
if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
    print('found optimal solution\n')
    print(solver.ObjectiveValue())
    for i, j in permutations(Nodes, 2):
        if solver.Value(x_var[0, i, j]) and solver.Value(start_service[j]):
            print('vehicle is travelling from {} to {}'.format(i, j))
            print('start time of service at {} is'.format(j), solver.Value(start_service[j]))
            print('service time at node {} is'.format(j), solver.Value(service_time[j]))

else:
    print('failed')


# print('max_start_service is', solver.Value(max_start_service))
# print(np.where(start_service == solver.Value(max_start_service)))


