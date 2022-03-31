from ortools.sat.python import cp_model
from itertools import permutations
import numpy as np

# [start input data]
Nodes = [0, 1, 2, 3]
distance = {(0, 1): 63, (1, 0): 63, (0, 2): 67, (2, 0): 67, (1, 2): 50, (2, 1): 50, (0, 3): 100, (3, 0): 100, (1, 3): 162, (3, 1): 162, (2, 3): 161, (3, 2): 161}
demand = [0, 300, 500, 700]
# [end input data]

# [start initiate model and solver]
model = cp_model.CpModel()
solver = cp_model.CpSolver()
# [end initiate model and solver]

k = {}
for i in Nodes:
    for j in Nodes:
        k[i, j] = model.NewBoolVar(f'{i} --> {j}')

M = 99999

start_service = {}
for i in Nodes:
    start_service[i] = model.NewIntVar(0, 9999, f'start time at node{i}')
model.Add(start_service[0] == 0)

for i in Nodes:
    for j in Nodes[1:]:
        if i != j:
            start_service[j] = start_service[i] + demand[i] + distance[i, j] - (k[i, j] - 1) * M


t_list = []
for key, value in start_service:

obj = model.NewIntVar(0, 9999, 'max')
model.AddMaxEquality(obj, start_service.values())


model.Minimize(obj)
status = solver.Solve(model)
if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
    print(solver.ObjectiveValue())
    for i in Nodes:
        for j in Nodes:
            if solver.Value(k[i, j]) and solver.Value(start_service[i]):
                print(i, j, solver.Value(start_service[i]))
