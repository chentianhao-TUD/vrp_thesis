from ortools.sat.python import cp_model
from itertools import permutations
import numpy as np

# [start input data]
Nodes = [0, 1, 2, 3]
x = {}
arcs = []
model = cp_model.CpModel()
solver = cp_model.CpSolver()
for i, j in permutations(Nodes, 2):
    x[i, j] = model.NewBoolVar(f'{i} -> {j}')
    arcs.append((i, j, x[i, j]))
    model.AddExactlyOne(x[i, j])

print(x)
print(arcs)

model.AddCircuit(arcs)

p = cp_model.VarArraySolutionPrinter(x.values())
status = solver.SearchForAllSolutions(model, p)
solver.parameters.enumerate_all_solutions = True
# if status == cp_model.FEASIBLE or status == cp_model.OPTIMAL:
#     for i, j in permutations(Nodes, 2):
#         if solver.Value(x[i, j]):
#             print(i, j)

node = 0
count = 0
print(node, end="")
while count < 4:
    for i in Nodes:
        if i != node and solver.Value(x[node, i]):
            print(f" -> {i}", end="")
            node = i
            break
    else:
        break
    count += 1
