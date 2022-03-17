from itertools import permutations
from ortools.sat.python import cp_model

model = cp_model.CpModel()
solver = cp_model.CpSolver()

literals = {}
# An arc is just a (int, int, BoolVar) tuple
all_arcs = []
nodes = range(1, 11)
print(type(nodes))

for i in nodes:
    # We use 0 as a dummy nodes as we don't have an actual circuit
    literals[0, i] = model.NewBoolVar(f"0 -> {i}")  # start arc
    literals[i, 0] = model.NewBoolVar(f"{i} -> 0")  # end arc
    all_arcs.append([0, i, literals[0, i]])
    all_arcs.append([i, 0, literals[i, 0]])


for i, j in permutations(nodes, 2):
    # this booleans will be true if the arc is present
    literals[i, j] = model.NewBoolVar(f"{i} -> {j}")
    all_arcs.append([i, j, literals[i, j]])
# to make an arc optional, add the [i, i, True] loop
print(type(sum(literals[i, j] * abs(i - j) for i, j in permutations(nodes, 2))))

model.AddCircuit(all_arcs)
model.Maximize(sum(literals[i, j] * abs(i - j) for i, j in permutations(nodes, 2)))
solver.Solve(model)

node = 0
print(node, end="")
while True:
    for i in nodes:
        if i != node and solver.Value(literals[node, i]):
            print(f" -> {i}", end="")
            node = i
            break
    else:
        break
print(" -> 0")


