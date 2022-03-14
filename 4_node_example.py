from ortools.sat.python import cp_model
import numpy as np

# input data, initiate two array to store variable x and distance
Nodes = [0, 1, 2, 3]
distance = {(0, 1): 63, (1, 0): 63, (0, 2): 67, (2, 0): 67, (1, 2): 50, (2, 1): 50, (0, 3): 100, (3, 0): 100, (1, 3): 162, (3, 1): 162, (2, 3): 161, (3, 2): 161}
demand = [0, 300, 500, 700]
x_var = np.empty([1, 4, 4], dtype=object)
distance_var = np.empty([1, 4, 4], dtype=object)
start_service = np.empty(4, dtype=object)
service_time = np.empty(4, dtype=object)
print('x_var before assignment \n', x_var, '\n')

# create a model, assign boolean variable to the array x_var and integer variable to distance_var
# if i =j, no vehicle travel between the same node --> boolean value = 0, distance between them is set to 0
# if i != j, put distance into the array
model = cp_model.CpModel()
for i in Nodes:
    for j in Nodes:
        x_var[0, i, j] = model.NewBoolVar('vehicle 0 travel from {} to {}'.format(i, j))
        distance_var[0, i, j] = model.NewIntVar(0, 1000, "distance from {} to {}".format(i, j))

for j in Nodes:
    service_time[j] = model.NewIntVar(0, 10000, 'service time at node {}'.format(j))
    start_service[j] = model.NewIntVar(0, 10000, 'start service time at node {}'.format(j))
model.Add(start_service[0] == 0)

for i in Nodes:
    for j in Nodes:
        if j != 0:
            model.Add(start_service[j] == start_service[i] + distance_var[0, i, j]).OnlyEnforceIf(x_var[0, i, j])



print('x_var after assignment\n', x_var, '\n')
print('distance var\n', distance_var, '\n')
print('type of x_var element', type(x_var[0, 0, 1]), '\n')

# if a node is visited, the car must leave this node
for i in Nodes:
    for j in Nodes:
        model.Add(sum(x_var[0, :, i]) == 1)  # every node should be visited once
        model.Add(sum(x_var[0, j, :]) == 1).OnlyEnforceIf(x_var[0, i, j])  # vehicle visit a node also leaves it
        # model.Add(x_var[0, j, i] == 0).OnlyEnforceIf(x_var[0, i, j])
        if i == j:
            model.Add(x_var[0, i, j] == 0)
            model.Add(distance_var[0, i, j] == 0)
        else:
            model.Add(distance_var[0, i, j] == distance[(i, j)]).OnlyEnforceIf(x_var[0, i, j])

print('service time is', type(service_time))

# max_of_service_time = np.amax(service_time)
# index_of_max = np.where(service_time == max_of_service_time)
# makespan = max_of_service_time + distance[(index_of_max, 0)]


# model.Minimize(sum(sum_distance))
model.Minimize(makespan)
solver = cp_model.CpSolver()
status = solver.Solve(model)
# turn on the log if needed
# solver.parameters.log_search_progress = True
solver.parameters.enumerate_all_solutions = True
solver.Solve(model)
distance_solution = []
if status == cp_model.OPTIMAL:
    print('found optimal solution\n')
    for i in Nodes:
        for j in Nodes:
            if solver.Value(x_var[0, i, j]):
                print('vehicle is travelling from {} to {}'.format(i, j))
                distance_solution.append(distance[(i, j)])
else:
    print('failed')

print('total distance in solution\n', sum(distance_solution))

