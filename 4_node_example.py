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
    model.Add(service_time[j] == demand[j])  # link modes with robots later
model.Add(start_service[0] == 0)




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

# calculation of start service at a node
for i in Nodes:
    for j in Nodes[1:]:
        model.Add(start_service[j] >= start_service[i] + distance_var[0, i, j] + service_time[i]).OnlyEnforceIf(x_var[0, i, j])

max_start_service = model.NewIntVar(0, 10000, 'max service time')
model.AddMaxEquality(max_start_service, start_service)

# makespan = max_start_service + distance_var[0, index_of_last_node, 0]


model.Minimize(max_start_service)
solver = cp_model.CpSolver()
status = solver.Solve(model)
# turn on the log if needed
# solver.parameters.log_search_progress = True
solver.parameters.enumerate_all_solutions = True
solver.Solve(model)

if status == cp_model.OPTIMAL:
    print('found optimal solution\n')
    for i in Nodes:
        for j in Nodes:
            if solver.Value(x_var[0, i, j]) and solver.Value(start_service[j]):
                print('vehicle is travelling from {} to {}'.format(i, j))
                print('start time of service at {} is'.format(j), solver.Value(start_service[j]))
                print('service time at node {} is'.format(j), solver.Value(service_time[j]))

else:
    print('failed')


print('max_start_service is', solver.Value(max_start_service))
# print(np.where(start_service == solver.Value(max_start_service)))

