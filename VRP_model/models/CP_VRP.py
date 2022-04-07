from ortools.sat.python import cp_model
from CRVRP import generate_input_data
import os
import read_xml

number_customer = 5
directory = os.path.join("/home/louis/DA/VRP_model/instances", str(number_customer))
for entry in os.scandir(directory):
    if entry.path.endswith(".xml") and entry.is_file():
        instance_configuration = entry.path[len(directory) +
                                            1:len(entry.path) - 4]
        print(instance_configuration)
path = os.path.join(directory, instance_configuration + '.xml')
print(path)

Nodes, demand, mode_amount, loc_x, loc_y, SetOfVehicles, SetOfRobots = read_xml.read_xml_file(path, 1)
data = generate_input_data(path, 1)
all_nodes = data['all_nodes']
distance = data['distance']
model = cp_model.CpModel()

w_var = []
v_var = []
y_var = []
start_of_service = 0
service_time_var = []


def get_x():
    x_var = []
    for k in SetOfVehicles:
        for i in all_nodes[:-1]:
            for j in all_nodes[1:]:
                if i != j:
                    x = model.NewBoolVar(("{}, {}, {}".format(k, i, j)))
                    x_var.append(x)
    return x_var


for i in all_nodes[:-1]:
    for j in all_nodes[1:]:
        if i != j:
            w = model.NewIntVar(0, data["transport_capacity"], 'w')
            w_var.append((i, j, w))


for r in SetOfRobots:
    for i in all_nodes[:-1]:
        for j in all_nodes[1:]:
            if i != j:
                v = model.NewBoolVar('robot {} is travel from {} to {}'.format(r, i, j))
                v_var.append(v)


for m in data['modes']:
    for j in Nodes:
        y = model.NewBoolVar('field {} is served under mode {}'.format(j, m))
        y_var.append(y)
        service_time = demand[j] / m
        service_time_var.append((m, service_time))




x_var = get_x()
print('x_var', type(x_var[1]), x_var[1])

solver = cp_model.CpSolver()
status = solver.Solve(model)
solver.parameters.log_search_progress = True
solver.parameters.enumerate_all_solutions = True

if status == cp_model.OPTIMAL:
    print(1111)
elif status == cp_model.FEASIBLE:
    print(2222)
elif status == cp_model.INFEASIBLE:
    print(3333)





