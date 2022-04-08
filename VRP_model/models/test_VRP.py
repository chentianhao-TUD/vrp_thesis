from CRVRP import generate_input_data
import os

os.chdir('..')
path = 'instances/5/5-2-7-100.xml'
data = generate_input_data(path, 4)
print(data["corresponding_nodes"])