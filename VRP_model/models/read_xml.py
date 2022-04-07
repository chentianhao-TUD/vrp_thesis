import os
from xml.etree import ElementTree as ET

path = 'instances/5/5-2-7-100.xml'


def count_instances(path):
    tree = ET.parse(path)
    root = tree.getroot()
    instance_amount = len(root.getchildren())
    return instance_amount


def read_xml_file(path, instance_number):
    tree = ET.parse(path)
    root = tree.getroot()

    # instance = configuration[]
    HARVESTER_AMOUNT = int(root[instance_number][0][0].text)
    K_p = [i for i in range(1, HARVESTER_AMOUNT+1)]
    TRANSPORTER_AMOUNT = int(root[instance_number][0][1].text)
    K_s = [i for i in range(1, TRANSPORTER_AMOUNT + 1)]

    N = []
    n = int(root[instance_number][1][1].text)
    loc_x = []
    loc_y = []
    demand = {}
    mode_amount = {}

    for vertex in root[instance_number][1][1]:
        number = int(vertex[0].text)
        x = float(vertex[1].text)
        y = float(vertex[2].text)
        d = int(vertex[3].text)
        mode = int(vertex[4].text)

        N.append(number)
        loc_x.append(x)
        loc_y.append(y)
        demand[number] = d
        mode_amount[number] = mode
    demand[n+1] = 0
    mode_amount[n+1] = 1
    loc_x.append(loc_x[0])
    loc_y.append(loc_y[0])

    V = N + [n+1]

    print("n: ", n)
    print("N: ", N[1:])
    print("V: ", V)
    print("demand: ", demand)
    print(mode_amount)
    print("loc_x: ", loc_x)
    print("loc_y: ", loc_y)
    print("K_p: ", K_p)
    print("K_s: ", K_s)

    return N[1:], demand, mode_amount, loc_x, loc_y, K_p, K_s




