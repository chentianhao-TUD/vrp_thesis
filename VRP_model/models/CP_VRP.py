#! /usr/bin/env python
from ortools.sat.python import cp_model
from CRVRP import generate_input_data
import os

storage = {}


def init(data):
    data['first'] = {}
    data['middle'] = {}
    data['last'] = {}


def lookup(data, label, name):
    return data[label].get(name)


def store(data, full_name):
    names = full_name.split()
    labels = ['first', 'middle', 'last']
    for label, name in zip(labels, names):
        data[label] = name


store(storage, 'CHEN TIAN HAO')
print(storage)