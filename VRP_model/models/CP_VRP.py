#! /usr/bin/env python
from ortools.sat.python import cp_model
from CRVRP import generate_input_data
import os


class Filter():
    def __init__(self):
        self.block = []

    def filter(self, seq):
        return [x for x in seq if x not in self.block]


class SpamFilter(Filter):
    def __init__(self):
        super().__init__()
        self.block = ['spam']


K = Filter()
K.filter([1, 2, 3])
print(K.filter([1, 2, 3]))

Y = SpamFilter()
Y.filter([1, 2, 4, 'spam'])
print(Y.filter([1, 2, 4, 'spam']))