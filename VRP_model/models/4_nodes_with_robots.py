from ortools.sat.python import  cp_model
import numpy as np


def create_decision_variable(m, k, n):
    var = np.empty([k, n, n], dtype=object)
    for k in list(range(k)):
        for i in list(range(n)):
            for j in list(range(n)):
                var[k, i, j] = m.NewBoolVar('vehicle {} travel from {} to {}'.format(k, i, j))
    return var


model = cp_model.CpModel()
x_var = create_decision_variable(model, 1, 4)
print(x_var)
