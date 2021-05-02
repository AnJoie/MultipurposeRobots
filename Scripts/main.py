import numpy as np

from scipy.integrate import odeint
from scipy.optimize import minimize

from dataclasses import dataclass

mLs = [3.0, 2.9, 3.1, 3.2, 2.8, 1.5, 2.5]
mRs = [3.0, 3.1, 2.9, 2.8, 3.0, 3.0, 2.0]


@dataclass
class Velocities:
    mLmR3: np.array
    mL29mR31: np.array
    mL31mr29: np.array
    mL32mr28: np.array
    mL28mr30: np.array
    mL15mr30: np.array
    mL25mr20: np.array


angular_files = [f"data/angular{x}.txt" for x in ['M3', 'M29', 'M31', 'M32', 'M28', 'M15', 'M25']]
linear_files = [f"data/speed{x}.txt" for x in ['M3', 'M29', 'M31', 'M32', 'M28', 'M15', 'M25']]


def load_data(paths):
    nparrays = []

    assert (len(paths) == 7)

    for i, path in enumerate(paths):
        with open(path) as f:
            velocities = []
            for line_number, line in enumerate(f.readlines()):
                velocities.append(float(line))
            nparrays.append(np.array(velocities))

    return Velocities(nparrays[0], nparrays[1], nparrays[2], nparrays[3],nparrays[4],nparrays[5],nparrays[6])


def linear_system(vs, t, kv, a):
    dv = []
    for i in range(7):
        dv.append(kv * vs[i] + a * (mLs[i] + mRs[i]))
    return np.array(dv)

def angular_system(ws, t, kw, b):
    dw = []
    for i in range(7):
        dw.append(kw * ws[i] + b * (mLs[i] - mRs[i]))
    return np.array(dw)


# 4 dimension each
def cost_MSE(expected, actual):
    #     accumulate abs^2
    assert (len(expected) == len(actual) and len(expected[0]) == len(actual[0]))

    acc = 0.0
    for i, elem in enumerate(expected):
        for j in range(len(elem)):
            acc += (abs(elem[j] - actual[i][j]) ** 2) / (len(expected[0]) * len(expected))
    return acc





if __name__ == '__main__':
    av = load_data(angular_files)
    lv = load_data(linear_files)

    TT = np.arange(1.0, 21.0, 1.0)
    print(av.mL15mr30)

    def linear_cost(a_kv):
        #     turn 20 x 7 array to 7 * 20
        actual = np.transpose(odeint(linear_system, [lv.mLmR3[0], lv.mL29mR31[0], lv.mL31mr29[0], lv.mL32mr28[0], lv.mL28mr30[0], lv.mL15mr30[0], lv.mL25mr20[0]], TT,
                                     (a_kv[0], a_kv[1])))
        expected = np.array([lv.mLmR3, lv.mL29mR31, lv.mL31mr29, lv.mL32mr28, lv.mL28mr30, lv.mL15mr30, lv.mL25mr20])
        return cost_MSE(expected, actual)


    def angular_cost(b_kw):
        actual = np.transpose(odeint(angular_system, [av.mLmR3[0], av.mL29mR31[0], av.mL31mr29[0], av.mL32mr28[0], av.mL28mr30[0], av.mL15mr30[0], av.mL25mr20[0]], TT,
                                     (b_kw[0], b_kw[1])))
        expected = np.array([av.mLmR3, av.mL29mR31, av.mL31mr29, av.mL32mr28,av.mL28mr30, av.mL15mr30, av.mL25mr20])
        return cost_MSE(expected, actual)



    result = minimize(angular_cost, np.array([0.5, 0.5]))
    print("kw = {} and b = {}".format(result.x[0],result.x[1]))


    result = minimize(linear_cost, np.array([0.5, 0.5]))
    print("kv = {} and a = {}".format(result.x[0], result.x[1]))
