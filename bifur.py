import numpy as np
import matplotlib.pyplot as plt
plt.style.use('dark_background')
def logistic(r, x):
    return r*x*(1-x)

def plot():
    n = 10_0000
    r = np.linspace(2.5, 4.0, n)
    r_total = []
    iterations = 10000
    last = 100
    x = 1e-5 * np.ones(n)
    x_total = []
    lyapunov = np.zeros(n)
    for i in range(iterations):
        x = logistic(r, x)
        # We compute the partial sum of the
        # Lyapunov exponent.
        lyapunov += np.log(abs(r - 2 * r * x))
        # We display the bifurcation diagram.
        if i >= (iterations - last):
            r_total.append(np.array(r))
            x_total.append(np.array(x))
    return np.array(r_total), np.array(x_total)

r, x = plot()
sortis = np.argsort(r.reshape(-1))
plt.plot(r.reshape(-1)[sortis], x.reshape(-1)[sortis], ',', alpha=.25)
plt.show(block=True)

