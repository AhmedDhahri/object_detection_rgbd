import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import scipy.stats as stats
import math

df=pd.read_csv('data.csv', sep=',',header=None)
n = df[0].to_numpy()

plt.plot(n)
plt.ylabel('some numbers')
plt.show()

mu = n.mean()
print mu
variance = n.std()
sigma = math.sqrt(variance)
x = np.linspace(mu - 3*sigma, mu + 3*sigma, 100)
plt.plot(x, stats.norm.pdf(x, mu, sigma))
plt.show()
