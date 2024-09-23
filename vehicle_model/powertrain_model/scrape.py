from scipy.interpolate import interp1d
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

data = pd.read_csv("data.csv")
data = data.dropna()

x = data["x"].tolist()
y = data["y"].tolist()

# x = np.log10(x)
# y = np.log10(y)

interpolate = interp1d(x, y)

points = np.linspace(start=min(x), stop=max(x), num=50)
interps = interpolate(points)
plt.plot(points, interps)
plt.show()