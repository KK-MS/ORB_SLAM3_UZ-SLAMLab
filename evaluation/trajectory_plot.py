import sys
import csv
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
import tkinter as tk
matplotlib.use('Q')
from mpl_toolkits.mplot3d import axes3d

df = pd.read_csv(r'../kf_testoutput.txt', sep=' ', encoding='cp1252')

df.head()

X = df.x
Y = df.y
Z = df.z

# Plot X,Y,Z

# for creating a responsive plot

# creating figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot_trisurf(X, Y, Z, color='white', edgecolors='grey', alpha=0.5)
#ax = Axes3D(fig)

# creating the plot
ax.scatter(X, Y, Z, c='red')

# setting title and labels
ax.set_title("3D plot")
ax.set_xlabel('x-axis')
ax.set_ylabel('y-axis')
ax.set_zlabel('z-axis')
 
# displaying the plot
plt.show()