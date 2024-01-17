import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import sys


file1 = sys.argv[1]
file2 = sys.argv[2]

# CSV-Datei lesen
df1 = pd.read_csv(file1)
gp_acc_x = df1['Accelerometer [m/s2]'].values
gp_acc_y = df1['1'].values
gp_acc_z = df1['2'].values

print(len(gp_acc_x))

y_axis_gp = []
count = 1
for i in range(len(gp_acc_x)):
    y_axis_gp.append(count)
    count = count+1


#acc_x,acc_y,acc_z
df2 = pd.read_csv(file2)
adma_acc_x = df2['acc_x'].values
adma_acc_y = df2['acc_y'].values
adma_acc_z = df2['acc_z'].values

y_axis_adma = []
count = 1
for i in range(len(adma_acc_x)):
    y_axis_adma.append(count)
    count = count+1

print(gp_acc_x)


# Erstellung des Diagramms mit 3 Subplots, jeder mit 2 Linienkurven
fig, axs = plt.subplots(3, 1, figsize=(8, 12))

# Subplot 1: Zwei Linienkurven (Sinus und Kosinus)
axs[0].plot(y_axis_gp,gp_acc_y, color='blue', label='gp_acc_y')
axs[0].legend()

# Subplot 2: Zwei Linienkurven (Zufällige Werte)
axs[1].plot(y_axis_gp, gp_acc_x, color='red', label='gp_acc_x')
axs[1].legend()

# Subplot 3: Zwei Linienkurven (Sinus * Kosinus und Kosinus)
axs[2].plot(y_axis_gp, gp_acc_z, color='orange', label='gp_acc_z')
axs[2].legend()


# Erstellung des Diagramms mit 3 Subplots, jeder mit 2 Linienkurven
fig2, axs2 = plt.subplots(3, 1, figsize=(8, 12))

axs2[0].plot( y_axis_adma,adma_acc_z, color='green', label='adma_acc_z')
axs2[0].legend()

axs2[1].plot(y_axis_adma, adma_acc_y, color='purple', label='adma_acc_y')
axs2[1].legend()

axs2[2].plot(y_axis_adma, adma_acc_x, color='black', label='adma_acc_x')
axs2[2].legend()

# Anpassung des Layouts
plt.tight_layout()
plt.show()
