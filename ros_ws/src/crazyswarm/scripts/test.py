import numpy as np

row1 = np.array([[x, 0, 0] for x in np.linspace(0, 2.13, 5)])
col1 = np.array([[2.13, y + 0.51, 0] for y in np.linspace(0, 3.07, 7)[1:]])
row2 = np.array([[x, 3.07, 0] for x in np.linspace(0, 2.13, 5)[::-1]][1:])
col2 = np.array([[0, y + 0.51, 0] for y in np.linspace(0, 3.07, 7)[::-1]][1:-1])

RealWorldCoords = np.concatenate((row1, col1, row2, col2))

print(RealWorldCoords)
print(len(RealWorldCoords))
