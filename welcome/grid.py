import matplotlib.pyplot as plt
ox = []
oy = []
for i in range(-25, 26):
    ox.append(i)
    oy.append(9.0)
for i in range(-25, -8):
    ox.append(i)
    oy.append(4.0)
for i in range(-1, 5):
    ox.append(-8.0)
    oy.append(i)
for i in range(-1, 5):
    ox.append(8.0)
    oy.append(i)
for i in range(8, 25):
    ox.append(i)
    oy.append(4.0)
for i in range(-8, 8):
    ox.append(i)
    oy.append(-1.0)
for i in range(4, 9):
    ox.append(-25)
    oy.append(i)
for i in range(4, 9):
    ox.append(25)
    oy.append(i)

plt.grid(True)
plt.axis("equal")
plt.plot(ox,oy,'.k')
plt.show()
