import numpy as np

x = np.array([[1,1,1],[1,1,1],[1,1,1]])
y = np.array([[4,4,4],[5,5,5],[6,6,6]])


m = np.multiply(x,y)
d = np.dot(x,y)

print(f"mult: {m}")
print(f"dot: {d}")