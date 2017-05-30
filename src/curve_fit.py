import numpy as np
import matplotlib.pyplot as plt

def f(t1,t2,p):
	t = np.arange(t1,t2,0.01)
	x = (1 - t) * (1 - t) * p[0][0] + 2 * (1 - t) * t * p[1][0] + t * t * p[2][0]
	y = (1 - t) * (1 - t) * p[0][1] + 2 * (1 - t) * t * p[1][1] + t * t * p[2][1]
	return x,y

p =np.array([[0,0],[1, 0], [1, 6]])
x,y = f(0,1,p) 
plt.plot(x, y, lw=2)
plt.show()
