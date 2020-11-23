import matplotlib.pyplot as plt 
import numpy as np 



f_real=open("/home/iris/catkin_ws/src/myrobot/scripts/psi_real.txt", "r")
f_des=open("/home/iris/catkin_ws/src/myrobot/scripts/psi_des.txt", "r")
psi_real = f_real.read()
psi_des = f_des.read()



X, Y = [], []
for line in open('/home/iris/catkin_ws/src/myrobot/scripts/psi_real.txt', 'r'):
  values = [float(s) for s in line.split()]
  X.append(values[0])
  

points = [0.0 for i in range(len(X))]
for line in open('/home/iris/catkin_ws/src/myrobot/scripts/psi_des.txt', 'r'):
  values = [float(s) for s in line.split()]
  Y.append(values[0])

plt.plot(X)
plt.plot(Y)
plt.show()





# a = [1,2,3,4]
# plt.plot(a[0:2])
# plt.show()


# x = np.linspace(-10 , 10, 100)
# y = np.sin(x) 
# plt.plot(x, y, marker="x")
# plt.show()