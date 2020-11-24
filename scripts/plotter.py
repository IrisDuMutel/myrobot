import matplotlib.pyplot as plt 
import numpy as np 



f_real=open("/home/iris/catkin_ws/src/myrobot/scripts/psi_real.txt", "r")
f_des=open("/home/iris/catkin_ws/src/myrobot/scripts/psi_des.txt", "r")
sim_time = open("/home/iris/catkin_ws/src/myrobot/scripts/sim_time.txt", "r")
psi_real = f_real.read()
psi_des = f_des.read()



X, Y, Z = [], [], []
for line in open('/home/iris/catkin_ws/src/myrobot/scripts/psi_real.txt', 'r'):
  values = [float(s) for s in line.split()]
  X.append(values[0])
  

# points = [0.0 for i in range(len(X))]
for line in open('/home/iris/catkin_ws/src/myrobot/scripts/psi_des.txt', 'r'):
  values = [float(s) for s in line.split()]
  Y.append(values[0])

for line in open('/home/iris/catkin_ws/src/myrobot/scripts/sim_time.txt', 'r'):
  values = [float(s) for s in line.split()]
  Z.append(values[0])

length=min(len(X),len(Y),len(Z))

plt.plot(Z[1:length],X[1:length])#
plt.plot(Z[1:length],Y[1:length])#
plt.grid()
plt.xlabel('Time [s]')
plt.ylabel('Heading [º]')
# plt.axes.Axes.set_ylabel("Heading [º]")
# plt.axes.set_xlabel("Simulation time [s]")
plt.show()





# a = [1,2,3,4]
# plt.plot(a[0:2])
# plt.show()


# x = np.linspace(-10 , 10, 100)
# y = np.sin(x) 
# plt.plot(x, y, marker="x")
# plt.show()