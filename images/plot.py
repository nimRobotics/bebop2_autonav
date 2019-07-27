from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt



fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

x =[0.7112,0.8001,0.7112,0.8001,0.7112,0.8001,0.7112,0.8001,1.3335,1.4351,1.3335,1.4351,1.3335,1.4351,1.3335,1.4351]
y =[1.9304,1.9304,1.8034,1.8034,1.6383,1.6383,1.5113,1.5113,1.9304,1.9304,1.8034,1.8034,1.6383,1.6383,1.5113,1.5113]
z =[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

ax.scatter(x, y, z, c='r', marker='o')
ax.set_zlim([0, 1])
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()
