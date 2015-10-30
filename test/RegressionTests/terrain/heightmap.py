from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from sklearn import mixture
import time

def fit_samples(samples, ncomponents):
  gmix = mixture.GMM(n_components=ncomponents, covariance_type='full')
  gmix.fit(samples)
  return gmix

print time.time()
np.random.seed(int(time.time()))
samples = np.random.rand(1000, 2)
gmix = fit_samples(samples,100)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
x = y = np.arange(0, 1.0, 0.01)
X, Y = np.meshgrid(x, y)
z = gmix.predict(zip(np.ravel(X), np.ravel(Y)))
# place on ground
z = z - min(z)
# normalize
z = z / max(z)
# scale
z = z * 0.05

Z = z.reshape(X.shape)

zsize = Z.shape
zsize_txt = ''.join((str(zsize[0]),' ',str(zsize[1])))
print zsize_txt
np.savetxt('heightmap.dat',Z,delimiter=' ',header=zsize_txt)

ax.plot_surface(X, Y, Z)

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()

