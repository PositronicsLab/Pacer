from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from sklearn import mixture

def fit_samples(samples, ncomponents):
  gmix = mixture.GMM(n_components=ncomponents, covariance_type='full')
  gmix.fit(samples)
  return gmix

np.random.seed(0)
samples = np.random.rand(10000, 2)
gmix = fit_samples(samples,100)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
x = y = np.arange(-3.0, 3.0, 0.05)
X, Y = np.meshgrid(x, y)
zs = np.array([gmix.predict([x,y]) for x,y in zip(np.ravel(X), np.ravel(Y))])
Z = zs.reshape(X.shape)

ax.plot_surface(X, Y, Z)

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()
