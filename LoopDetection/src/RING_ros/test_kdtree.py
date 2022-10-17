import numpy as np
from sklearn.neighbors import KDTree
import time

times = time.time()
rng = np.random.RandomState(0)
X = rng.random_sample((512, 1024))  # 10 points in 3 dimensions
tree = KDTree(X)  
times = time.time()
ind = tree.query_radius(X[:1], r=0.3)  
timee = time.time()

print("kdtree time: ", timee-times,'s')