import time
import numpy as np
start = time.time()
list = np.array([])
for i in range(100000000):
    list=np.append(list, i)


print("time :", time.time() - start)
