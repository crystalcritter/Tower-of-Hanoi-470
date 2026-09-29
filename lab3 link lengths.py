import numpy as np

offset = np.array([-0.150, 0.150, 0.010])

link1 = offset + np.array([0, 0, 0.152])
link2 = link1 + np.array([0, 0.120, 0])
link3 = link2 + np.array([0.244, 0, 0])
link4 = link3 + np.array([0.213, -0.093, 0])
link5 = link4 + np.array([0, 0.104, 0])
link6 = link5 + np.array([0.085, 0, 0])

for i in range(1, 7):
    print(f"Link {i} position: {eval(f'link{i}')}")
