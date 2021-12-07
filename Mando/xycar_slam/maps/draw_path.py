import pickle
import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    with open("/home/amap/aMAP_catkin_ws/src/xycar_slam/maps/mando_11_27_1.pkl", "rb") as f:
        path = pickle.load(f)

    print(len(path['x']))
    steps = 1
    plt.plot(path['x'][::steps], path['y'][::steps], '.')
    plt.axis("equal")
    plt.show()
    # print(path['yaw'])
    plt.plot(path['yaw'][::steps], '.-')

    plt.axis("equal")
    plt.show()
