import numpy as np
from scipy.spatial.transform import Rotation as R

deg90 = np.pi / 2

def getTransformation(transformation: str, axis: str, value):
    if transformation == "lin":
        # Linear transformation
        if axis == "x":
            T = np.array([[1, 0, 0, value],
                          [0, 1, 0, 0],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])
        elif axis == "y":
            T = np.array([[1, 0, 0, 0],
                          [0, 1, 0, value],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])
        elif axis == "z":
            T = np.array([[1, 0, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 1, value],
                          [0, 0, 0, 1]])
        else:
            raise ValueError("Invalid axis for linear transformation")
    elif transformation == "ang":
        # Angular transformation
        if axis == "x":
            r = R.from_rotvec(value * np.array([1, 0, 0]))
        elif axis == "y":
            r = R.from_rotvec(value * np.array([0, 1, 0]))
        elif axis == "z":
            r = R.from_rotvec(value * np.array([0, 0, 1]))
        else:
            raise ValueError("Invalid axis for angular transformation")
        T = np.round(np.block([[r.as_matrix(),np.zeros((3,1))],
                      [np.zeros((1,3)),np.array([1])]]).reshape(4,4),6)
    else:
        raise ValueError("Invalid transformation type")
    
    return T

def getTotalTransformation(transformation_list, axis_list, value):
    T= np.eye(4)
    for i in range(len(transformation_list)):
        T=T@getTransformation(transformation_list[i],axis_list[i],value[i])
    return T