import numpy as np


def get_object_float_parameter(client, object_handle, name):
    return client.simxGetObjectFloatParameter(object_handle,
                                              client.simxEvaluateToInt(name, client.simxServiceCall())[1],
                                              client.simxServiceCall())


def get_object_int_parameter(client, object_handle, name):
    return client.simxGetObjectIntParameter(object_handle, client.simxEvaluateToInt(name, client.simxServiceCall())[1],
                                            client.simxServiceCall())


def get_object_string_parameter(client, object_handle, name):
    return client.simxGetObjectStringParameter(object_handle,
                                               client.simxEvaluateToStr(name, client.simxServiceCall())[1],
                                               client.simxServiceCall())


def set_object_float_parameter(client, object_handle, name, value):
    return client.simxSetObjectFloatParameter(object_handle,
                                              client.simxEvaluateToInt(name, client.simxServiceCall())[1], value,
                                              client.simxServiceCall())


def set_object_int_parameter(client, object_handle, name, value):
    return client.simxSetObjectIntParameter(object_handle, client.simxEvaluateToInt(name, client.simxServiceCall())[1],
                                            value, client.simxServiceCall())


def set_object_string_parameter(client, object_handle, name, value):
    return client.simxSetObjectStringParameter(object_handle,
                                               client.simxEvaluateToStr(name, client.simxServiceCall())[1], value,
                                               client.simxServiceCall())


def build_list_str(items):
    ret = ""
    for item in items:
        ret += "{},".format(item)
    ret = "{{{}}}".format(ret[:-1])
    return ret


def screw(w):
    mat = np.zeros([3, 3])
    mat[0, 1] = -w[2]
    mat[0, 2] = w[1]
    mat[1, 2] = -w[0]
    mat += -mat.T
    return mat


def unscrew(mat):
    w = np.zeros(3)
    w[0] = -mat[1, 2]
    w[1] = mat[0, 2]
    w[2] = -mat[0, 1]
    return w


def twist2mat(twist):
    v = twist[:3]
    w = twist[3:]
    mat = np.zeros([4, 4])
    mat[:3, :3] = screw(w)
    mat[:3, 3] = v
    return mat


def mat2twist(mat):
    v = mat[:3, 3]
    w = unscrew(mat[:3, :3])
    return np.concatenate([v, w], axis=0)
