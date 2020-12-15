

def get_object_float_parameter(client, object_handle, name):
    return client.simxGetObjectFloatParameter(object_handle, client.simxEvaluateToInt(name, client.simxServiceCall())[1], client.simxServiceCall())


def get_object_int_parameter(client, object_handle, name):
    return client.simxGetObjectIntParameter(object_handle, client.simxEvaluateToInt(name, client.simxServiceCall())[1], client.simxServiceCall())


def get_object_string_parameter(client, object_handle, name):
    return client.simxGetObjectStringParameter(object_handle, client.simxEvaluateToStr(name, client.simxServiceCall())[1], client.simxServiceCall())



def set_object_float_parameter(client, object_handle, name, value):
    return client.simxSetObjectFloatParameter(object_handle, client.simxEvaluateToInt(name, client.simxServiceCall())[1], value, client.simxServiceCall())


def set_object_int_parameter(client, object_handle, name, value):
    return client.simxSetObjectIntParameter(object_handle, client.simxEvaluateToInt(name, client.simxServiceCall())[1], value, client.simxServiceCall())


def set_object_string_parameter(client, object_handle, name, value):
    return client.simxSetObjectStringParameter(object_handle, client.simxEvaluateToStr(name, client.simxServiceCall())[1], value, client.simxServiceCall())