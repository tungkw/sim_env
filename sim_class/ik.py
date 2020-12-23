_, ik_x_constraint = client.simxEvaluateToInt("sim.ik_x_constraint", self.client.simxServiceCall())
_, ik_y_constraint = client.simxEvaluateToInt("sim.ik_y_constraint", self.client.simxServiceCall())
_, ik_z_constraint = client.simxEvaluateToInt("sim.ik_z_constraint", self.client.simxServiceCall())
_, ik_alpha_beta_constraint = client.simxEvaluateToInt("sim.ik_alpha_beta_constraint",
                                                       self.client.simxServiceCall())
_, ik_gamma_constraint = client.simxEvaluateToInt("sim.ik_gamma_constraint", self.client.simxServiceCall())
_, ik_avoidance_constraint = client.simxEvaluateToInt("sim.ik_avoidance_constraint",
                                                      self.client.simxServiceCall())
print(ik_x_constraint)
print(ik_y_constraint)
print(ik_z_constraint)
print(ik_alpha_beta_constraint)
print(ik_gamma_constraint)
print(ik_avoidance_constraint)
res, self.ik_group_handle = client.simxExecuteScriptString(
    "sim.createIkGroup({options}, nil, nil)".format(options=0),
    self.client.simxServiceCall())
print(res, self.ik_group_handle)
if res == 0 or self.ik_group_handle == -1:
    print("get UR5 ik group failed")
_, self.tip_handle = client.simxGetObjectHandle("UR5_link7_visible", self.client.simxServiceCall())
self.tip_handle = client.simxCreateDummy()
self.base_handle = self.handle  # client.simxGetObjectHandle()
self.alt_base_handle = -1  # client.simxGetObjectHandle()
# self.target_handle = client.simxGetObjectHandle()
self.constraints = ik_x_constraint  # +ik_y_constraint+ik_z_constraint+ik_alpha_beta_constraint+ik_alpha_beta_constraint

print("sim.createIkElement({ik_group_handle}, {options}, {int_params}, {float_params})".format(
    ik_group_handle=self.ik_group_handle,
    options=0,
    int_params='{%d,%d,%d,%d}' % (self.tip_handle, self.base_handle, self.alt_base_handle, self.constraints),
    float_params="nil",
))
res, self.ik_element_handle = client.simxExecuteScriptString(
    "simCreateIkElement({ik_group_handle}, {options}, {int_params}, nil, nil)".format(
        ik_group_handle=self.ik_group_handle,
        options=0,
        int_params='{%d,%d,%d,%d}' % (self.tip_handle, self.base_handle, self.alt_base_handle, self.constraints),
        # float_params="nil",
    ),
    self.client.simxServiceCall())
# res, self.ik_element_handle = client.simxExecuteScriptString(
#     "sim.createIkElement()",
#     self.client.simxServiceCall())
print(res, self.ik_element_handle)
print(self.ik_element_handle)