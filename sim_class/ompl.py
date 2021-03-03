
class OMPL_arm():
    # using with OMPL_arm.ttm model

    def __init__(self, client, joints_handles):
        self.client = client
        self.joints_handles = joints_handles

        self.max_search_position_times = 10
        self.min_midpoints = 10

    def get_path(self, start_joints_positions, end_joints_positions):
        res, path = self.client.simxCallScriptFunction(
            "get_path@OMPL_arm",
            "sim.scripttype_childscript",
            [
                self.joints_handles,
                start_joints_positions,
                end_joints_positions,
                self.max_search_position_times,
                self.min_midpoints
            ],
            self.client.simxServiceCall()
        )
        return path
