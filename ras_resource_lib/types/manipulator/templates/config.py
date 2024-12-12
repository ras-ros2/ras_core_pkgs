
def template_func(asset_name):
    from ras_resource_lib.types.manipulator.config import ManipulatorCfg
    from trajectory_msgs.msg import JointTrajectory
    class MyManipulatorCfg(ManipulatorCfg):
        def __init__(self):
            #things to do before initialization and assignment
            super().__init__(
                label=asset_name,
                movegroup_name="manipulator",
                moveit_config=gen_moveit_config(),
                launch_actions=[],
            )
            
        def execute_trajectory(self,trajectory: JointTrajectory):
            #do something with the trajectory
            pass


    a = MyManipulatorCfg()

    def generate_configuration():
        return MyManipulatorCfg()