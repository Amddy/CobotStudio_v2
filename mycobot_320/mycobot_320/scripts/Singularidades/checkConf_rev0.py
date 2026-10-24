from Cobot_sdk import *

def run(robot: BaseRobotController, **kwargs):
    dar = RobTarget(SE3(340, -135.721, 200)* SE3.Ry(np.pi/2)*SE3.Rz(np.pi), [-1, 1, 1])
    pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)


    def moveit(robot:ROSManager):
        # robot.testPose(dar, pinza, SE3())
        robot.explore_ik_configs(dar, pinza)

    def revisar_configs(robot:ROSManager):
        q1 = [-1.2755958665876677, -1.3289575266776272, 0.44925002025312466, 0.3557442771457664, 0.08017588467153125, -0.120232919661861]
        q2 = [1.2795043909078911, -0.39132916992762823, -0.31007227665906445, -1.0427024000616862, 2.728832562096298, 0.02386265543772037]
        q3 = [-1.0048658135079798, 1.1620244530099635, 0.25907754825867757, 1.201175692157742, -1.1466350049522143, -0.14946468140611024]
        robot.view_pose(q3)
        robot.GoHome()

    revisar_configs(robot)