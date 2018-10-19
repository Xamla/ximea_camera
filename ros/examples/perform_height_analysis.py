from __future__ import print_function

import rospy
import actionlib
import ximea_msgs.msg
import ximea_msgs.srv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

bridge = CvBridge()

def heightAnalysisClient():
    # initialization
    # needs to be performed only once
    rospy.wait_for_service('/ximea_mono/slstudio/setupSlstudio')
    try:
        setup_slstudio = rospy.ServiceProxy('/ximea_mono/slstudio/setupSlstudio', ximea_msgs.srv.SlstudioSetup)
        resp = setup_slstudio('/home/xamla/Rosvita.Control/projects/laundrometer/calibration/current/StereoCalibration.xml', 50, ['CAMAU1639042', 'CAMAU1710001'], [])
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
        return

    # call height analysis action
    client = actionlib.SimpleActionClient('/ximea_mono/slstudio/heightAnalysis', ximea_msgs.msg.HeightAnalysisAction)
    client.wait_for_server()

    goal = ximea_msgs.msg.HeightAnalysisGoal(shutter_speed_in_ms=50)

    print("send goal")
    print(goal)
    client.send_goal(goal)
    print("wait")
    client.wait_for_result()
    print("done")

    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('heightAnalysisClient')
        result = heightAnalysisClient()

	if result.success == False:
            exit(1)

        try:
            colorMap = bridge.imgmsg_to_cv2(result.response.color_map, "bgr8")
            imageOn = bridge.imgmsg_to_cv2(result.response.image_on, "bgr8")
        except CvBridgeError, e:
            print(e)
        else:
            cv2.imwrite('/tmp/color_map.png', colorMap)
            cv2.imwrite('/tmp/imageOn.png', imageOn)

        print("Done.")
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
