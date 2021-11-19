from os import stat_result
import dxfgrabber
import logging
import bosdyn.client #pour communiquer
from bosdyn.client.image import ImageClient #pour obtenir ses images
from PIL import Image #visualisation d'images
import io #visualisation d'image
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder,blocking_stand #mobilité du robot
from bosdyn.geometry import EulerZXY #métriques angulaires du robot
import math
import sys
from numpy.lib.shape_base import dstack
import requests
import time
from bosdyn.api.basic_command_pb2 import RobotCommandFeedbackStatus
import bosdyn.client
import bosdyn.client.util
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder, blocking_stand
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME, VISION_FRAME_NAME, BODY_FRAME_NAME, get_se2_a_tform_b
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive

def path_from_dxf(dxf_file: str):

    dxf = dxfgrabber.readfile(dxf_file)

    logging.info("Interprétation de %i entités..."%len(dxf.entities))

    lines = dxf.entities.get_entities() #liste des lignes du fichier svg

    path = []

    for line in lines:
        debut = line.start

        fin = line.end
        path.append(([int(debut[1]), int(debut[0])], "off")) 
        path.append(([int(fin[1]), int(fin[0])], "on"))
    
    logging.info("Fait.")

    return path





def follow_path(path, command_client, state_client):
    position = [0.0, 0.0]
    for step in path:
        point = step[0]
        status = step[1]
        deplacement = soustraction(position, point)
        x=deplacement[0]/100
        y=deplacement[1]/100
        input("Confirmer le déplacement (x=" + str(x)+", y="+str(y)+ "), traçage "+ status + " ?")
        #requests.get("http://192.168.80.105/paint-"+ status)
        relative_move(x, y, 0, ODOM_FRAME_NAME, command_client, state_client, False)
        position= somme(position, deplacement)

    deplacement =  soustraction(position, [0.0, 0.0])
    x=deplacement[0]/100
    y=deplacement[1]/100
    input("Confirmer le déplacement (x=" + str(x)+", y="+str(y)+ "), traçage "+ status + " ?")
    relative_move(x, y, 0, ODOM_FRAME_NAME, command_client, state_client, False)


def soustraction(position, deplacement):
    zipped_lists = zip(deplacement, position)
    return [x - y for (x, y) in zipped_lists]

def somme(position, deplacement):
    zipped_lists = zip(position, deplacement)
    return [x + y for (x, y) in zipped_lists]

def relative_move(dx, dy, dyaw, frame_name, robot_command_client, robot_state_client, stairs=False):
    transforms = robot_state_client.get_robot_state().kinematic_state.transforms_snapshot

    # Build the transform for where we want the robot to be relative to where the body currently is.
    body_tform_goal = math_helpers.SE2Pose(x=dx, y=dy, angle=dyaw)
    # We do not want to command this goal in body frame because the body will move, thus shifting
    # our goal. Instead, we transform this offset to get the goal position in the output frame
    # (which will be either odom or vision).
    out_tform_body = get_se2_a_tform_b(transforms, frame_name, BODY_FRAME_NAME)
    out_tform_goal = out_tform_body * body_tform_goal

    # Command the robot to go to the goal point in the specified frame. The command will stop at the
    # new position.
    robot_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
        goal_x=out_tform_goal.x, goal_y=out_tform_goal.y, goal_heading=out_tform_goal.angle,
        frame_name=frame_name, params=RobotCommandBuilder.mobility_params(stair_hint=stairs))
    end_time = 10.0
    cmd_id = robot_command_client.robot_command(lease=None, command=robot_cmd,
                                                end_time_secs=time.time() + end_time)
    # Wait until the robot has reached the goal.
    while True:
        feedback = robot_command_client.robot_command_feedback(cmd_id)
        mobility_feedback = feedback.feedback.synchronized_feedback.mobility_command_feedback
        if mobility_feedback.status != RobotCommandFeedbackStatus.STATUS_PROCESSING:
            print("Failed to reach the goal")
            return False
        traj_feedback = mobility_feedback.se2_trajectory_feedback
        if (traj_feedback.status == traj_feedback.STATUS_AT_GOAL and
                traj_feedback.body_movement_status == traj_feedback.BODY_STATUS_SETTLED):
            print("Arrived at the goal.")
            return True
        time.sleep(1)

if __name__ == "__main__":
    sdk = bosdyn.client.create_standard_sdk("test-snoopy-vision")
    robot = sdk.create_robot('192.168.80.3')
    robot.authenticate('user', '0tvaa515q4d5')
    state_client = robot.ensure_client('robot-state')
    estop_client = robot.ensure_client("estop")
    estop_end_point = bosdyn.client.estop.EstopEndpoint(client=estop_client, name="estop-de-test", estop_timeout=9.0)
    estop_end_point.force_simple_setup()
    estop_keep_alive = bosdyn.client.estop.EstopKeepAlive(estop_end_point)
    lease_client = robot.ensure_client("lease")
    lease = lease_client.acquire()
    lease_keep_alive = bosdyn.client.lease.LeaseKeepAlive(lease_client)
    robot.power_on(timeout_sec=20)
    robot.time_sync.wait_for_sync()
    command_client = robot.ensure_client(RobotCommandClient.default_service_name)
    path = path_from_dxf("dessin.dxf")
    follow_path(path, command_client, state_client)
