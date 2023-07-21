import RobotRaconteur as RR
RRN = RR.RobotRaconteurNode.s
import RobotRaconteurCompanion as RRC
from abb_robot_client import egm, rws
import traceback
import threading
import time
import numpy as np
from robotraconteur_abstract_robot import AbstractRobot
from contextlib import suppress
import argparse
from RobotRaconteurCompanion.Util.RobDef import register_service_types_from_resources
from RobotRaconteurCompanion.Util.AttributesUtil import AttributesUtil
from RobotRaconteurCompanion.Util.InfoFileLoader import InfoFileLoader
import drekar_launch_process

class ABBRWSRobotImpl(AbstractRobot):
    def __init__(self, robot_info, robot_url):
        super().__init__(robot_info, 6)

        self._robot_url = robot_url

        self._base_set_controller_state = False
        self._base_set_operational_mode = False
        self.robot_info.robot_capabilities = 0

        self._uses_homing = False
        self._has_position_command = False
        self._has_velocity_command = False
        self._has_jog_command = False

        self._rws = None
        self._egm = None
        self._missed_egm = 0

    def _start_robot(self):
        self._egm = egm.EGM()
        self._rws = rws.RWS(self._robot_url)

        super()._start_robot()

    def _stop_robot(self):
        super()._stop_robot()

    def _close(self):
        super()._close()

    def _send_robot_command(self, now, joint_pos_cmd, joint_vel_cmd):
        pass

    def _verify_robot_state(self, now):
        self._command_mode = self._robot_command_mode["halt"]
        return True
    
    def _fill_robot_command(self, now):
        return False, None, None
    
    def async_disable(self, handler):
        raise RR.InvalidOperationException("Invalid command for ABB RWS driver")
    
    async def _send_disable(self, handler):
        raise NotImplemented()

    def async_enable(self, handler):
        raise RR.InvalidOperationException("Invalid command for ABB RWS driver")

    async def _send_enable(self, handler):
        raise NotImplemented()

    def async_reset_errors(self, handler):
        raise RR.InvalidOperationException("Invalid command for ABB RWS driver")
    
    def _send_reset_errors(self, handler):
        raise NotImplemented()
    
    def async_jog_freespace(self, joint_position, max_velocity, wait, handler):
        raise RR.InvalidOperationException("Invalid command for ABB RWS driver")
    
    def async_jog_joint(self, joint_velocity, timeout, wait, handler):
        raise RR.InvalidOperationException("Invalid command for ABB RWS driver")
    
    def execute_trajectory(self, trajectory):
        raise RR.InvalidOperationException("Invalid command for ABB RWS driver")
    
    def jog_cartesian(self, velocity, timeout, wait):
        raise RR.InvalidOperationException("Invalid command for ABB RWS driver")
    
    def async_home(self, handler):
        raise RR.InvalidOperationException("Invalid command for ABB RWS driver")
    
    def tool_attached(self, chain, tool):
        raise RR.InvalidOperationException("Invalid command for ABB RWS driver")

    def tool_detached(self, chain, tool_name):
        raise RR.InvalidOperationException("Invalid command for ABB RWS driver")

    def payload_attached(self, chain, payload, pose):
        raise RR.InvalidOperationException("Invalid command for ABB RWS driver")

    def payload_detached(self, chain, payload_name):
        raise RR.InvalidOperationException("Invalid command for ABB RWS driver")
    
    @AbstractRobot.command_mode.setter
    def command_mode(self, value):
        raise RR.InvalidOperationException("Invalid command for ABB RWS driver")
    
    def _run_timestep(self, now):
        res = True
        robot_state = None
        while res:
            res, robot_state1 = self._egm.receive_from_robot()
            if res:                
                robot_state = robot_state1
                self._missed_egm = 0
        if robot_state is None:
            self._missed_egm += 1

        if robot_state is not None:
            egm_last_recv = self._stopwatch_ellapsed_s()
            self._last_joint_state = egm_last_recv
            self._last_endpoint_state = egm_last_recv
            self._last_robot_state = egm_last_recv
            self._enabled = robot_state.motors_on
            self._ready = self._enabled #robot_state.rapid_running
        
            self._joint_position = np.deg2rad(robot_state.joint_angles)            
            self._endpoint_pose = self._node.ArrayToNamedArray(\
                np.concatenate((robot_state.cartesian[1],robot_state.cartesian[0]*1e-3)), self._pose_dtype)
            
        else:
            if self._communication_failure:
                self._joint_position = np.zeros((0,))        
                self._endpoint_pose = np.zeros((0,),dtype=self._pose_dtype)

        if self._error:
            self._ready = False

        super()._run_timestep(now)

def main():

    parser = argparse.ArgumentParser(description="ABB RWS/EGM robot driver service for Robot Raconteur")
    parser.add_argument("--robot-info-file", type=argparse.FileType('r'),default=None,required=True,help="Robot info file (required)")
    parser.add_argument("--robot-url", type=str,default="http://127.0.0.1:80",help="Robot Web Services URL")
    parser.add_argument("--robot-name", type=str,default=None,help="Optional device name override")
    
    args, _ = parser.parse_known_args()

    RRC.RegisterStdRobDefServiceTypes(RRN)
    register_service_types_from_resources(RRN, __package__, ["experimental.abb_robot.rws"])

    with args.robot_info_file:
        robot_info_text = args.robot_info_file.read()

    info_loader = InfoFileLoader(RRN)
    robot_info, robot_ident_fd = info_loader.LoadInfoFileFromString(robot_info_text, "com.robotraconteur.robotics.robot.RobotInfo", "device")

    attributes_util = AttributesUtil(RRN)
    robot_attributes = attributes_util.GetDefaultServiceAttributesFromDeviceInfo(robot_info.device_info)

    robot = ABBRWSRobotImpl(robot_info, args.robot_url)
    try:

        robot._start_robot()
        time.sleep(0.5)
        with RR.ServerNodeSetup("experimental.abb_rws_robot.robot",59926):

            service_ctx = RRN.RegisterService("robot","experimental.abb_robot.rws.ABBRWSRobot",robot)
            service_ctx.SetServiceAttributes(robot_attributes)

            print("Press Ctrl-C to exit")
            drekar_launch_process.wait_exit()
            robot._close()
    except:
        with suppress(Exception):
            robot._close()
        raise

    