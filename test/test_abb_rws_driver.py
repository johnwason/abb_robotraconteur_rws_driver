import RobotRaconteur as RR
import time

def _new_rr_client_node():
    n = RR.RobotRaconteurNode()
    n.Init()

    t = RR.TcpTransport(n)
    n.RegisterTransport(t)

    return n

class _client:
    def __init__(self, node, client):
        self.node = node
        self.client = client
        self.rws_const = node.GetConstants("experimental.abb_robot.rws", client)
        self.task_cycle = self.rws_const["TaskCycle"]
        self.task_exec_state = self.rws_const["TaskExecutionState"]

    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        self.node.Shutdown()

def _connect_client():
    n = _new_rr_client_node()

    c = n.ConnectService("rr+tcp://localhost:59926?service=robot")

    return _client(n, c)

def test_rws_metadata():

    with _connect_client() as c_obj:
        c = c_obj.client

        device_info = c.device_info
        assert device_info is not None
        assert device_info.device.name is not None
        robot_info = c.robot_info
        assert robot_info is not None
        assert len(robot_info.chains) > 0
        n_joints = len(robot_info.joint_info)
        assert n_joints >= 6
        assert len(robot_info.chains[0].H) == n_joints
        assert len(robot_info.chains[0].P) == n_joints + 1

def test_rws_controller_state():
    with _connect_client() as c_obj:
        c = c_obj.client

        print(c.getf_execution_state())
        print(c.getf_controller_state())
        print(c.getf_operation_mode())

        _ = c.operational_mode
        _ = c.controller_state

def test_rws_tasks():
    with _connect_client() as c_obj:
        c = c_obj.client
        c.resetpp()
        c.start(c_obj.task_cycle["forever"], ["T_ROB1"])
        time.sleep(0.1)
        assert c.getf_execution_state().ctrlexecstate == c_obj.task_exec_state["running"]
        assert c.getf_execution_state().cycle == c_obj.task_cycle["forever"]
        time.sleep(1)
        c.stop()
        assert c.getf_execution_state().ctrlexecstate == c_obj.task_exec_state["stopped"]

        task_state = c.getf_tasks()

        c.deactivate_task("T_ROB1")
        c.activate_task("T_ROB1")

def test_rws_egm_feedback():
    with _connect_client() as c_obj:
        c = c_obj.client

        robot_info = c.robot_info
        n_joints = len(robot_info.joint_info)

        # Make sure EGM is running
        c.resetpp()
        c.start(c_obj.task_cycle["forever"], ["T_ROB1"])
        time.sleep(1)
        c.stop()

        robot_state, _ = c.robot_state.PeekInValue()
        assert robot_state is not None
        assert len(robot_state.joint_position) == n_joints
        adv_robot_state, _ = c.advanced_robot_state.PeekInValue()
        assert adv_robot_state is not None
        assert len(adv_robot_state.joint_position) == n_joints
        robot_state_sns_ep = c.robot_state_sensor_data.Connect(-1)
        robot_state_sns = robot_state_sns_ep.ReceivePacketWait(1)
        robot_state_sns_ep.Close()
        assert robot_state_sns is not None
        assert robot_state_sns.data_header is not None
        assert len(robot_state_sns.robot_state.joint_position) == n_joints

def test_rws_isoch():
    with _connect_client() as c_obj:
        c = c_obj.client
        isoch_info = c.isoch_info
        print(isoch_info)
        assert isoch_info is not None
        isoch_downsample = c.isoch_downsample
        assert isoch_downsample == 0
        c.isoch_downsample = 1
        assert c.isoch_downsample == 1
        c.isoch_downsample = 0
        assert c.isoch_downsample == 0

        device_now, _ = c.device_clock_now.PeekInValue()
        print(device_now)

def test_rws_signal():
    with _connect_client() as c_obj:
        c = c_obj.client
        c.setf_digital_io("test_digital_io1", 1)
        assert c.getf_digital_io("test_digital_io1") == 1
        c.setf_digital_io("test_digital_io1", 0)
        assert c.getf_digital_io("test_digital_io1") == 0

        c.setf_digital_io2("test_digital_io2", "DeviceNet", "d651", 1)
        assert c.getf_digital_io2("test_digital_io2", "DeviceNet", "d651") == 1
        c.setf_digital_io2("test_digital_io2", "DeviceNet", "d651", 0)
        assert c.getf_digital_io2("test_digital_io2", "DeviceNet", "d651") == 0

        c.setf_analog_io("test_analog_io1", 14.285)
        assert abs(c.getf_analog_io("test_analog_io1") - 14.285) < 0.001
        c.setf_analog_io("test_analog_io1", 0)
        assert abs(c.getf_analog_io("test_analog_io1") - 0) < 0.001

        c.setf_analog_io2("test_analog_io2", "DeviceNet", "d651", 8.562)
        assert abs(c.getf_analog_io2("test_analog_io2", "DeviceNet", "d651") - 8.562) < 0.001
        c.setf_analog_io2("test_analog_io2", "DeviceNet", "d651", 0)
        assert abs(c.getf_analog_io2("test_analog_io2", "DeviceNet", "d651") - 0) < 0.001

def test_rws_rapid_vars():
    with _connect_client() as c_obj:
        c = c_obj.client
        # TODO: getf_rapid_variables() is not implemented
        # rapid_vars = c.getf_rapid_variables("T_ROB1")
        # assert "test_var_str" in rapid_vars
        c.setf_rapid_variable("test_var_str", "T_ROB1", "\"Hello World!\"")
        assert c.getf_rapid_variable("test_var_str", "T_ROB1") == "\"Hello World!\""
        c.setf_rapid_variable("test_var_num", "T_ROB1/Module1", str(123.456))
        assert abs(float(c.getf_rapid_variable("test_var_num", "T_ROB1/Module1")) - 123.456) < 0.001

def test_rws_files():
    with _connect_client() as c_obj:
        c = c_obj.client

        ramdisk_path = c.getf_ramdisk_path()
        data = "Hello World from file!".encode("utf-8")
        c.upload_file(ramdisk_path + "/test_file.txt", data)
        file_list = c.list_files(ramdisk_path)
        assert "test_file.txt" in file_list
        assert c.read_file(ramdisk_path + "/test_file.txt").tobytes() == data
        c.delete_file(ramdisk_path + "/test_file.txt")
        file_list = c.list_files(ramdisk_path)
        assert "test_file.txt" not in file_list
    
def test_evtlog():
    with _connect_client() as c_obj:
        c = c_obj.client

        evts = c.read_event_log()

        print(evts)

def test_current_targets():
    with _connect_client() as c_obj:
        c = c_obj.client

        c.getf_jointtarget("ROB_1")
        c.getf_robtarget("ROB_1")
        c.getf_robtarget2("ROB_1", "tool0", "wobj0", "Base")

def test_speed_ratio():
    with _connect_client() as c_obj:
        c = c_obj.client

        _ = c.speed_ratio
        c.speed_ratio= 0.5
        assert c.speed_ratio == 0.5
        c.speed_ratio = 1.0
        assert c.speed_ratio == 1.0