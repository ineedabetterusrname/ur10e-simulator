/**
 * Python source for the simulator's mock ur_rtde modules. Students' scripts
 * import rtde_control / rtde_receive exactly as they would for the real
 * robot; every call is routed to the JS bridge (registered as `_urbridge`),
 * which records motions for playback on the simulated UR10e.
 *
 * Only primitive numbers cross the Python<->JS boundary (spread joint/pose
 * values), so no PyProxy lifetime management is needed.
 */

export const RTDE_CONTROL_PY = `
"""Simulator stand-in for ur_rtde's rtde_control module."""
import _urbridge as _b

def _six(v, what="value"):
    v = [float(x) for x in v]
    if len(v) < 6:
        raise ValueError("expected 6 %ss, got %d" % (what, len(v)))
    return v[:6]

class RTDEControlInterface:
    """Records motion commands; the web simulator plays them back with real
    motion physics, joint limits and collision protection."""

    def __init__(self, ip="", *args, **kwargs):
        self._ip = str(ip)
        _b.note("RTDEControlInterface connected to simulated UR10e (ip '%s' ignored)" % self._ip)

    # ---- motion -------------------------------------------------------
    def moveJ(self, q, speed=1.05, acceleration=1.4, asynchronous=False):
        if q and isinstance(q[0], (list, tuple)):
            for wp in q:
                wp = [float(x) for x in wp]
                _b.move_j(wp[0], wp[1], wp[2], wp[3], wp[4], wp[5],
                          wp[6] if len(wp) > 6 else float(speed))
            return True
        q = _six(q, "joint value")
        _b.move_j(q[0], q[1], q[2], q[3], q[4], q[5], float(speed))
        return True

    def moveJ_IK(self, pose, speed=1.05, acceleration=1.4, asynchronous=False):
        p = _six(pose, "pose component")
        q = list(_b.ik(p[0], p[1], p[2], p[3], p[4], p[5]))
        _b.move_j(q[0], q[1], q[2], q[3], q[4], q[5], float(speed))
        return True

    def moveL(self, pose, speed=0.25, acceleration=1.2, asynchronous=False):
        p = _six(pose, "pose component")
        _b.move_l(p[0], p[1], p[2], p[3], p[4], p[5], float(speed))
        return True

    def moveL_FK(self, q, speed=0.25, acceleration=1.2, asynchronous=False):
        q = _six(q, "joint value")
        p = list(_b.fk(q[0], q[1], q[2], q[3], q[4], q[5]))
        _b.move_l(p[0], p[1], p[2], p[3], p[4], p[5], float(speed))
        return True

    def servoJ(self, q, speed=0, acceleration=0, time=0.008,
               lookahead_time=0.1, gain=300):
        q = _six(q, "joint value")
        _b.servo_j(q[0], q[1], q[2], q[3], q[4], q[5])
        return True

    def speedL(self, xd, acceleration=0.25, time=0.05):
        xd = _six(xd, "velocity component")
        _b.speed_l(xd[0], xd[1], xd[2], xd[3], xd[4], xd[5], float(time or 0.05))
        return True

    def speedJ(self, qd, acceleration=1.4, time=0.05):
        qd = _six(qd, "joint velocity")
        _b.speed_j(qd[0], qd[1], qd[2], qd[3], qd[4], qd[5], float(time or 0.05))
        return True

    # ---- kinematics ---------------------------------------------------
    def getInverseKinematics(self, pose, qnear=None, *a, **k):
        p = _six(pose, "pose component")
        return list(_b.ik(p[0], p[1], p[2], p[3], p[4], p[5]))

    def getForwardKinematics(self, q=None, tcp_offset=None):
        if q is None:
            return list(_b.get_tcp())
        q = _six(q, "joint value")
        return list(_b.fk(q[0], q[1], q[2], q[3], q[4], q[5]))

    # ---- configuration / no-ops --------------------------------------
    def setTcp(self, offset): return True
    def getTCPOffset(self): return [0.0] * 6
    def setPayload(self, mass, cog=None): return True
    def speedStop(self, a=10.0):
        _b.speed_stop()
        return True
    def servoStop(self, a=10.0): return True
    def stopJ(self, a=2.0, asynchronous=False): return True
    def stopL(self, a=10.0, asynchronous=False): return True
    def stopScript(self): return True
    def triggerProtectiveStop(self): return True
    def isConnected(self): return True
    def isProgramRunning(self): return False
    def isSteady(self): return True
    def reconnect(self): return True
    def disconnect(self):
        _b.note("RTDEControlInterface disconnected")
        return True
    def kickWatchdog(self): return True
    def initPeriod(self): return 0.0
    def waitPeriod(self, t): return None

RTDEControl = RTDEControlInterface
`;

export const RTDE_RECEIVE_PY = `
"""Simulator stand-in for ur_rtde's rtde_receive module."""
import _urbridge as _b

class RTDEReceiveInterface:
    """State reads return the pose the simulated robot will have after the
    motions queued so far (kinematically propagated)."""

    def __init__(self, ip="", *args, **kwargs):
        pass

    def getActualQ(self): return list(_b.get_q())
    def getTargetQ(self): return list(_b.get_q())
    def getActualTCPPose(self): return list(_b.get_tcp())
    def getTargetTCPPose(self): return list(_b.get_tcp())
    def getActualQd(self): return [0.0] * 6
    def getTargetQd(self): return [0.0] * 6
    def getActualTCPSpeed(self): return [0.0] * 6
    def getActualTCPForce(self): return [0.0] * 6
    def getTimestamp(self): return float(_b.timestamp())
    def getRobotMode(self): return 7        # RUNNING
    def getSafetyMode(self): return 1       # NORMAL
    def getActualRobotVoltage(self): return 48.0
    def getActualCurrent(self): return [0.0] * 6
    def isProtectiveStopped(self): return False
    def isEmergencyStopped(self): return False
    def isConnected(self): return True
    def reconnect(self): return True
    def disconnect(self): return True

RTDEReceive = RTDEReceiveInterface
`;

export const DASHBOARD_PY = `
"""Simulator stand-in for ur_rtde's dashboard_client module."""
import _urbridge as _b

class DashboardClient:
    def __init__(self, ip="", *args, **kwargs): pass
    def connect(self, timeout_ms=2000): return None
    def isConnected(self): return True
    def powerOn(self): _b.note("dashboard: power on")
    def powerOff(self): _b.note("dashboard: power off")
    def brakeRelease(self): _b.note("dashboard: brake release")
    def unlockProtectiveStop(self): return None
    def closeSafetyPopup(self): return None
    def popup(self, msg): print("POPUP:", msg)
    def closePopup(self): return None
    def loadURP(self, name): return None
    def play(self): return None
    def pause(self): return None
    def stop(self): return None
    def running(self): return False
    def robotmode(self): return "Robotmode: RUNNING"
    def programState(self): return "STOPPED"
    def polyscopeVersion(self): return "URSim 5.x (web)"
    def shutdown(self): return None
    def quit(self): return None
    def disconnect(self): return None
`;

/** Run once after the mock modules exist: reroute blocking sleeps etc. */
export const PRELUDE_PY = `
import time as _time
import builtins as _builtins
import _urbridge as _b
_time.sleep = lambda s: _b.sleep(float(s))

def _no_input(prompt=""):
    raise RuntimeError(
        "input() is not available in the simulator - hard-code the value instead")
_builtins.input = _no_input
`;
