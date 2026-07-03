/**
 * Built-in example scripts for the Code lab. These are real ur_rtde programs:
 * they run unchanged on the lab's UR10e and on this simulator. The same
 * files ship in the Robo Lab repo under UR10e_Documentation/Python_Samples/.
 */

export const FIRST_MOVES = `# ============================================================
# SAMPLE 1 - FIRST MOVES
# Connect, read the robot state, do a joint move and a straight
# TCP move, then return. Works on the real UR10e and in this
# simulator without changes.
# ============================================================
import rtde_control
import rtde_receive
import math

ROBOT_IP = "YOUR_ROBOT_IP"   # ignored by the simulator

def main():
    rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
    rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)

    # --- read where the robot is right now -------------------
    q = rtde_r.getActualQ()
    print("joints [deg]:", [round(math.degrees(v), 1) for v in q])
    tcp = rtde_r.getActualTCPPose()
    print("tcp x/y/z [m]:", [round(v, 3) for v in tcp[:3]])

    # --- a safe joint-space "work" pose (moveJ = fast, curved)
    work = [0.0, -1.31, -1.75, -1.66, 1.57, 0.0]
    rtde_c.moveJ(work, speed=1.0, acceleration=1.4)

    # --- straight-line TCP moves (moveL = slower, exact path)
    pose = rtde_r.getActualTCPPose()
    pose[2] -= 0.15                      # 15 cm straight down
    rtde_c.moveL(pose, speed=0.25, acceleration=1.2)
    pose[1] += 0.20                      # 20 cm sideways (+Y)
    rtde_c.moveL(pose, speed=0.25, acceleration=1.2)

    # --- back to the work pose and report --------------------
    rtde_c.moveJ(work, speed=1.0)
    print("final tcp [m]:", [round(v, 3) for v in rtde_r.getActualTCPPose()[:3]])
    rtde_c.disconnect()

if __name__ == "__main__":
    main()
`;

export const PICK_AND_PLACE = `# ============================================================
# SAMPLE 2 - PICK AND PLACE (OnRobot 2FG7 gripper)
# Pick a brick with the parallel gripper, carry it, place it,
# then bring it back. In the simulator, add "Bricks (graspable)"
# from the catalogue first - the first brick appears exactly at
# the PICK station. The fingers stop on contact with the brick;
# opening past its width releases it (it falls and settles).
# ============================================================
import rtde_control
import rtde_receive

try:
    from onrobot import TwoFG7            # provided by the web simulator
except ImportError:                        # real robot: OnRobot Compute Box
    import xmlrpc.client
    class TwoFG7:
        """Minimal 2FG7 wrapper. Check the method names against your
        OnRobot Compute Box XML-RPC documentation before first use."""
        def __init__(self, ip, port=41414):
            self._cb = xmlrpc.client.ServerProxy("http://%s:%d" % (ip, port))
        def grip(self, width_mm, force=80, speed=100, wait=True):
            self._cb.twofg_grip_external(0, float(width_mm), force, speed)
        def open(self, width_mm=73.0):
            self.grip(width_mm, force=20)
        def get_width(self):
            return self._cb.twofg_get_external_width(0)

ROBOT_IP = "YOUR_ROBOT_IP"   # ignored by the simulator

BRICK_W = 60.0               # brick width [mm]; sim bricks are 120 x 60 x 60
PICK    = (0.69, -0.32)      # base-frame X,Y of the pick station [m]
PLACE   = (0.69,  0.08)      # base-frame X,Y of the place target [m]
Z_TRAVEL = 0.25              # carry height above the base plane [m]
Z_GRIP   = 0.032             # grip-point height when picking [m]

def goto(rtde_c, pose, x, y, z, v=0.25):
    p = list(pose)
    p[0], p[1], p[2] = x, y, z
    rtde_c.moveL(p, speed=v)

def main():
    rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
    rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
    gripper = TwoFG7(ROBOT_IP)

    work = [0.0, -1.31, -1.75, -1.66, 1.57, 0.0]   # tool points straight down
    rtde_c.moveJ(work, speed=1.0)
    pose = rtde_r.getActualTCPPose()               # keep this orientation
    gripper.open(73)

    for i, (src, dst) in enumerate([(PICK, PLACE), (PLACE, PICK)]):
        print(f"cycle {i + 1}: pick {src} -> place {dst}")
        goto(rtde_c, pose, src[0], src[1], Z_TRAVEL)
        goto(rtde_c, pose, src[0], src[1], Z_GRIP, v=0.1)
        gripper.grip(BRICK_W - 5)                  # fingers stop on the brick
        goto(rtde_c, pose, src[0], src[1], Z_TRAVEL)
        goto(rtde_c, pose, dst[0], dst[1], Z_TRAVEL)
        goto(rtde_c, pose, dst[0], dst[1], Z_GRIP + 0.003, v=0.1)
        gripper.open(73)                           # release - brick settles
        goto(rtde_c, pose, dst[0], dst[1], Z_TRAVEL)

    rtde_c.moveJ(work, speed=1.0)
    print("done - the brick is back at the pick station")
    rtde_c.disconnect()

if __name__ == "__main__":
    main()
`;

export const DRAW_CIRCLE = `# ============================================================
# SAMPLE 3 - DRAW A CIRCLE
# Trace a horizontal circle with the TCP using short moveL
# segments - the basic recipe behind any toolpath (welding,
# gluing, drawing...). Try changing RADIUS or SEGMENTS.
# ============================================================
import rtde_control
import rtde_receive
import math

ROBOT_IP = "YOUR_ROBOT_IP"   # ignored by the simulator

RADIUS = 0.10        # circle radius [m]
SEGMENTS = 24        # straight segments approximating the circle

def main():
    rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
    rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)

    work = [0.0, -1.31, -1.75, -1.66, 1.57, 0.0]
    rtde_c.moveJ(work, speed=1.0)

    # circle centre = current TCP; keep Z and orientation fixed
    cx, cy = rtde_r.getActualTCPPose()[:2]
    pose = rtde_r.getActualTCPPose()

    def waypoint(k):
        a = 2 * math.pi * k / SEGMENTS
        p = list(pose)
        p[0] = cx + RADIUS * math.cos(a)
        p[1] = cy + RADIUS * math.sin(a)
        return p

    rtde_c.moveL(waypoint(0), speed=0.25)          # onto the circle
    for k in range(1, SEGMENTS + 1):
        rtde_c.moveL(waypoint(k), speed=0.25)
        if k % 6 == 0:
            print(f"{k}/{SEGMENTS} segments")

    rtde_c.moveJ(work, speed=1.0)                  # back to centre pose
    print("circle complete")
    rtde_c.disconnect()

if __name__ == "__main__":
    main()
`;

export const EXAMPLES = [
  { name: 'Sample 1 — first moves', code: FIRST_MOVES },
  { name: 'Sample 2 — pick & place', code: PICK_AND_PLACE },
  { name: 'Sample 3 — draw a circle', code: DRAW_CIRCLE },
];
