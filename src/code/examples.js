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
# SAMPLE 2 - PICK AND PLACE
# The classic cycle: hover over the pick point, descend straight
# down, "grip", lift, carry to the place point, descend, release.
# time.sleep() stands in for gripper I/O commands.
# ============================================================
import rtde_control
import rtde_receive
import time

ROBOT_IP = "YOUR_ROBOT_IP"   # ignored by the simulator

VEL_J, VEL_L = 1.0, 0.25     # joint speed rad/s, TCP speed m/s
HOVER = 0.15                 # approach height above the object [m]

def descend_and_return(rtde_c, rtde_r, depth, action):
    """Move straight down by depth, do the action, come back up."""
    pose = rtde_r.getActualTCPPose()
    pose[2] -= depth
    rtde_c.moveL(pose, speed=0.1)        # slow near the object
    print(action)
    time.sleep(0.5)                      # gripper open/close placeholder
    pose[2] += depth
    rtde_c.moveL(pose, speed=VEL_L)

def main():
    rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
    rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)

    start = [0.0, -1.31, -1.75, -1.66, 1.57, 0.0]
    rtde_c.moveJ(start, speed=VEL_J)

    # pick and place TCP targets, relative to the start pose
    base = rtde_r.getActualTCPPose()
    pick = list(base);  pick[1] -= 0.15   # 15 cm to -Y
    place = list(base); place[1] += 0.25  # 25 cm to +Y

    for i in range(2):                    # two full cycles
        print(f"cycle {i + 1}: pick")
        rtde_c.moveL(pick, speed=VEL_L)
        descend_and_return(rtde_c, rtde_r, HOVER, "  gripper CLOSE")
        print(f"cycle {i + 1}: place")
        rtde_c.moveL(place, speed=VEL_L)
        descend_and_return(rtde_c, rtde_r, HOVER, "  gripper OPEN")

    rtde_c.moveJ(start, speed=VEL_J)
    print("done")
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
