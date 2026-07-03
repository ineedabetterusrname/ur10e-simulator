# Exporting a Grasshopper toolpath to the simulator

Grasshopper definitions can't run in a browser (they need Rhino), but the
simulator's **Code lab** tab plays exported target programs. Export with one
vanilla **GhPython** component — no plugins required — then drop the `.json`
file onto *Code lab → Load file…*.

## GhPython component

Inputs (set *List Access* where noted):

| Input | Type | Access | Meaning |
|---|---|---|---|
| `P` | Plane | List | TCP target planes, in robot base coordinates |
| `J` | Text | List | optional joint targets, `"j1,j2,j3,j4,j5,j6"` in degrees |
| `F` | Text | Item | output file path, e.g. `C:\temp\toolpath.json` |
| `run` | Boolean | Item | write the file when True |

```python
"""Export UR10e targets for the web simulator (Code lab tab)."""
import json, math
import Rhino

# document units -> metres
scale = Rhino.RhinoMath.UnitScale(
    Rhino.RhinoDoc.ActiveDoc.ModelUnitSystem, Rhino.UnitSystem.Meters)

def plane_to_pose(pl):
    """Plane -> UR pose [x,y,z, rx,ry,rz] (position m, rotation vector rad).
    Plane X/Y/Z axes become the TCP frame axes."""
    r = [[pl.XAxis.X, pl.YAxis.X, pl.ZAxis.X],
         [pl.XAxis.Y, pl.YAxis.Y, pl.ZAxis.Y],
         [pl.XAxis.Z, pl.YAxis.Z, pl.ZAxis.Z]]
    tr = r[0][0] + r[1][1] + r[2][2]
    c = max(-1.0, min(1.0, (tr - 1.0) / 2.0))
    ang = math.acos(c)
    if ang < 1e-9:
        rv = [0.0, 0.0, 0.0]
    elif abs(math.pi - ang) < 1e-6:
        # 180 deg: axis from the dominant diagonal element
        ax = [math.sqrt(max(0, (r[i][i] + 1.0) / 2.0)) for i in range(3)]
        i = ax.index(max(ax))
        for j in range(3):
            if j != i and ax[j] > 1e-9:
                ax[j] = math.copysign(ax[j], r[i][j] + r[j][i])
        n = math.sqrt(sum(a * a for a in ax))
        rv = [a / n * ang for a in ax]
    else:
        s = 2.0 * math.sin(ang)
        rv = [(r[2][1] - r[1][2]) / s * ang,
              (r[0][2] - r[2][0]) / s * ang,
              (r[1][0] - r[0][1]) / s * ang]
    return [pl.OriginX * scale, pl.OriginY * scale, pl.OriginZ * scale] + rv

moves = []
for txt in (J or []):
    q = [float(v) for v in str(txt).replace(";", ",").split(",")]
    moves.append({"type": "joints", "q": q[:6], "v": 60})
for pl in (P or []):
    moves.append({"type": "pose", "pose": plane_to_pose(pl), "v": 0.25})

if run and F and moves:
    with open(F, "w") as f:
        json.dump({"units": "deg", "moves": moves}, f, indent=1)
    a = "wrote %d moves -> %s" % (len(moves), F)
else:
    a = "%d moves ready (set run=True to write)" % len(moves)
```

## File format

```json
{
  "units": "deg",
  "moves": [
    { "type": "joints", "q": [0, -90, -90, -90, 90, 0], "v": 60 },
    { "type": "pose",   "pose": [-0.4, -0.29, 0.5, 0, 3.1416, 0], "v": 0.25 }
  ]
}
```

- `joints` — `q` in `units` (`deg` default, or `rad`); `v` = leading-axis speed in units/s.
- `pose` — `[x, y, z, rx, ry, rz]`: metres + rotation vector (radians) in the
  **robot base frame** (Z-up, same as `getActualTCPPose` on the real UR10e);
  `v` = TCP speed in m/s. Values that look like millimetres (>10) are
  auto-converted. Pose targets are played as straight TCP lines (IK-solved).
- A plain **CSV** also works: one line per waypoint, `j1..j6` in degrees,
  optional 7th column = speed (deg/s).

## Notes

- Model the robot base at the world origin, Z up, in Rhino — poses are taken
  as-is in that frame.
- The simulator validates joint limits, keeps velocity/acceleration limits,
  and protective-stops on self/floor/rail collision during playback — if your
  toolpath survives here, the geometry is sound before you book lab time.
