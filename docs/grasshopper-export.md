# Grasshopper & `Robots` Plugin Workflow

The simulator's **Code lab** tab supports three ways to run your Grasshopper toolpaths on the UR10e—with zero copy-pasting required:

---

## Method 1: 1-Click Live Sync with our Grasshopper Node *(Recommended)*

1. In the simulator's **Code** tab, click **`⬇ GH Node`** to download `UR10e_Sim_Sync.ghx`.
2. **Drag and drop `UR10e_Sim_Sync.ghx` directly onto your open Grasshopper canvas.**
3. Connect **either**:
   - The **`Code`** output from the **Robots** plugin's `Create Program` component into the node's **`Code`** input, **OR**
   - Your list of Rhino/Grasshopper target **`Plane`s** into the node's **`P`** input.
4. Back in the web simulator, click **`🔄 Live Sync`** and select `ur10e_sim_live.script` (or `ur10e_sim_live.json`) on your **Desktop**.
5. **Move any slider in Grasshopper** — the simulator automatically detects the file update, redraws the 3D TCP toolpath preview, and plays the motion!

---

## Method 2: Direct `.script` Upload or Paste from the `Robots` Plugin

If you are using the **Robots** plugin in Grasshopper:
- **Upload `.script`:** Use the `Robots` plugin's `Save Program` component to save a `.script` file, then click **`Load file…`** in the simulator's Code tab and select your `.script` file.
- **Or Paste Directly:** Connect a Panel to `Create Program → Code`, right-click the Panel → *Copy Content*, paste it directly into the Code Lab editor, and press **`▶ Run`**.

Supported URScript commands:
- `movej([j1..j6], a=..., v=...)` and `movej(p[x,y,z,rx,ry,rz], ...)`
- `movel(p[x,y,z,rx,ry,rz], a=..., v=...)` and `movep(...)`
- `set_tcp(p[...])`, `sleep(...)`, and OnRobot `twofg_grip_external(...)`

---

## Method 3: JSON / CSV Toolpath Upload

You can also upload a `.json` or `.csv` file via **`Load file…`**:

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
- `pose` — `[x, y, z, rx, ry, rz]`: metres + rotation vector (radians) in the **robot base frame** (Z-up, same as `getActualTCPPose` on the real UR10e); `v` = TCP speed in m/s. Values that look like millimetres (>10) are auto-converted.
- Plain **CSV** also works: one line per waypoint, `j1..j6` in degrees, optional 7th column = speed (deg/s).
