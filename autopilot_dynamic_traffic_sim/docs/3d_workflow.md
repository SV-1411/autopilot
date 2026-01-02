# 3D Workflow

## Overview

Keep your planning/simulation logic in Python (this repo). Export trajectories to CSV and import them into Unity/Blender for 3D visualization.

---

## 1) Export CSV from Python

Set in `config.py`:

```python
CSV_EXPORT = True
CSV_FILENAME = "trajectory_log.csv"
```

Run the simulation. A CSV will be written with columns:

- `t`: simulation time
- `ax, ay, ayaw, av, aomega`: agent pose and velocity
- For each obstacle `ox{i}, oy{i}, ovx{i}, ovy{i}, or{i}`: position, velocity, radius

---

## 2) Unity: CSV Playback

### Assets (Blender → Unity)

- Model low-poly car/ship and obstacles in **Blender**
- Export as `.fbx` or `.glb`
- Import into Unity as prefabs

### CSV Reader + Playback Script

```csharp
using UnityEngine;
using System.Collections.Generic;
using System.IO;

public class TrajectoryPlayer : MonoBehaviour
{
    public GameObject agentPrefab;
    public GameObject obstaclePrefab;
    public string csvFile = "trajectory_log.csv";

    class Frame {
        public float t;
        public Vector3 agentPos;
        public float agentYaw;
        public List<Vector3> obstaclePos = new();
    }

    List<Frame> frames = new();
    int current = 0;
    float startTime;

    void Start()
    {
        LoadCSV();
        startTime = Time.time;
    }

    void LoadCSV()
    {
        string[] lines = File.ReadAllLines(csvFile);
        for (int i = 1; i < lines.Length; i++)
        {
            string[] parts = lines[i].Split(',');
            Frame f = new Frame();
            f.t = float.Parse(parts[0]);
            f.agentPos = new Vector3(float.Parse(parts[1]), 0, float.Parse(parts[2]));
            f.agentYaw = float.Parse(parts[3]);
            int n = (parts.Length - 6) / 5;
            for (int j = 0; j < n; j++)
            {
                float ox = float.Parse(parts[6 + j*5]);
                float oy = float.Parse(parts[7 + j*5]);
                f.obstaclePos.Add(new Vector3(ox, 0, oy));
            }
            frames.Add(f);
        }
    }

    void Update()
    {
        float elapsed = Time.time - startTime;
        while (current < frames.Count && frames[current].t <= elapsed)
        {
            Frame f = frames[current];
            // Update agent
            if (agentPrefab != null)
            {
                agentPrefab.transform.position = f.agentPos;
                agentPrefab.transform.rotation = Quaternion.Euler(0, f.agentYaw * Mathf.Rad2Deg, 0);
            }
            // Update obstacles (you should spawn them once and reuse)
            // ...
            current++;
        }
    }
}
```

- Attach this script to a GameObject in Unity
- Assign prefabs and CSV path
- Press Play

---

## 3) Blender: CSV Animation

- Import CSV via **Import CSV as Keyframes** addon or a simple Python script in Blender
- Use the data to drive object transforms over time
- Render a video or export as `.glb`

---

## 4) Tips for Low-End Laptops

- Keep models low-poly (under 1k tris each)
- Use simple materials (unlit or basic lighting)
- In Unity, disable shadows or use baked lighting
- Reduce obstacle count for 3D playback if needed

---

## 5) Optional: Real-Time 3D via Networking (advanced)

- Run Python simulation on your laptop
- Stream poses over UDP to Unity on the same machine
- Not required for demos; CSV playback is simpler.
