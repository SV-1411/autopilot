# Autopilot Dynamic Traffic Simulation

A 2D autonomous navigation simulation:

- Global planning: grid-based A*
- Visualization: Pygame
- Modular structure to extend into moving obstacles + replanning + DWA later

## Setup

```bash
pip install -r requirements.txt
```

## Run

```bash
python main.py
```

## Phase 1 (implemented)

- Grid map + static obstacles
- A* path planning
- Agent follows computed waypoints and draws the path
