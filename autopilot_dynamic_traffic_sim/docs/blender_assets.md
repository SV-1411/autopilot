# Blender Asset Tips

## 1) Agent (Car/Ship)

- Start with a cube or simple mesh
- Scale to roughly 20 units in world space (matches your `AGENT_RADIUS` = 10)
- Add a simple “front” indicator (arrow or colored face) so heading is visible
- Keep under 500 triangles

## 2) Obstacles

- Use spheres or cylinders
- Radius ~10 units (matches `MOVING_OBSTACLE_RADIUS`)
- Keep under 100 triangles each
- Duplicate as needed

## 3) Environment

- Optional: a floor plane (large quad) with a grid texture
- Optional: low-poly walls if you want to visualize static obstacles

## 4) Export

- Select objects, go to File > Export > FBX (.fbx)
- Enable “Apply Transform” and “Selected Objects”
- Or export as glTF/GLB for Unity/Three.js

## 5) Materials

- Use unlit or basic diffuse shaders
- Distinct colors: agent green, obstacles red, floor gray

## 6) UVs (optional)

- If you want textures, do a simple unwrap (Smart UV Project) and use a 256x256 grid texture

---

Result: lightweight assets you can drop into Unity for CSV playback.
