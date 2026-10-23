# 2D Rigid Body Physics Sim

A small rigid body physics simulator written in plain HTML, CSS and JavaScript (no libraries).

- Circles and oriented boxes
- Gravity, restitution (bounciness) and friction
- Sequential impulse solver with positional correction
- Click empty space to drop shapes, drag shapes to throw them, space to pause

## Files
- `index.html`: page structure and controls
- `style.css`: layout and theme
- `physics.js`: the physics engine (`Physics.Body`, `Physics.World`)
- `main.js`: rendering, input and the simulation loop
- `test.js`: headless sanity tests (`node test.js physics.js`)

## Run locally
Open `index.html` in a browser.

## Live demo
Once GitHub Pages is enabled: https://infernoblip.github.io/physics-sim-2d/
