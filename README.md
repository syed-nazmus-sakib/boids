# Boids — Flocking Sandbox

An interactive visualization of [Craig Reynolds' boids](https://www.red3d.com/cwr/boids/).
Each agent follows three simple local rules — **separation**, **alignment**,
**cohesion** — and lifelike flocking emerges. Zero dependencies, plain canvas.

## Run

Just open `index.html` in a browser (it's a classic script, no server needed).

## Play

- **Separation / Alignment / Cohesion** sliders reshape the flock's character:
  crank cohesion for a nervous blob, alignment for laminar rivers, separation
  for a scattered gas.
- **Perception radius** sets how far each boid sees its neighbours.
- **Cursor force** turns the mouse into food (attract) or a predator (repel).
- Toggle **trails**, **velocity vectors**, and the **vision radius** overlay.

Boids are coloured by heading, so when the flock aligns the colours converge.

## How it works

`main.js` runs a two-pass update each frame: pass one computes every boid's
steering acceleration from a snapshot of the flock; pass two integrates. Steering
uses Reynolds' "desired minus current velocity" with `maxSpeed`/`maxForce` limits.
