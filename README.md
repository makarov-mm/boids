# Boids 3D — Swarm Intelligence

A real-time 3D flocking simulation written from scratch in C++ with a native
Win32 + OpenGL viewer. Tens of thousands of agents self-organise into flowing,
swirling flocks using only three local rules — no global coordination, no
external libraries. Predators dive through the swarm and it splits and reforms
around them, exactly like a starling murmuration under attack.

![Screenshot](screenshot.png)

## Highlights

- **Reynolds boids in 3D** — separation, alignment and cohesion, each a local
  steering rule, producing emergent flocks from simple agents.
- **Predators** — hunters that chase the nearest boid; the flock tears open and
  closes back up around them. This is the visual payoff.
- **Scales to ~12k+ agents** — neighbour queries run through a uniform spatial
  hash (counting sort, O(N)) over agent data gathered in cell order, and the
  whole step runs on a small persistent `std::thread` pool with dynamic load
  balancing. No TBB.
- **Oriented rendering** — every agent is drawn as a small arrowhead pointing
  along its velocity and coloured by heading, so aligned sub-flocks read as
  coherent colour and you can see the flow direction at a glance. The
  arrowheads are built on the GPU: one instanced draw straight from the
  simulation's position and velocity arrays.
- **Optional obstacle** — a sphere the flock has to stream around.
- **On-screen HUD** — live FPS, population, and the full key legend, drawn with
  an embedded bitmap font (no GDI, no external font files).
- **Zero dependencies** — links only `opengl32`, `gdi32`, `user32` from the
  Windows SDK. Modern GL entry points are resolved at runtime.

## Build & run

1. Open `BoidsViewer.slnx` in **Visual Studio 2026** (toolset v145).
2. Select **Release | x64**.
3. Run with **Ctrl+F5**.

Release matters — Debug is far too slow for this many agents. Run on Windows 10
(1703+) or 11 with real GPU drivers (OpenGL 3.3+); it will not work over Remote
Desktop.

There is no CMake build here; the project is Windows-only by design (native
Win32 windowing + WGL context).

## Controls

| Input | Action |
| --- | --- |
| Left-drag | Orbit the camera |
| Mouse wheel | Zoom |
| `Space` | Pause / resume |
| `Up` / `Down` | Simulation speed (substeps per frame) |
| `O` | Toggle the spherical obstacle (live — the flock reacts from where it is) |
| `P` | Toggle predators on / off (live) |
| `R` | Reset to the initial flock |
| `H` | Show / hide the HUD |
| `Esc` | Quit |

## How it works

### Steering

Each boid carries a position and a velocity. Every step it looks at the
neighbours inside its perception radius and combines three Reynolds rules:

- **Separation** — steer away from neighbours that are too close (weighted by
  inverse square distance, so crowding is punished hard).
- **Alignment** — match the average heading of nearby boids.
- **Cohesion** — drift toward the local centre of mass.

On top of that there is a soft turn-around force near the box walls, a flee
force away from any predator inside the flee radius, and an optional push away
from the obstacle. The forces are summed into an acceleration, each rule is
clamped to a maximum steering force, speed is clamped to a `[min, max]` band so
agents never stall or run away, and position is integrated forward.

Predators run a simple seek toward the nearest boid (found through the same
spatial hash) with their own speed and turn limits.

### Neighbour search

The naive all-pairs version is O(N^2) and dies past a couple thousand agents.
Instead the boids are bucketed into a uniform grid whose cell size equals the
perception radius, built each frame with a counting sort. A neighbour query
then only visits the 3x3x3 block of cells around an agent — O(N) overall.
Positions and velocities are gathered into cell order first, so a query reads
contiguous memory and consecutive agents share most of their candidates in the
cache. Candidates are first filtered without branching (two out of three lie
outside the perception sphere, at random, which a branch would keep
mispredicting), then the neighbours are accumulated in their original order.
The per-agent work (sense + steer) is handed out to the worker threads in
small blocks, so dense flocks do not leave threads idle; acceleration and
integration are separated into phases so there are no data races.

### Rendering

The simulation lives in continuous space. Each frame the viewer uploads the
position and velocity arrays as they are, and the vertex shader builds a small
camera-facing arrowhead per instance, oriented along its velocity. Colour comes
from the normalised heading
(plus a brightness boost from speed), which makes locally aligned groups share
a colour and turns the whole flock into a shifting field of colour domains.
Predators are larger red darts. A wireframe box bounds the domain, and the HUD
is a second screen-space pass using an embedded font atlas.

## Tuning

All of the behaviour lives in `BoidsParams` (`src/boids.hpp`):

| Field | Meaning |
| --- | --- |
| `boids`, `predators` | Population sizes |
| `perception` | Neighbour radius — larger gives bigger, smoother flocks |
| `separationDist` | Personal space |
| `wSeparation`, `wAlignment`, `wCohesion` | The three Reynolds weights |
| `maxSpeed`, `minSpeed`, `maxForce` | Agility and speed band |
| `fleeRadius`, `wFlee` | How sharply boids react to predators |
| `obstacle`, `obstacleCenter`, `obstacleRadius` | The sphere obstacle |
| `threads` | Simulation threads (0 = one per hardware thread) |

The defaults are tuned for large, sweeping flocks.

## Project layout

```
src/
  vec3.hpp               3D vector maths
  mat4.hpp               matrices + orbit camera
  hashgrid.hpp           uniform spatial hash (counting sort)
  thread_pool.hpp/.cpp   persistent worker pool, dynamic blocks
  boids.hpp/.cpp         flock + predators + obstacle, multi-threaded
  gl_api.hpp/.cpp        OpenGL 3.3 entry points loaded at runtime
  renderer.hpp/.cpp      instanced arrowheads, box / obstacle lines, HUD
  viewer_win32.cpp       Win32 window, WGL context, input, frame loop
  font_atlas.hpp         embedded bitmap font for the HUD
```

## Technical notes

- **Native Win32 + WGL** — the window, input loop and GL context are all set up
  by hand; there is no GLFW/GLAD/GLEW. The viewer asks for an OpenGL 3.3 core
  context (`WGL_ARB_create_context`), and the GL functions are loaded at runtime
  from one X-macro list via `wglGetProcAddress`.
- **Per-monitor DPI aware** — the window and the HUD scale with the display.
- **C++23** (`/std:c++latest`), no third-party dependencies.
- The work scales across cores with a hand-rolled thread pool.

## License

See `LICENSE`.

## Support

If you found this project interesting or useful, you can support my work:

[![GitHub Sponsors](https://img.shields.io/github/sponsors/makarov-mm?style=flat&logo=github)](https://github.com/sponsors/makarov-mm)