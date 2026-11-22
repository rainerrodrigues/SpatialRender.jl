# The viewer

## Controls

| Input | Orbit mode | Fly mode |
|---|---|---|
| left drag | orbit about the pivot | look |
| right / middle / shift-left drag | pan | pan |
| scroll | zoom | fly speed |
| W A S D, Q E (shift = 4×) | — | move, down / up |
| double-click | pick: set the pivot and log the point in local + source coordinates | same |

`Tab` switches orbit ↔ fly · `F` frame all · `T` top view · `O` ortho ·
`1`–`5` color by height / intensity / class / RGB / scan age · `+`/`-` point size ·
`P` pixel ↔ world point size · `L` LOD on/off · `M` wireframe · `G` grid ·
`B` eye-dome lighting · `R` measure mode (click two points; `Backspace` undoes) ·
`I` status overlay · `F12` screenshot · `H` help · `Esc` close.

Bag replay: `Space` play / pause · `←` `→` seek 1 s (shift: 10 s) · `[` `]`
half / double speed · `Home` restart.

Pass `block = false` to get the `Viewer` back and drive it yourself
(`step!(viewer)`), for example from a robot's update loop. Layers added to the
scene while it is open are uploaded on the next frame.

## Labels and measurements

Labels use a small stroke font drawn by the line shader, so no font files are
needed. Measure mode in the viewer (`R`) calls `measure!` on the clicked points.

```@docs; canonical=false
Annotations
annotate!
measure!
undo!
```

## Screenshots and headless use

`render_offscreen` and `screenshot` render without opening a window. On a
machine without any display (a server or CI), GLFW falls back to its null
platform and offscreen rendering still works where the OpenGL driver supports
it. Set `JULIA_GLFW_PLATFORM=null` to say this is intended and silence GLFW's
warnings.

```@docs; canonical=false
open_viewer
render_offscreen
screenshot
RenderSettings
pick!
```
