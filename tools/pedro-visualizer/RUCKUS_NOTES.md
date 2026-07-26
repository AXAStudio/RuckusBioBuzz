# Ruckus Bio Buzz Pedro Visualizer

This is a local copy of the Pedro Pathing visualizer:

https://github.com/Pedro-Pathing/Visualizer

Local customization:

- The export menu includes `TeamCode Auto`.
- That exporter generates an FTC OpMode in `org.firstinspires.ftc.teamcode.auto`.
- Generated autos use this repo's `PathStep` helper for the start and endpoint poses.
- The sequence editor supports `Path`, `Wait`, and `Event` items. `Add Event` creates a timed `Shoot` event by default.
- The control panel supports named pose variables that can be assigned to the start pose or path endpoints.
- The starting point has an editable `Heading` in degrees (literal or expression) — the pose the robot is placed at, which `setStartingPose` receives. Turning from it to whatever the first path needs is timed like any other rotation, so a start heading that does not match the first path shows its real cost. A tangential start point has no heading of its own and follows the first path until a value is entered.
- The control panel supports path variables that store a reusable copy of a selected path chain and can insert that stored path back into the route.
- Endpoints assigned to pose variables still allow editable heading mode, linear start heading, and heading curve while the pose controls the final position/heading.
- Individual paths can be duplicated, selected groups of paths can be wrapped in repeat loops, and path chains can be looped as a group.
- Repeat loops and `if` blocks hold paths, waits and events in one ordered list, so a pause or a mechanism trigger can sit between two paths inside a loop and run on every pass. Drag any of them in by its handle.
- Number variables store reusable numeric constants for repeat counts, waits/events, path speed, and event marker values. TeamCode export emits them as Java constants.
- Each path has a `Path Speed` scale from `0.05` to `1.0`; TeamCode export passes it to PedroPathing as the per-path max power.
- Each path can define parallel event markers. TeamCode export turns them into PedroPathing parametric, temporal, or pose callbacks so mechanisms can start while the path is still running, with optional timed finish handling.
- The control panel has a telemetry readout that follows playback, showing the current path state, pose, path progress, path speed, active parallel events, and the next queued event marker.
- Visualization settings can show event pins on the field, per-segment path length/time labels, a 30-second autonomous countdown overlay, an optional blue-to-red velocity gradient based on the trapezoidal motion profile, and estimated swerve module angles on robot previews.
- Undo/redo history keeps recent recovery snapshots in localStorage across reloads.
- Browser project files are stored in IndexedDB, with one-time migration from the old localStorage file blob.
- Linear heading paths include an editable heading curve graph. Values above `1.0` shift more of the turn toward the end of the path.
- The control panel includes `Mirror X` and `Mirror Y` actions for switching alliances by flipping path coordinates and headings on either field axis.
- Paths are checked against the obstacles and the field walls using the robot's own footprint at the heading it holds. Path rows show `Hits <thing>` or the clearance in inches, the field draws the robot where it is in trouble, and the export warns before the auto reaches a match. The chip is a button that moves a control point, an endpoint or the starting point until the robot clears.
- Number fields drag: hover a literal for an `ew-resize` cursor and drag sideways to adjust it, with Shift for coarse and Alt for fine. Fields driven by a variable do not drag — scrub the variable itself in the Variables tab.
- `Settings → Motion Parameters` can read the robot's tuned PedroPathing constants straight out of TeamCode, showing each value, what it changes it from, and which Java call it came from — plus what PedroPathing does not measure, so the remaining guesses stay visible.
- The TeamCode exporter validates generated autos, can download a `.java` file, and can save directly to `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/` while running from Vite. Direct saves run `:TeamCode:compileDebugJavaWithJavac` and restore the previous file if the generated Java does not compile.

Path chaining:

- Consecutive paths are followed as a single PedroPathing `PathChain`. A chain
  built through `pathBuilder()` defaults to `DecelerationType.LAST_PATH`, so it
  brakes only on its final path — the robot drives straight through every
  waypoint inside the chain instead of decelerating and settling on each one.
- The time estimate profiles a chain as one accelerate/cruise/decelerate move,
  so interior paths are timed at cruise speed. Four collinear paths take 4.3s
  chained versus 8.0s stopping at each.
- A chain is cut only where the robot genuinely has to stop: a wait or event
  step follows, a repeat loop starts its next pass, a block ends, the next path
  uses a different speed, a path can be switched off at runtime, or `Stop at
  end` is ticked on the path. `buildChainRuns` owns those rules and both the
  estimate and the exporter read them, so the auto stops exactly where the
  editor says it will.
- Each path row shows `Chains on` or `Stops`, the field marks every full stop
  (Settings → Stop Points), and Telemetry counts them. Stops are what cost time
  now, so they are the thing worth seeing.

Turn model:

- The robot never stops to turn. Not even at the head of a chain: `followPath`
  starts driving and correcting heading in the same command, so a stationary
  rotation was always fiction.
- Turning still costs time, because driving and turning draw on one motor-power
  budget — the drivetrain mixes the pathing and heading vectors and normalizes
  them into wheel power. It is charged as a cap on speed, applied where the
  turning happens: sweeping the heading `dθ` per inch travelled means the robot
  can go no faster than `aVelocity ÷ dθ/ds`, and with the budget shared the
  cruise speed drops to `1 ÷ (1/maxVelocity + coupling · (dθ/ds) / aVelocity)`.
- `Settings → Turn Coupling` (0..1, default 1) sets how much of the budget the
  two share, so it can be calibrated against the robot instead of guessed. At 1
  the per-inch costs of driving and turning add — a 180° sweep over 40 in halves
  the cruise speed to 20 in/s, exactly saturating the drivetrain. At 0 only the
  hard spin-rate limit applies.
- The heading rate is read along the path, so an S-curve on a tangential path or
  a heading curve that front-loads its turn slows the robot where the turning
  actually is. It is smoothed over half the robot's width, because a robot
  blends a heading change over roughly its own length rather than snapping to it
  at a point.

Cornering:

- Speed through a curve is capped at `sqrt(grip × radius)`: holding radius `r`
  at speed `v` needs `v² / r` sideways, and past what the wheels give the
  follower cannot hold the line. `Settings → Cornering Grip` is that limit in
  in/s², measurable by driving an arc until the robot slides.
- A chain is profiled numerically rather than as a trapezoid — the curve is
  sampled, each sample capped by its own radius, then a forward pass keeps
  acceleration in range and a backward pass leaves room to brake for what is
  coming. On a straight it reproduces the trapezoid exactly.
- Grip is a property of the wheels, so the per-path speed scale does not touch
  it. A radius tighter than half the robot is treated as pivoting rather than
  cornering, since sampled geometry reports a near-zero radius at a hard join
  between two paths and the turn model already covers that cost.

Steady speed:

- A Bezier's parameter is not proportional to its arc length — with control
  points bunched to one side the curve can cover ground twenty times faster at
  one end than the other. Every path carries an arc-length table so a distance
  maps back to the right parameter; advancing the parameter in proportion to
  distance instead made the robot visibly surge and stall along a path it
  should cross at a steady speed.
- The profile is built from samples spread evenly along the curve itself, not
  at evenly spaced parameters and not by interpolating the chords between them.
  Curvature read across three nearly-touching or unevenly spaced points is
  mostly rounding error, and a noisy curvature reading becomes an oscillating
  speed cap.
- Distances, chain offsets and arc-length tables all live on one ruler. Mixing
  a chord-measured length with a sampled one makes the position drift and then
  snap back at every path boundary.
- Within a profile interval the robot holds a constant acceleration, so time
  maps to distance as a quadratic. Interpolating linearly held the speed flat
  across each interval and stepped it at every boundary — smooth acceleration
  chopped into stairs.
- Turning is charged as a cap on speed where the turning happens, not as a
  stretch applied to a whole path. Stretching per path made the speed jump
  wherever two chained paths turned by different amounts.

Velocity gradient:

- Coloured from the same numbers as the time estimate, never from the path on
  its own. Two things that are invisible from a single path decide the speed:
  the profile belongs to the whole PathChain, so a path in the middle of one is
  driven at cruise rather than ramping up and back down; and turning slows the
  path it happens on. Both come from the timeline, so the colours cannot
  disagree with the clock.

Heading continuity:

- A path states a heading *goal*; where two paths meet that goal can jump — a
  tangential path after a linear one picks up wherever its curve points. The
  robot cannot jump, so it rotates toward the new goal at `aVelocity` while it
  keeps driving. `headingAlongPath` blends over that catch-up window, and the
  animated robot, the swept-area preview and the onion layers all read it, so
  the heading is continuous everywhere.
- The head of a chain never needs a catch-up: the robot turns in place there and
  that rotation is timed.
- A path row shows `Heading jump N°` when its goal does not pick up where the
  previous path left off. On a linear path the chip is a button that restarts it
  at the incoming heading; on a tangential path the geometry decides, so it is
  only a warning. The exporter warns above 20°.
- Switching a path to linear seeds its start heading from the heading the robot
  arrives with, so the common way of creating a jump no longer does.

Robot picture from CAD (`src/utils/cadRobot.ts`):

- `Settings → Robot Configuration → Build from CAD` reads an STL (binary or
  ASCII) or OBJ export and renders a top-down picture of the real robot. Both
  formats are parsed here rather than pulling in a mesh library.
- Pick the model's up axis and sign, spin it in the plane until the robot's
  forward points right — the field draws the image that way at heading 0 — and
  say what units the CAD is in.
- The view basis is chosen so `right × up` equals the axis being looked down,
  which keeps the render from coming out mirrored and putting every asymmetric
  feature on the wrong side.
- Faces are painted back to front and shaded by how squarely they face the
  camera, so the top deck reads bright and the sides fall away.
- `Use image and size` also sets the robot dimensions from the projected
  footprint. `rWidth` is the extent along forward (the image's horizontal
  axis), `rHeight` is across.
- A real export runs to hundreds of thousands of triangles, so the work is split
  in two. Reading, parsing, centring and per-face normals all happen once at
  upload behind a progress bar; changing the axis or dragging the rotation only
  re-projects what is already prepared.
- The redraw is a scanline rasteriser with a depth buffer, not a painter's-order
  pass. Sorting every face and making a canvas call per triangle was what made
  dragging crawl; rasterising needs no sort at all, costs what the covered
  pixels cost rather than what the mesh weighs, resolves overlapping decks
  exactly, and leaves no seams between neighbouring triangles.
- Buffers are reused between frames, and a drag renders at draft resolution with
  the supersampled version landing once the controls stop moving. On a 19 MB,
  400,000-triangle STL that is a first preview at ~110 ms and a rotation drag at
  ~30 fps.

Clearance (`src/utils/clearance.ts`):

- A path is a curve through the middle of the robot, and the middle of the robot
  fits places the robot does not. What is checked is the footprint, planted every
  0.75in along the route and measured against the obstacles and the field walls.
- It is checked at the heading the robot actually holds, taken from the same
  heading model the animation uses. That is the whole point: an 18in robot clears
  a 20in gap square-on and cannot fit it at 45°, because its diagonal is 25.5in.
  Two of the three problems on the first real auto this ran against were exactly
  that — clearances that only exist because the robot is turned.
- `Settings → Safety Margin` is how close the robot may come before a path is
  flagged. Inside the margin is amber, overlapping is red, and both are drawn on
  the field as the robot's own shape at the worst point, so the problem is a
  picture rather than a number.
- Overlap is measured as how far past the boundary the intrusion reaches, not as
  the distance between outlines. Once two shapes cross, that distance is zero
  however deep the robot is buried, so a path driven straight through an obstacle
  would report as just touching.
- Samples are spread evenly along each curve. Stepping the curve parameter would
  sample densely at one end and stride past a whole obstacle at the other, for
  the same reason the speed profile cannot use it either.
- Where one path ends and the next begins is one pose, not two, so it is checked
  once. Counting it twice would report a single tight corner as two problems. A
  branch starts somewhere else, so the join has to actually line up.
- A CAD upload also stores the robot's real outline, and clearance uses it
  instead of the bounding rectangle. A robot with a corner intake sweeps a
  hexagon, and the rectangle claims material where there is only air, so gaps it
  genuinely fits through would read as collisions. The outline is the convex hull
  of the projected footprint, reduced to the extreme point of each thin column
  and row first because a real export has over a million vertices — a reduction
  that cannot shrink the shape, and what it could shave off is added back. It is
  stored with the size it was measured at, so editing the robot's dimensions
  rescales it rather than silently checking the wrong robot.
- The chips on the path rows, the shapes on the field, the Telemetry readout and
  the export warnings all read one report, so they cannot disagree.

Fix collision issues:

- The chip on a flagged path is a button that moves a waypoint on X and Y until
  the robot clears. It searches outward for the smallest move that works, since
  the smallest change is the one least likely to undo whatever the path was
  placed for.
- Handles are tried in order of what they cost to accept, and the first that
  reaches the safety margin wins. A **control point** only bends the curve
  between two waypoints the robot still hits, so it is nearly free and goes
  first; an **endpoint** changes where the robot parks, usually a scoring
  position; the **previous path's endpoint** goes last, because it edits a
  different path than the button that was pressed. A collision partway along a
  path is normally a control-point problem and out of an endpoint's reach
  entirely.
- The first stretch of a path is pinned to wherever the previous path left the
  robot, so a collision there is not really that path's at all: its own endpoint
  and control points are downstream of the problem and none of them can move it.
  That is why the button has to be able to reach back. On the first real auto
  this ran against, three of the five collisions were at a hand-off like that,
  and the button reported it could do nothing while pressing the *other* path's
  button fixed them.
- When nothing works, the message names the thing that would have to change
  rather than shrugging: which path hands the robot over, whether that endpoint
  is locked or comes from a pose variable, or that the gap is simply tighter than
  the robot. A path handed a bad pose by a pose-variable endpoint says so —
  that position is owned by the variable, and no amount of moving points here
  will shift it.
- The starting point has its own button, labelled `fix start position only` so it
  is clear it moves where the robot is staged and nothing else. It is a separate
  button rather than one more handle on the path button. It is judged on where the robot is *staged*, not on the whole
  first path, because a path that drives through a goal is that path's problem
  and re-staging cannot fix it. Mixing the two objectives made the search
  re-stage the robot to gain an inch at the start while leaving the path driven
  into a wall. The rest of the path still has a veto: a move that makes it worse
  scores as that worse number. It also reaches further, 24in against 14in, since
  a robot parked inside a goal has to come most of its own width to get out.
- The search spirals rather than pushing away from the obstacle, because a
  handle governs the shape of the whole curve: the direction that clears a
  problem in the middle of a path is often not the direction away from the thing
  being hit.
- The path that follows starts where this one ends, so both are measured on
  every endpoint candidate. A move that traded one collision for another would
  not be a fix, and none is offered in that case.
- Candidates are scanned against a coarsely sampled route and the winner is then
  re-measured at full resolution; anything that does not survive that is
  discarded. Scanning at the real resolution made one click take most of a
  second. A fix also has to be worth making — it must move at least 0.02in, buy
  at least 0.05in, and actually end up clear. Without those the search happily
  reported nudging a waypoint half an inch to go from 0.99in to 0.99in, and
  reshaping a path by a foot to end up still overlapping.
- Clearing the obstacle and reaching the safety margin are separate outcomes and
  the button says which it got. It aims for the plateau rather than for bare
  contact, since stopping at the first move that merely gets off the obstacle
  leaves the robot at 0.00in.
- A locked point or one driven by a pose variable is left alone — the pose
  variable owns the position, so writing x/y would not stick — and the chip stays
  a plain label there. Where nothing can help, the button says so instead of
  moving the path for nothing.
- It is one edit on the undo stack.

Fix all collisions:

- One button in the step toolbar walks everything currently flagged, worst
  first, applying the best fix for each until nothing more improves. The
  per-path buttons each fix one thing; a chain of hand-offs would otherwise have
  to be clicked through one at a time. On the first real auto this ran against
  it reported `6 moves — everything clears by 1in`, from five collisions.
- Targets are re-derived after every applied fix rather than planned up front:
  moving one waypoint moves the start of the path after it, so what is still in
  trouble changes as it goes. A target that cannot be fixed is not retried, since
  its geometry has not changed and retrying would spin.
- Poses are tried before the paths bound to them. A path whose endpoint a pose
  owns cannot be fixed on its own, so going path-first would only burn a round
  marking it unfixable.
- It yields to the page between rounds. Each round runs a few hundred candidate
  measurements and would otherwise freeze the frame.

Pose variables:

- A point bound to a pose takes its position from the variable, so the path it
  sits on cannot move it — writing x/y on the point is overwritten on the next
  resolve. The pose itself is a handle, and the Variables tab shows a fix button
  on any pose something flagged depends on.
- Moving a pose moves everywhere it is used, so every bound path and the path
  after each of them is measured on every candidate. A move that clears one and
  buries another is not offered.
- The fix is written to the variable, not to the points, and the usual sync
  carries it to everything bound to it.

Dragging numbers:

- The drag lives on the field's **label** — the `X:`, `Y:`, `Start:`, `Deg:` next
  to the box — not on the box. Hover a label for an `ew-resize` cursor and drag
  sideways. Shift is ten times coarser for crossing the field, Alt ten times
  finer for the last hundredth. Typing a coordinate is fine for a first guess and
  bad for the tenth, and the field redraws as the drag goes, so a position can be
  dialled in by eye instead of by retyping.
- On the input it would have had to tell a click for the caret apart from the
  start of a drag, and a stray few pixels while clicking would nudge the value.
  A label has no other job, so there is nothing to disambiguate: labels drag,
  boxes type.
- A label whose field holds an expression goes inert — no cursor, no drag. The
  field is owned by whatever variable it names, and writing a number over it
  would silently cut the link; the variable itself drags in the Variables tab,
  and everything reading it follows.
- `scrubbable` in `src/utils/scrub.ts` is a Svelte action, so a label attaches it
  in one line with the same value and callbacks its input already gets. The
  labels `ExpressionInput` renders itself carry it too, which is what covers the
  Variables tab. Headings ask for a bigger step: at the coordinate default a full
  turn would need a 7200px drag.
- The value is computed from where the press started rather than accumulated, so
  a drag out and back lands exactly where it began instead of drifting by the
  rounding of every frame between. It commits once at the end, so a drag is one
  entry on the undo stack rather than one per frame.

Tuned constants from TeamCode (`src/utils/teamcodeConstants.ts`):

- `Settings → Motion Parameters → Load from TeamCode` reads the robot's own
  PedroPathing constants instead of asking anyone to guess them. Tuning already
  measures several of these on the real robot, and they were sitting two
  directories away while the visualizer ran on hand-typed defaults.
- On the first auto this ran against, `maxVelocity` was set to 40 in/s against a
  tuned 73.9, and `maxDeceleration` to 30 in/s² against a measured 197.1 — off by
  1.85x and 6.6x. The auto's estimate went 30.99s to 25.07s, which changed the
  answer to the only question that matters about it: it did not fit in 30
  seconds, and it does.
- `config.jsonc` picks the drivetrain, and the matching constants file is read.
  A swerve states one top speed; a mecanum is tuned forward and sideways
  separately, and paths are driven forward, so `xVelocity` is the cap and the
  sideways figure is shown as context only.
- Comments are stripped before anything is read. These files park alternates
  behind `//`, and picking one up would silently import a number the robot is
  not using.
- Nothing is applied until it is shown: the dialog lists each value, what it
  changes it from, and the Java call it came from.
- What PedroPathing does *not* measure is listed just as explicitly, so the
  guessed numbers stay visibly guessed. There is no acceleration limit in the
  model at all (it is PIDF plus a power cap), no stated turn rate, nothing
  matching this tool's turn coupling, and `centripetalScaling` is a correction
  gain rather than a sliding limit, so it does not convert to cornering grip.
- `forwardZeroPowerAcceleration` is flagged as a ceiling rather than applied
  silently. It is how hard the robot slows with *no power applied*, which is the
  most it can brake, not what the follower holds while still tracking a line —
  taken as-is the estimate leans optimistic.
- The endpoint only exists under the dev server, since that is the only place
  the repo is on disk to read from, and the dialog says so if it is missing.

Onion layers:

- The bodies drawn along the route are for judging clearance by eye, so they use
  the robot's real outline when CAD has given us one rather than a bounding box.
  Drawing a box around a robot whose shape is known would show the wrong shape
  for the one job they have, and would disagree with both the picture on the
  field and the clearance check.
- `getVisibleOnionLayers` takes the playback position as an argument instead of
  reading it from the component. Svelte works out what a reactive block depends
  on from the names appearing in it, and reading `percent` inside the helper left
  the blocks that draw the bodies with no dependency on it: with `Next point
  only` on they froze on whichever path was current when something else last
  changed, while the swerve overlay — whose block does mention the playback
  position — kept following the robot. The two drifted apart on screen.
- The geometry and the filter are separate reactive steps, because only the
  filter depends on playback. Walking the route to place every body is the
  expensive half and it was being redone on every animation frame for a result
  that had not changed — at a 1in spacing that alone blocked the main thread for
  37ms a frame, 55ms with swerve modules on, which is 18fps before anything else
  on the page gets a turn. Split, and with the Two.js bodies rebuilt only when
  the visible set actually changes, the same scene costs 4-5ms a frame, the same
  as having them switched off.
- Svelte treats any assignment of an object as a change, so the block that builds
  the bodies still runs every frame; what stops it allocating a few thousand
  anchors for an identical picture is an explicit guard on the layer array's
  identity, the selected path, the colour and the field scale.
- Spacing is measured along the route, so the honest check is how many bodies
  each path gets for its length. Straight-line distance between consecutive
  bodies is shorter wherever the route doubles back, which is not a spacing
  error — on the first real auto this ran against, three joins reverse and read
  as 0.17in, 0.75in and 2.08in gaps against a 3in spacing while every path's
  count was exact.

Waits and events inside a loop or an `if`:

- A repeat loop or an `if` block holds an ordered list of members that can be
  paths, waits or events. It used to hold a bare list of path ids, so a pause
  between two paths inside a loop was not representable at all — which is why
  dragging one in did nothing. Files written before this are converted on load,
  and the old field stays on the type as deprecated so a project saved months
  ago still opens.
- A wait inside a three-pass loop happens three times, in its place in the
  order: `path, wait, path` becomes
  `path wait path  path wait path  path wait path`. A loop that cannot run
  spends none of them.
- In the generated Java a hold takes a slot in the loop's `PathChain[]` with a
  null chain, and a parallel `long[]` carries its duration; `followRepeatStep`
  reads a null chain as "hold here" rather than something to drive. An event
  member fires the same generated method a top-level one would. Verified by
  compiling the output against the real SDK.
- Only paths are reconciled against `lines`. A wait carries its own data, so
  there is nothing for it to have gone stale against, and dropping one there
  would quietly delete a step from inside a loop.

Route invariants (`src/utils/sequence.ts`):

- `buildRoute` is the single walk of the step list. The field drawing, the time
  estimate, the animation, the overlays and the exporters all read the route
  from it, so they cannot disagree about order, start points, or which steps run.
- Every path appears in the step list exactly once. `reconcileSequence` drops
  steps whose path is gone and appends any path the list forgot, so a path can
  never sit in the editor while being invisible to the field and the exporters.
- Adjacent `if` blocks are one if / else-if chain: every branch starts where the
  chain is reached, and only the first branch whose condition holds runs.
- A step whose `Enabled if` is false costs no time and does not move the robot,
  matching what the generated Java does. It is still drawn, faded.
- A parallel event with a zero duration fires once. The generated
  `startParallelEvent` finishes it immediately, so the field and telemetry show
  it as a momentary trigger rather than as active until the end of auto.

Run locally:

```powershell
npm install
npm run dev -- --host 127.0.0.1 --port 5173
```
