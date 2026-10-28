# navcore — Project Brief for Claude Code

You are building `navcore`: an engine-agnostic voxel pathfinding library in C++17,
modelled on Baritone's solver, targeting Minecraft Bedrock movement semantics.

Read this entire file before writing code. Follow it. Where it is silent, choose
the simpler option and write it down in `DECISIONS.md`.

---

## 1. What this is, and what it is not

**In scope (v1):** goal-directed movement. A* search over a voxel grid with
Bedrock-accurate movement costs, the full Baritone goal taxonomy, a path
executor that emits abstract per-tick movement intent, and a replanning loop.

**Explicitly out of scope (v1):**
- Block mining and block placement moves (pillar, bridge, mine-through).
  The move generator must be *structured* so these drop in later (see §6), but
  do not implement them.
- Any code that attaches to, reads, or writes a running game process.
  No signature scanning, no offset tables, no vtable hooks, no injector,
  no process handles. `navcore` links against the STL and nothing else.
- Inventory, combat, interaction, chat.

The boundary in §4 is the whole point of the design. Everything above it is
patch-stable and testable in isolation. Do not reach below it.

---

## 2. Build target

- C++17. No exceptions in the hot path (search loop); `std::optional` /
  status enums for recoverable failure.
- CMake ≥ 3.20. Three targets:
  - `navcore` — static library. Zero third-party dependencies.
  - `navcore_tests` — GoogleTest, fetched via `FetchContent`.
  - `navcore_bench` — a plain `main()` that loads a fixture world and times
    a batch of searches. No framework.
- Warnings as errors: `-Wall -Wextra -Wpedantic` / `/W4 /WX`.
- Must configure and build clean on both MSVC and clang. Verify by actually
  running the build; do not assume.

---

## 3. Repository layout

```
include/navcore/      public headers — this is the entire API surface
  types.h             BlockPos, hash, PathStatus, MovementIntent
  world.h             WorldProvider interface, BlockState
  goal.h              IGoal + concrete goals
  cost.h              CostModel
  moves.h             Move, MoveGenerator
  pathfinder.h        Pathfinder, SearchConfig, SearchResult
  executor.h          PathExecutor, ExecutorConfig
  log.h               logging interface + sink registration
src/                  implementation, one .cpp per header
tests/                one test file per unit + integration tests
tests/fixtures/       hand-authored ASCII worlds (see §8)
bench/
DECISIONS.md          every judgement call you made, with reasoning
```

---

## 4. The two boundary interfaces

These are the seam. The consumer implements both. `navcore` never looks past them.

### `WorldProvider` (pure virtual, read-only)

```cpp
virtual BlockState getBlock(const BlockPos& p) const = 0;
virtual bool isLoaded(const BlockPos& p) const = 0;
```

`BlockState` is a plain struct carrying the traversability facts the solver
needs — `solid`, `liquid`, `climbable`, `avoid` (fire, cactus, magma, lava),
`walkableTop` (slabs, stairs, soul sand — anything whose collision top is not
at an integer Y), plus `collisionTopOffset` as a float. It must NOT carry
Bedrock block IDs or runtime IDs. The mapping from game blocks to `BlockState`
lives in the consumer's code, not here.

`isLoaded` returning false must be treated as "do not path through" — never as
"air". Searches that run into unloaded chunks terminate with
`PathStatus::UnloadedBoundary` and return the best partial path found.

### `PlayerController` (pure virtual, write-only)

```cpp
virtual PlayerSnapshot getSnapshot() const = 0;
virtual void applyIntent(const MovementIntent& intent) = 0;
```

`MovementIntent` is abstract and declarative — the executor's output, not
keystrokes:

```cpp
struct MovementIntent {
    double targetYaw, targetPitch;   // degrees, Bedrock convention
    double forward, strafe;          // [-1, 1]
    bool jump, sneak, sprint;
};
```

How that becomes input is the consumer's problem entirely. Ship a
`NullController` in the library (logs intents, advances a kinematic dummy) so
the executor is testable with no game present.

---

## 5. Movement cost model

Costs are in ticks. Put every constant in one `CostModel` struct with named
fields and documented defaults — no magic numbers scattered through the move
generator.

Baseline (tune against real Bedrock movement, record findings in `DECISIONS.md`):

| Move | Cost |
|---|---|
| Walk, cardinal | ~4.63 ticks/block (0.21585 blocks/tick) |
| Walk, diagonal | cardinal × √2 |
| Sprint | cardinal / 1.3 |
| Ascend one | walk + jump penalty |
| Descend one | walk, slight discount |
| Fall, n blocks | free to 3; rising penalty above; forbidden past `maxFallBlocks` |
| Swim / water traversal | heavy multiplier |
| Ladder / vine up / down | separate constants |

Bedrock is not Java here. Sprint-jump distance, step-up behaviour, and fall
damage thresholds differ. Where you are unsure, pick the conservative value,
mark it `// UNVERIFIED` and list it in `DECISIONS.md` under "needs empirical
calibration". Do not invent precise-looking constants you cannot justify.

Hard rule: a move's cost must be a lower bound on its true duration, or A*
loses admissibility and starts returning non-optimal paths.

---

## 6. Move generator

`MoveGenerator::expand(pos, world, costs, out)` appends candidate `Move`s.

Each move type is its own function with its own clearance check — do not write
one mega-function with branches. v1 types:

`WalkCardinal`, `WalkDiagonal` (requires both adjacent cardinals clear — no
corner-cutting), `Ascend`, `Descend`, `Fall`, `LadderUp`, `LadderDown`,
`SwimHorizontal`.

A `Move` carries: target pos, cost, type tag, and an ordered list of
intermediate positions the executor must pass through.

Structure for later extension: each move function has signature
`bool tryX(const Ctx&, Move& out)` and is registered in a table. Adding
`MineThrough` or `PillarUp` later means adding a function and a table entry,
nothing else. Build the table; leave the mining/placement entries out.

Player is 2 blocks tall, 0.6 wide. Clearance checks must account for the full
hitbox, not just the feet block.

---

## 7. Pathfinder

A* with a binary heap open set and an open-addressing hash map keyed by
`BlockPos::key()`. Do not use `std::unordered_map` in the search loop — write a
flat hash map; the search is the hot path and this is the single biggest win.

- Preallocate the node pool. Node count caps via `SearchConfig`.
- `SearchConfig`: `maxNodes`, `maxTimeMs`, `allowSprint`, `maxFallBlocks`,
  `allowSwim`, `heuristicWeight`.
- Heuristic: `IGoal::heuristic(pos)`, must be admissible. `heuristicWeight > 1.0`
  enables weighted A* — document that it trades optimality for speed.
- On budget exhaustion return the best-scoring node reached, not failure.
  Baritone's incremental "path toward the goal, then replan" behaviour depends
  on this.
- `SearchResult`: `PathStatus`, the node list, nodes expanded, elapsed ms.
  `PathStatus`: `Complete`, `Partial`, `UnloadedBoundary`, `NodeLimit`,
  `TimeLimit`, `NoPath`, `AlreadyAtGoal`.

### Goals (`IGoal`: `isInGoal(pos)`, `heuristic(pos)`, `describe()`)

`GoalBlock`, `GoalXZ`, `GoalYLevel`, `GoalNear(pos, radius)`,
`GoalComposite` (any-of, min heuristic), `GoalInverted` (flee — heuristic is
negated distance, needs a bounded search radius or it runs forever).

`GoalGetToBlock` is out of scope for v1 — it implies block interaction.

---

## 8. Testing

This is a search algorithm. It is fully testable with no game. Treat weak tests
as a bug.

- **Fixture worlds.** ASCII, layer per Y, loaded by a `StringWorld`
  implementation of `WorldProvider`. Legend: `.` air, `#` stone, `~` water,
  `L` lava, `=` slab, `H` ladder, `?` unloaded. Author by hand: flat plain,
  staircase, 1-wide bridge over void, walled maze with one opening, lava field
  with a safe corridor, a region bordered by `?`.
- **Admissibility test.** On fixtures small enough for exhaustive BFS, assert
  A*'s cost equals the true optimum. Any inadmissible cost constant shows up
  here and nowhere else.
- **Property tests.** Random worlds, fixed seeds: every returned path is
  contiguous, every step is a legal move, no step enters a solid or `avoid`
  block, no fall exceeds `maxFallBlocks`.
- **Boundary tests.** Unloaded regions yield `UnloadedBoundary` with a valid
  partial path. Goal inside solid rock yields `NoPath` within budget.
  Start == goal yields `AlreadyAtGoal`.
- **Executor tests.** `NullController` + kinematic dummy: assert the executor
  reaches each waypoint, triggers a replan when pushed off-path, and gives up
  after `maxStuckTicks`.
- CI-ready: `ctest` must pass with zero failures before you call anything done.

---

## 9. Executor

A tick-driven state machine: `Idle → Pathing → Following → Recalculating →
{Arrived, Failed}`.

- Aim at the next waypoint's centre, emit `MovementIntent`, advance when within
  `waypointTolerance`.
- Replan when: off-path beyond `maxDeviation`, no progress for `maxStuckTicks`,
  or a block on the remaining path changed.
- **Path splicing.** Replan from a point ~N ticks ahead of the player, not the
  current position, and splice the new path onto the old. Replanning from the
  live position makes the bot stutter. This matters more than it sounds.
- Never block the game thread. If search exceeds `maxTimeMs`, return partial
  and continue next tick.

---

## 10. Logging

Header-only interface, five levels, pluggable sink, compile-time disable via
`NAVCORE_LOG_LEVEL`. Default sink writes to stderr. No logging allocations
inside the A* loop — gate those behind `NAVCORE_TRACE_SEARCH`, off by default.

---

## 11. How to work

1. Build in the order: `types` → `world` → `goal` → `cost` → `moves` →
   `pathfinder` → `executor`. Tests alongside each, not at the end.
2. Compile and run the tests after each unit. Do not stack unverified code.
3. No stubs, no TODOs, no placeholder returns. If something is genuinely
   deferred to v2, it does not appear in the code at all — it goes in
   `DECISIONS.md` under "v2".
4. Every `// UNVERIFIED` constant gets an entry in `DECISIONS.md` saying what
   measurement would settle it.
5. When a design question has no clean answer, stop and ask rather than
   guessing silently.

Done means: `cmake --build` clean on MSVC and clang with warnings-as-errors,
`ctest` green, `navcore_bench` running a 500-block path on the staircase
fixture and printing timings.
