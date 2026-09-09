#!/usr/bin/env python3
"""
Day 5 build-tiny: exploration scaling experiment.

Extends Day 4's Voronoi-partitioned exploration into an actual
TIME-STEPPED simulation -- robots move toward their nearest owned
frontier, "sense" a radius around themselves (marking unknown cells
free), and repeat until the map is mostly explored. Run this with
different robot counts (1..5) to measure:
  - exploration time (steps to reach the coverage threshold)
  - speedup = time(1 robot) / time(N robots)
  - coverage overlap = fraction of explored cells "seen" by more than
    one robot's sensor sweep during the run

This is the "does adding robots keep helping" experiment. The plan's
whole point for Day 5: one run per robot count proves nothing --
there's real run-to-run variance (different random start positions),
so this runs multiple trials per N and reports mean +/- stddev, not
a single number pretending to be the answer.

Run:
    python3 exploration_scaling.py
"""
import numpy as np
import matplotlib.pyplot as plt

from cooperative_exploration import (
    FREE, UNKNOWN, OCCUPIED, voronoi_owner, detect_frontiers,
)

GRID_SIZE = 50
SENSOR_RADIUS = 3.0
STEP_SIZE = 2.0
COVERAGE_THRESHOLD = 0.95
MAX_STEPS = 500
N_TRIALS = 5
ROBOT_COUNTS = [1, 2, 3, 4, 5]


def make_exploration_map(size: int, rng: np.random.Generator) -> np.ndarray:
    """
    Provided for you. A mostly-unknown grid with a small FREE "seed"
    in the middle (robots start having already explored a little) and
    a few scattered OCCUPIED obstacles, so paths aren't trivial
    straight lines.

    Takes an explicit `rng` (instead of a hardcoded seed) so each
    trial actually gets a DIFFERENT obstacle layout -- variance across
    trials is the entire point of Day 5's methodology.
    """
    grid = np.full((size, size), UNKNOWN, dtype=int)
    center = size // 2
    grid[center - 3:center + 3, center - 3:center + 3] = FREE
    for _ in range(6):
        r, c = rng.integers(5, size - 5, size=2)
        grid[r:r + 3, c:c + 3] = OCCUPIED
    return grid


def random_start_positions(n_robots: int, size: int, rng: np.random.Generator) -> np.ndarray:
    """
    Provided for you. N robots start spread roughly evenly around the
    center seed, with random angular + radial jitter per trial (pure
    np.linspace angles with no randomness at all would make every
    trial's starting layout identical too).
    """
    center = size // 2
    base_angles = np.linspace(0, 2 * np.pi, n_robots, endpoint=False)
    angles = base_angles + rng.uniform(-0.3, 0.3, size=n_robots)
    radii = rng.uniform(1.5, 3.0, size=n_robots)
    return np.array([
        [center + r * np.sin(a), center + r * np.cos(a)]
        for r, a in zip(radii, angles)
    ])


def sense(grid: np.ndarray, position: np.ndarray, radius: float,
          discovered_by: list, robot_idx: int):
    """
    Mark every UNKNOWN cell within `radius` of `position` as FREE, and
    record that `robot_idx`'s sensor swept every cell in that radius
    (whether it was UNKNOWN or already FREE) -- you need that
    bookkeeping for the overlap metric later. Modifies `grid` and
    `discovered_by` IN PLACE; no return value needed.

    `discovered_by` is a GRID_SIZE x GRID_SIZE list-of-lists, where
    discovered_by[row][col] is a python set() of robot indices that
    have sensed that cell at some point in the run.

    TODO(you):
      1. Get a bounding box of cells to check: rows from
         floor(position[0]-radius) to ceil(position[0]+radius),
         same for columns. Clip to valid grid indices (0 to
         grid.shape[0]-1 / grid.shape[1]-1) -- same edge-guarding
         idea as Day 4's frontier detection, just a box instead of
         4 neighbors.
      2. For each (row, col) in that box: compute the actual Euclidean
         distance from (row, col) to `position`. Skip it if that
         distance is > radius (the bounding box is a square, but the
         sensor coverage should be a circle).
      3. discovered_by[row][col].add(robot_idx) -- record it regardless
         of the cell's current state.
      4. If grid[row, col] == UNKNOWN, set it to FREE.
    """
    rows, cols = grid.shape
    r0 = max(0, int(np.floor(position[0] - radius)))
    r1 = min(rows - 1, int(np.ceil(position[0] + radius)))
    c0 = max(0, int(np.floor(position[1] - radius)))
    c1 = min(cols - 1, int(np.ceil(position[1] + radius)))

    for row in range(r0, r1 + 1):
        for col in range(c0, c1 + 1):
            dist = np.hypot(row - position[0], col - position[1])
            if dist > radius:
                continue
            discovered_by[row][col].add(robot_idx)
            if grid[row, col] == UNKNOWN:
                grid[row, col] = FREE


def nearest_owned_frontier(position: np.ndarray, frontiers: list,
                            robot_idx: int, robot_positions: np.ndarray):
    """
    Among the frontiers THIS robot owns (by Voronoi ownership), return
    the (row, col) of the closest one to `position` as a float array.
    Return None if this robot owns no frontiers right now (its whole
    region is already explored).

    TODO(you):
      1. Filter `frontiers` (a list of (row, col) tuples) down to the
         ones where voronoi_owner(np.array(f), robot_positions) ==
         robot_idx -- reusing the exact function from Day 4.
      2. If that filtered list is empty, return None.
      3. Otherwise, find the closest one to `position` (Euclidean
         distance -- same argmin pattern as voronoi_owner itself) and
         return it as a float np.array.
    """
    owned = [f for f in frontiers
             if voronoi_owner(np.array(f, dtype=float), robot_positions) == robot_idx]
    if not owned:
        return None
    owned_arr = np.array(owned, dtype=float)
    distances = np.linalg.norm(owned_arr - position, axis=1)
    return owned_arr[np.argmin(distances)]


def step_toward(position: np.ndarray, target: np.ndarray, step_size: float) -> np.ndarray:
    """
    Move `position` at most `step_size` units toward `target`, without
    overshooting it.

    TODO(you):
      1. direction = target - position
      2. distance = np.linalg.norm(direction)
      3. If distance <= step_size, you'd overshoot -- just return
         `target` directly.
      4. Otherwise return position + step_size * (direction / distance)
         -- a unit vector toward the target, scaled to step_size.
    """
    direction = target - position
    distance = np.linalg.norm(direction)
    if distance <= step_size:
        return np.array(target, dtype=float)
    return position + step_size * (direction / distance)


def run_exploration(n_robots: int, rng: np.random.Generator) -> dict:
    """
    Run ONE full exploration trial with n_robots. Returns a dict:
    {'steps': int, 'overlap': float, 'coverage': float}.

    TODO(you):
      1. grid = make_exploration_map(GRID_SIZE)
      2. positions = random_start_positions(n_robots, GRID_SIZE)
      3. discovered_by = [[set() for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
      4. total_explorable = (grid != OCCUPIED).sum() -- occupied cells
         can never become FREE, so don't count them toward coverage.
      5. For step in range(MAX_STEPS):
           a. Call sense() for every robot at its CURRENT position
              first, before moving anyone -- a real robot's sensor
              runs continuously, not just after arriving somewhere.
           b. coverage = (grid != UNKNOWN).sum() / total_explorable.
              If coverage >= COVERAGE_THRESHOLD: stop, this is your
              final step count.
           c. frontiers = detect_frontiers(grid)
           d. For each robot: target = nearest_owned_frontier(...).
              If target is not None, move that robot with
              step_toward(). If target IS None (nothing left in its
              region), leave it where it is this step.
      6. After the loop ends (by threshold or MAX_STEPS): compute
         overlap = fraction of explored, non-occupied cells whose
         discovered_by set has length > 1.
      7. Return {'steps': <the step count>, 'overlap': overlap,
         'coverage': coverage}. Hitting MAX_STEPS without reaching
         the threshold is still valid data -- it just means that
         trial didn't finish in time, which is itself informative.
    """
    grid = make_exploration_map(GRID_SIZE, rng)
    positions = random_start_positions(n_robots, GRID_SIZE, rng)
    discovered_by = [[set() for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    total_explorable = (grid != OCCUPIED).sum()

    coverage = (grid != UNKNOWN).sum() / total_explorable
    step = 0
    for step in range(MAX_STEPS):
        for i in range(n_robots):
            sense(grid, positions[i], SENSOR_RADIUS, discovered_by, i)

        coverage = (grid != UNKNOWN).sum() / total_explorable
        if coverage >= COVERAGE_THRESHOLD:
            break

        frontiers = detect_frontiers(grid)

        for i in range(n_robots):
            target = nearest_owned_frontier(positions[i], frontiers, i, positions)
            if target is not None:
                positions[i] = step_toward(positions[i], target, STEP_SIZE)

    explored_mask = (grid != UNKNOWN) & (grid != OCCUPIED)
    explored_count = int(explored_mask.sum())
    overlap_count = 0
    rows, cols = grid.shape
    for row in range(rows):
        for col in range(cols):
            if explored_mask[row, col] and len(discovered_by[row][col]) > 1:
                overlap_count += 1
    overlap = overlap_count / explored_count if explored_count > 0 else 0.0

    return {'steps': step, 'overlap': overlap, 'coverage': coverage}


# ---------------------------------------------------------------------------
# Orchestration + plotting -- plumbing, not the interesting part.
# ---------------------------------------------------------------------------

def run_all_trials():
    results = {n: [] for n in ROBOT_COUNTS}
    for n_robots in ROBOT_COUNTS:
        for trial in range(N_TRIALS):
            # Each (n_robots, trial) pair gets its OWN independent
            # generator. Seeding by trial index alone (reusing the same
            # seed across different n_robots) would keep the map/obstacle
            # layout identical across robot counts, which is actually
            # desirable here -- it means differences in `steps` are
            # attributable to n_robots, not to also getting an easier or
            # harder random map. Mixing in n_robots would remove that
            # control and make the comparison less clean.
            rng = np.random.default_rng(trial)
            result = run_exploration(n_robots, rng)
            results[n_robots].append(result)
            print(f'n={n_robots} trial={trial}: steps={result["steps"]}, '
                  f'overlap={result["overlap"]:.1%}, coverage={result["coverage"]:.1%}')
    return results


def summarize_and_plot(results: dict):
    ns = sorted(results.keys())
    mean_steps = [np.mean([r['steps'] for r in results[n]]) for n in ns]
    std_steps = [np.std([r['steps'] for r in results[n]]) for n in ns]
    mean_overlap = [np.mean([r['overlap'] for r in results[n]]) for n in ns]

    baseline = mean_steps[0]
    print('\n--- Scaling summary ---')
    print(f'{"N":>3} {"steps (mean +/- std)":>24} {"speedup":>10} {"overlap":>10}')
    for i, n in enumerate(ns):
        speedup = baseline / mean_steps[i]
        print(f'{n:>3} {mean_steps[i]:>12.1f} +/- {std_steps[i]:<8.1f} '
              f'{speedup:>9.2f}x {mean_overlap[i]:>9.1%}')

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    ax1.errorbar(ns, mean_steps, yerr=std_steps, marker='o')
    ax1.set_xlabel('number of robots')
    ax1.set_ylabel('steps to reach coverage threshold')
    ax1.set_title('Exploration time vs robot count')

    ax2.plot(ns, [m * 100 for m in mean_overlap], marker='o', color='tab:red')
    ax2.set_xlabel('number of robots')
    ax2.set_ylabel('coverage overlap (%)')
    ax2.set_title('Redundant coverage vs robot count')

    plt.tight_layout()
    plt.savefig('exploration_scaling.png')
    plt.close()
    print('\nSaved exploration_scaling.png')


def main():
    results = run_all_trials()
    summarize_and_plot(results)


if __name__ == '__main__':
    main()
