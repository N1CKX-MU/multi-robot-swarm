#!/usr/bin/env python3
"""
Day 4 build-tiny: cooperative exploration via Voronoi partitioning,
standalone (no ROS, no Gazebo).

Four robots share a fake occupancy grid (0=free/explored, -1=unknown,
100=occupied/wall). Every frontier cell (a free cell touching unknown
space) gets assigned to whichever robot is CLOSEST to it -- the
Voronoi rule you derived by hand (perpendicular bisectors -> nearest-
neighbor assignment). The point: each robot only chases frontiers in
its own region, so 4 robots don't waste effort re-covering the same
territory.

Run:
    python3 cooperative_exploration.py
"""
import numpy as np
import matplotlib.pyplot as plt

GRID_SIZE = 40
FREE, UNKNOWN, OCCUPIED = 0, -1, 100

# (row, col) positions -- roughly the 4 quadrants of the grid, like
# your hand example, but not perfectly symmetric (real robots won't be).
ROBOT_POSITIONS = np.array([
    [8.0, 8.0],
    [9.0, 31.0],
    [32.0, 7.0],
    [30.0, 30.0],
])


def make_fake_map(size: int) -> np.ndarray:
    """
    Provided for you -- synthetic occupancy grid, not the interesting
    part. Builds a grid that's mostly UNKNOWN, with an already-explored
    FREE rectangle in the middle split by an OCCUPIED wall (with a
    gap), so there's real free/unknown boundary for frontier detection
    to find, and some structure for the Voronoi split to look
    interesting against.
    """
    grid = np.full((size, size), UNKNOWN, dtype=int)
    grid[8:32, 8:32] = FREE
    grid[8:32, 19:21] = OCCUPIED
    grid[18:22, 19:21] = FREE  # gap in the wall
    return grid


def voronoi_owner(point: np.ndarray, robot_positions: np.ndarray) -> int:
    """
    Given a single (row, col) point, return the INDEX of the closest
    robot in robot_positions. This one function IS the Voronoi rule --
    no bisector geometry needed in code, just "who's nearest."

    TODO(you):
      1. distances = np.linalg.norm(robot_positions - point, axis=1)
         -- this computes the Euclidean distance from `point` to EVERY
         robot at once (broadcasting: robot_positions is (n_robots, 2),
         point is (2,), the subtraction broadcasts automatically).
      2. Return the index of the smallest distance: np.argmin(distances).
    """

    distances = np.linalg.norm(robot_positions - point, axis=1)
    return np.argmin(distances)
    




def voronoi_map(grid: np.ndarray, robot_positions: np.ndarray) -> np.ndarray:
    """
    Return a same-shape array where every cell holds the INDEX of its
    owning robot -- this is "color the whole map by Voronoi region."

    TODO(you): nested loop over every (row, col) in the grid (two
    nested `for` loops over grid.shape[0] and grid.shape[1] is
    completely fine -- this isn't performance-critical at this grid
    size). For each cell, call voronoi_owner(np.array([row, col]),
    robot_positions) and store the result in an output array of the
    same shape as `grid`.

    Optional stretch once the loop version works: this can be fully
    vectorized with no loops at all using np.meshgrid to build every
    (row, col) pair at once and broadcasting against all 4 robots
    simultaneously -- worth trying only after the simple version is
    correct and you understand why it's correct.
    """

    regions = np.zeros_like(grid,dtype=int)
    for row in range(grid.shape[0]):
        for column in range(grid.shape[1]):
            point = np.array([row,column])
            regions[row,column] = voronoi_owner(point,robot_positions)
    
    return regions

def detect_frontiers(grid: np.ndarray) -> list:
    """
    Return a list of (row, col) tuples -- every FREE cell that has at
    least one UNKNOWN cell as a direct (up/down/left/right) neighbor.
    Diagonal neighbors don't count; don't overthink this part.

    TODO(you): loop over every (row, col) in the grid. Skip it unless
    grid[row, col] == FREE. Then check its up to 4 neighbors -- watch
    grid edges, a cell on row 0 has no "up" neighbor, etc, so guard
    each neighbor check with a bounds check before indexing. If ANY
    valid neighbor is UNKNOWN, this cell is a frontier: append
    (row, col) to your results list.
    """

    frontiers = [ ]
    rows, cols = grid.shape

    for row in range(rows):
        for col in range(cols):
            if grid[row,col] != FREE:
                continue
            neighbours = [
                (row + 1, col),
                (row - 1, col),
                (row, col + 1),
                (row, col - 1)
            ]
            for nr , nc in neighbours:
                if 0 <= nr < rows and 0 <= nc < cols:
                    if grid[nr,nc] == UNKNOWN:
                        frontiers.append((row,col))
                        break

    return frontiers


def assign_frontiers(frontiers: list, robot_positions: np.ndarray) -> list:
    """
    Pair every frontier cell with its owning robot.

    TODO(you): for each (row, col) in `frontiers`, call
    voronoi_owner(np.array([row, col]), robot_positions) to get the
    owner index, and build a list of (row, col, owner_index) triples.
    """
    assignments = []

    for row,col in frontiers:
        point = np.array([row,col])
        owner = voronoi_owner(point,robot_positions)
        assignments.append((row,col,owner))

    return assignments


# ---------------------------------------------------------------------------
# Plotting -- plumbing, not the interesting part. Provided as-is.
# ---------------------------------------------------------------------------

def plot_result(grid, robot_positions, regions, frontiers_with_owners):
    n_robots = len(robot_positions)
    colors = plt.cm.tab10(np.linspace(0, 1, n_robots))

    fig, ax = plt.subplots(figsize=(8, 8))

    # Tint every cell by its Voronoi owner's color, but shade
    # OCCUPIED cells dark and UNKNOWN cells light so the raw map
    # structure stays visible underneath the region coloring.
    rgb = np.zeros((*grid.shape, 3))
    for r in range(grid.shape[0]):
        for c in range(grid.shape[1]):
            base = np.array(colors[regions[r, c]][:3])
            if grid[r, c] == OCCUPIED:
                rgb[r, c] = base * 0.2
            elif grid[r, c] == UNKNOWN:
                rgb[r, c] = base * 0.5 + 0.4
            else:  # FREE
                rgb[r, c] = base * 0.5 + 0.25
    ax.imshow(np.clip(rgb, 0, 1), origin='lower')

    for row, col, owner in frontiers_with_owners:
        ax.scatter(col, row, color=colors[owner], edgecolor='black', s=25, zorder=5)

    for i, (row, col) in enumerate(robot_positions):
        ax.scatter(col, row, color=colors[i], marker='*', s=300,
                    edgecolor='black', linewidth=1.5, zorder=6,
                    label=f'robot {i}')

    ax.set_title('Voronoi-partitioned exploration: frontiers colored by owning robot')
    ax.legend(loc='upper right')
    plt.savefig('cooperative_exploration.png')
    plt.close()
    print('Saved cooperative_exploration.png')


def main():
    grid = make_fake_map(GRID_SIZE)
    regions = voronoi_map(grid, ROBOT_POSITIONS)
    frontiers = detect_frontiers(grid)
    frontiers_with_owners = assign_frontiers(frontiers, ROBOT_POSITIONS)

    print(f'{len(frontiers)} frontier cells found')
    for i in range(len(ROBOT_POSITIONS)):
        count = sum(1 for _, _, owner in frontiers_with_owners if owner == i)
        print(f'  robot {i}: owns {count} frontier cells')

    plot_result(grid, ROBOT_POSITIONS, regions, frontiers_with_owners)


if __name__ == '__main__':
    main()
