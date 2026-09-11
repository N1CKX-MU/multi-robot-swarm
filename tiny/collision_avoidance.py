#!/usr/bin/env python3
"""
Day 6 build-tiny: dynamic formation switching + inter-robot collision
avoidance (Artificial Potential Fields), standalone (no ROS).

This is also the foundation for the capstone convoy-escort demo: one
PAYLOAD robot follows a route of waypoints; 3 ESCORT robots hold a
protective formation around it that reshapes per route segment
(diamond in the open -> line for a chokepoint -> diamond again). APF
repulsion keeps any two robots from colliding, layered on top of the
formation-tracking motion.

Run:
    python3 collision_avoidance.py
"""
import numpy as np
import matplotlib.pyplot as plt

DT = 0.05
PAYLOAD_SPEED = 1.0
D_SAFE = 0.6            # APF: zero repulsion at or beyond this distance
WALL_D_SAFE = 0.6       # same idea, but for robot-vs-wall repulsion
APF_CLIP = 0.6         # cap on repulsion magnitude per axis (stops flinging)
FORMATION_GAIN = 2.0   # how hard escorts pull toward their formation slot
MAX_STEPS = 2000
EPS = 1e-6

N_ROBOTS = 4  # index 0 = payload, indices 1..3 = escorts

# Route the payload follows. `formation` is the shape the escorts hold
# while travelling TOWARD that waypoint.
MISSION = [
    {'waypoint': (0.0, 0.0),  'formation': 'diamond'},   # start
    {'waypoint': (3.0, 0.0),  'formation': 'diamond'},   # open ground
    # ^ was (6.0, 0.0) -- only 1 unit before the wall's x=7 start. With
    # FORMATION_GAIN=2.0's convergence rate, that wasn't enough lead time
    # to fully collapse into 'line' before entering the corridor, so
    # escorts arrived still partway-diamond (close enough to the wall's
    # WALL_D_SAFE=0.6 radius to trigger repulsion) and got stuck fighting
    # formation-pull vs wall-push -- a real APF local minimum. Moved the
    # trigger 3 units earlier so escorts have time to settle into line
    # well before the wall's repulsion field is even relevant.
    {'waypoint': (13.0, 0.0), 'formation': 'line'},       # chokepoint: single file
    # ^ was (9.0, 0.0) -- the middle of the corridor (walls span x=7..11).
    # The formation attached to THIS waypoint applies to the NEXT segment,
    # so having it end mid-corridor meant the diamond reshape fired while
    # escorts were still physically between the walls. First moved to
    # 11.5 (just past the exit), which reduced but didn't fully fix it --
    # an escort reshaping from y=0 back to y=1.2 right at x=11.5 still
    # grazed the wall's CORNER at (11, 0.7): Euclidean distance to a
    # corner is smaller than the x-only gap suggests. 13.0 gives enough
    # x-clearance that the diagonal distance to the corner also clears
    # WALL_D_SAFE by the time the reshape happens.
    {'waypoint': (15.0, 0.0), 'formation': 'diamond'},   # open again
    {'waypoint': (18.0, 3.0), 'formation': 'diamond'},   # target
]

# Escort offsets relative to the payload, per formation shape. Index 0
# (the payload) is (0,0); escorts are 1..3.
FORMATIONS = {
    'diamond': [(0.0, 0.0), (-1.2, 0.0), (0.0, 1.2), (0.0, -1.2)],
    'line':    [(0.0, 0.0), (-1.0, 0.0), (-2.0, 0.0), (-3.0, 0.0)],
}

# Walls marking the chokepoint. Escorts get pushed away from these via
# wall_repulsion() below, using the same APF formula as inter-robot
# repulsion -- this is what makes the diamond->line reshape actually
# NECESSARY rather than decorative: without it (or in the wrong shape),
# escorts would get shoved off their formation slot trying to fit
# through the gap. The payload itself is NOT wall-repelled; its route
# is authored to go straight down the corridor centerline already.
CHOKEPOINT_WALLS = [
    ((7.0, 0.7), (11.0, 2.5)),    # (x0, y0), (x1, y1) upper wall
    ((7.0, -2.5), (11.0, -0.7)),  # lower wall
]


def rotation_matrix(theta: float) -> np.ndarray:
    """From Day 3 -- provided, you already derived this by hand."""
    return np.array([[np.cos(theta), -np.sin(theta)],
                     [np.sin(theta),  np.cos(theta)]])


def repulsion(positions: np.ndarray, robot_idx: int) -> np.ndarray:
    """
    APF: total repulsive velocity pushing robot_idx away from every
    OTHER robot within D_SAFE. This is the formula you worked out by
    hand: strength = (1/dist - 1/D_SAFE) / dist**2, in the direction
    pointing away from the other robot.

    TODO(you):
      1. push = np.zeros(2)
      2. For every other robot j (j != robot_idx):
           diff = positions[robot_idx] - positions[j]   # points AWAY from j
           dist = np.linalg.norm(diff)
           if EPS < dist < D_SAFE:
               strength = (1.0 / dist - 1.0 / D_SAFE) / (dist ** 2)
               push += strength * (diff / dist)         # unit vector * strength
         (the dist > EPS guard avoids divide-by-zero if two robots are
         exactly on top of each other -- a degenerate case, just skip it)
      3. Return np.clip(push, -APF_CLIP, APF_CLIP).
    """
    push = np.zeros(2)
    for j in range(len(positions)):
        if j == robot_idx:
            continue
        diff = positions[robot_idx] - positions[j]
        dist = np.linalg.norm(diff)
        if EPS < dist < D_SAFE:
            strength = (1.0 / dist - 1.0 / D_SAFE) / (dist ** 2)
            push += strength * (diff / dist)
    return np.clip(push, -APF_CLIP, APF_CLIP)


def closest_point_on_wall(position: np.ndarray, wall) -> np.ndarray:
    """
    Return the closest point ON or IN the axis-aligned wall rectangle
    to `position`. Clamping each coordinate to the rectangle's
    [x0, x1] x [y0, y1] range gives exactly that -- if `position` is
    already inside, clamping is a no-op and the "closest point" comes
    back equal to `position` itself (distance 0), which is the signal
    wall_repulsion() below uses to detect the degenerate inside-a-wall
    case.
    """
    (x0, y0), (x1, y1) = wall
    return np.array([np.clip(position[0], x0, x1), np.clip(position[1], y0, y1)])


def wall_repulsion(position: np.ndarray) -> np.ndarray:
    """
    Same APF idea as `repulsion`, but pushing away from each wall in
    CHOKEPOINT_WALLS instead of away from other robots -- the closest
    point on a wall's rectangle stands in for "the other object's
    position" in the exact same formula.

    One case `repulsion` never has to handle: a robot can end up
    ALREADY INSIDE a wall's rectangle (as you spotted happening before
    this function existed). There, closest_point_on_wall returns the
    robot's own position, so dist=0 and the normal formula would
    divide by zero. Handled separately: push straight out through
    whichever of the rectangle's four faces is nearest (the smallest
    penetration depth), at full APF_CLIP strength -- there's no
    well-defined "distance-based" push once you're already inside, so
    just get out via the shortest path.
    """
    push = np.zeros(2)
    for wall in CHOKEPOINT_WALLS:
        (x0, y0), (x1, y1) = wall
        closest = closest_point_on_wall(position, wall)
        diff = position - closest
        dist = np.linalg.norm(diff)

        if dist < EPS:
            depth_to_face = {
                'left':   (position[0] - x0, np.array([-1.0, 0.0])),
                'right':  (x1 - position[0], np.array([1.0, 0.0])),
                'bottom': (position[1] - y0, np.array([0.0, -1.0])),
                'top':    (y1 - position[1], np.array([0.0, 1.0])),
            }
            _, direction = min(depth_to_face.values(), key=lambda pair: pair[0])
            push += direction * APF_CLIP
        elif dist < WALL_D_SAFE:
            strength = (1.0 / dist - 1.0 / WALL_D_SAFE) / (dist ** 2)
            push += strength * (diff / dist)

    return np.clip(push, -APF_CLIP, APF_CLIP)


def formation_velocity(positions: np.ndarray, robot_idx: int,
                        payload_pos: np.ndarray, heading: float,
                        offsets: np.ndarray) -> np.ndarray:
    """
    The "get to your assigned slot" velocity for one escort: a vector
    from where the robot IS toward where its formation slot IS.

    TODO(you):
      1. slot = payload_pos + rotation_matrix(heading) @ offsets[robot_idx]
         (exactly Day 3's goal = center + R(theta) @ offset)
      2. return FORMATION_GAIN * (slot - positions[robot_idx])
    """
    slot = payload_pos + rotation_matrix(heading) @ offsets[robot_idx]
    return FORMATION_GAIN * (slot - positions[robot_idx])


def step_payload_along_route(payload_pos: np.ndarray, wp_index: int,
                             speed: float, dt: float):
    """
    Move the payload toward MISSION[wp_index]['waypoint']. If it reaches
    that waypoint this step, advance wp_index.

    Returns (new_payload_pos, new_wp_index, heading, done).

    TODO(you):
      1. target = np.array(MISSION[wp_index]['waypoint'], dtype=float)
      2. direction = target - payload_pos ; dist = np.linalg.norm(direction)
      3. heading = np.arctan2(direction[1], direction[0])   # direction of travel
      4. If dist <= speed * dt:
           payload_pos = target.copy()
           wp_index += 1
           done = wp_index >= len(MISSION)
         Else:
           payload_pos = payload_pos + speed * dt * (direction / dist)
           done = False
      5. Return (payload_pos, wp_index, heading, done)
    """
    target = np.array(MISSION[wp_index]['waypoint'], dtype=float)
    direction = target - payload_pos
    dist = np.linalg.norm(direction)
    heading = np.arctan2(direction[1], direction[0])
    if dist <= speed * dt:
        payload_pos = target.copy()
        wp_index += 1
        done = wp_index >= len(MISSION)
    else:
        payload_pos = payload_pos + speed * dt * (direction / dist)
        done = False
    return payload_pos, wp_index, heading, done

def run_mission():
    """
    Run the full convoy sim. Returns:
      history            -- (steps, N_ROBOTS, 2) positions over time
      min_distances      -- (steps,) smallest distance between ANY two
                             robots at each step (inter-robot APF metric)
      min_wall_distances -- (steps,) smallest distance from ANY escort
                             to the nearest wall at each step (wall APF
                             metric -- 0 means an escort was inside a
                             wall's rectangle that step)

    TODO(you):
      1. payload_pos = np.array(MISSION[0]['waypoint'], dtype=float)
      2. wp_index = 1  (waypoint 0 is the start; head for waypoint 1)
      3. Initialise escort positions at their diamond slots around the
         payload, plus small random noise so APF has work to do early:
             offsets0 = np.array(FORMATIONS['diamond'])
             positions = np.zeros((N_ROBOTS, 2))
             positions[0] = payload_pos
             for i in 1..N_ROBOTS-1:
                 positions[i] = payload_pos + offsets0[i] + np.random.uniform(-0.2, 0.2, 2)
      4. history, min_distances = [], []
      5. Loop up to MAX_STEPS:
           a. payload_pos, wp_index, heading, done = step_payload_along_route(...)
           b. positions[0] = payload_pos   (payload follows the route exactly)
           c. shape for this segment: MISSION[min(wp_index, len(MISSION)-1)]['formation']
              offsets = np.array(FORMATIONS[shape])
           d. For each escort i in 1..N_ROBOTS-1:
                v = formation_velocity(positions, i, payload_pos, heading, offsets)
                v += repulsion(positions, i)
                positions[i] = positions[i] + v * dt
           e. Record positions.copy() into history.
           f. Record the min pairwise distance: for all i<j,
              min(np.linalg.norm(positions[i] - positions[j])).
           g. if done: break
      6. Return np.array(history), np.array(min_distances)
    """
    payload_pos = np.array(MISSION[0]['waypoint'], dtype=float)
    wp_index = 1
    offsets0 = np.array(FORMATIONS['diamond'])
    positions = np.zeros((N_ROBOTS, 2))
    positions[0] = payload_pos
    for i in range(1, N_ROBOTS):
        positions[i] = payload_pos + offsets0[i] + np.random.uniform(-0.2, 0.2, 2)

    history, min_distances, min_wall_distances = [], [], []
    for step in range(MAX_STEPS):
        payload_pos, wp_index, heading, done = step_payload_along_route(
            payload_pos, wp_index, PAYLOAD_SPEED, DT)
        positions[0] = payload_pos
        shape = MISSION[min(wp_index, len(MISSION)-1)]['formation']
        offsets = np.array(FORMATIONS[shape])
        for i in range(1, N_ROBOTS):
            v = formation_velocity(positions, i, payload_pos, heading, offsets)
            v += repulsion(positions, i)
            v += wall_repulsion(positions[i])
            positions[i] += v * DT
        history.append(positions.copy())
        min_distances.append(np.min([np.linalg.norm(positions[i] - positions[j])
                                      for i in range(N_ROBOTS) for j in range(i+1, N_ROBOTS)]))
        min_wall_distances.append(np.min([
            np.linalg.norm(positions[i] - closest_point_on_wall(positions[i], wall))
            for i in range(1, N_ROBOTS) for wall in CHOKEPOINT_WALLS
        ]))
        if done:
            break
    return np.array(history), np.array(min_distances), np.array(min_wall_distances)

# ---------------------------------------------------------------------------
# Plotting -- plumbing, not the interesting part. Provided as-is.
# ---------------------------------------------------------------------------

def plot_mission(history: np.ndarray, min_distances: np.ndarray,
                  min_wall_distances: np.ndarray):
    colors = plt.cm.tab10(np.linspace(0, 1, N_ROBOTS))
    labels = ['payload'] + [f'escort {i}' for i in range(1, N_ROBOTS)]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    for (x0, y0), (x1, y1) in CHOKEPOINT_WALLS:
        ax1.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0,
                                     color='0.3', zorder=1))

    for i in range(N_ROBOTS):
        ax1.plot(history[:, i, 0], history[:, i, 1], color=colors[i],
                  label=labels[i], linewidth=1.5 if i == 0 else 1)
    for m in MISSION:
        ax1.scatter(*m['waypoint'], marker='x', color='black', zorder=5)
    ax1.set_title('Convoy escort: payload route + escort formation')
    ax1.set_xlabel('x')
    ax1.set_ylabel('y')
    ax1.legend()
    ax1.axis('equal')

    ax2.plot(min_distances, color='tab:red', label='min inter-robot distance')
    ax2.plot(min_wall_distances, color='tab:orange', label='min escort-to-wall distance')
    ax2.axhline(D_SAFE, color='gray', linestyle='--', label=f'D_SAFE = {D_SAFE}')
    ax2.axhline(0, color='black', linewidth=0.8)
    ax2.set_title('Minimum clearance over time')
    ax2.set_xlabel('timestep')
    ax2.set_ylabel('distance')
    ax2.set_ylim(bottom=0)
    ax2.legend()

    plt.tight_layout()
    plt.savefig('collision_avoidance.png')
    plt.close()
    print('Saved collision_avoidance.png')


def main():
    history, min_distances, min_wall_distances = run_mission()
    print(f'{len(history)} steps')
    print(f'closest any two robots ever got: {min_distances.min():.3f} '
          f'(D_SAFE = {D_SAFE})')
    print(f'closest any escort got to a wall: {min_wall_distances.min():.3f} '
          f'(WALL_D_SAFE = {WALL_D_SAFE}, 0.0 would mean inside a wall)')
    plot_mission(history, min_distances, min_wall_distances)


if __name__ == '__main__':
    main()
