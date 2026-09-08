#!/usr/bin/env python3
"""
Day 3 build-tiny: formation control, standalone (no ROS, no Gazebo).

A "leader" follows a sine-wave path. Three "follower" robots hold a
rotating formation around the leader -- rotating because the formation
must track the leader's HEADING, not just its position (you derived
this by hand: goal = center + R(theta) @ offset). Partway through the
run, the formation shape switches (square -> diamond -> line) to watch
a live transition.

Run:
    python3 formation_control.py
"""
import numpy as np
import matplotlib.pyplot as plt


N_STEPS = 200
DT = 0.05          # time step, seconds
LEADER_SPEED = 1.0  # leader's forward speed, world units/sec

# Formation offsets, same shapes as the plan. Index 0 is always the
# leader itself (offset (0,0) -- it doesn't offset from itself).
FORMATIONS = {
    'square': [(0.0, 0.0), (-0.75, -0.75), (0.75, -0.75), (0.0, 1.0)],
    'diamond': [(0.0, 0.0), (0.0, -1.0), (1.0, 0.0), (-1.0, 0.0)],
    'line': [(0.0, 0.0), (-1.5, 0.0), (-3.0, 0.0), (-4.5, 0.0)],
}


def rotation_matrix(theta: float) -> np.ndarray:
    """
    Return the 2x2 rotation matrix for angle theta (radians).

    TODO(you): you already computed this by hand for theta=90 degrees.
    Generalize it:
        R(theta) = [[cos(theta), -sin(theta)],
                    [sin(theta),  cos(theta)]]
    np.cos / np.sin take radians, not degrees -- careful if you test
    against your hand computation, which used 90 degrees = pi/2 radians.
    """
    return np.array([
        [np.cos(theta),-np.sin(theta)],
        [np.sin(theta),np.cos(theta)]
        ])

def leader_path(t: float):
    """
    Return (position, heading) of the leader at time t.

    Position: a sine wave -- moves along x at LEADER_SPEED, while y
    oscillates as a sine function of x. Something like:
        x = LEADER_SPEED * t
        y = 2.0 * sin(0.5 * x)

    Heading: the direction the leader is actually FACING, which for a
    path is the direction of its VELOCITY vector, not just "wherever
    it happens to be." That means you need the derivative of the path.

    TODO(you):
      1. Compute x, y from t using the formulas above.
      2. Compute the leader's velocity (dx/dt, dy/dt) analytically --
         dx/dt is just LEADER_SPEED (constant). dy/dt needs the chain
         rule on y = 2*sin(0.5*x): dy/dt = 2*cos(0.5*x)*0.5*(dx/dt).
      3. heading = atan2(dy/dt, dx/dt) -- np.arctan2(vy, vx) gives you
         the angle of the velocity vector, which IS the heading (a
         robot facing its direction of travel).
      4. Return (np.array([x, y]), heading).
    """
    x = LEADER_SPEED * t
    y = 2.0 * np.sin(0.5 * x)

    dx_dt = LEADER_SPEED
    dy_dt = 2.0 * np.cos(0.5 * x) * 0.5 * dx_dt

    heading = np.arctan2(dy_dt,dx_dt)

    return np.array([x,y]),heading

def formation_goals(center: np.ndarray, heading: float, offsets: list) -> np.ndarray:
    """
    Given the formation center, the heading to rotate by, and a list
    of (x, y) offsets, return the actual world-frame goal position for
    every robot.

    TODO(you): this is EXACTLY the hand computation you just did,
    generalized to N robots and vectorized. For each offset in
    `offsets`:
        goal_i = center + R(heading) @ offset_i
    You can do this with a python loop (totally fine), or vectorize by
    stacking offsets into an (N,2) array and doing
    (R(heading) @ offsets.T).T + center -- try the loop first if the
    vectorized version isn't obvious, get it working, THEN vectorize
    if you want the practice.
    """
    R = rotation_matrix(heading)

    goals = []

    for offset in offsets:
        offset = np.array(offset)
        goal = center + R @ offset
        goals.append(goal)

    return np.array(goals)

def run_formation(n_steps: int, dt: float) -> np.ndarray:
    """
    Simulate the whole run and return the full trajectory history for
    all 4 robots (leader + 3 followers).

    TODO(you):
      1. history = []
      2. For each step i in range(n_steps):
           - t = i * dt
           - center, heading = leader_path(t)
           - pick which formation shape is active (see below)
           - goals = formation_goals(center, heading, FORMATIONS[shape])
           - append goals to history
             (goals[0] IS the leader's own position -- since its
             offset is (0,0), formation_goals already handles it
             correctly without special-casing the leader anywhere)
      3. Return np.array(history) -- shape (n_steps, 4, 2)

      Formation switching: divide n_steps into three equal chunks --
      first third 'square', middle third 'diamond', last third 'line'.
      (i / n_steps) < 1/3 -> square, < 2/3 -> diamond, else line is one
      way to write that check.

      Note: this version SNAPS instantly between shapes at each
      boundary (no smoothing) -- that's fine for build-tiny. Smoothly
      blending between formations is a nice optional extension once
      the basic version works, not required.
    """
    history = []

    for i in range(n_steps):
        t = i*dt
        center,heading = leader_path(t)

        fraction = i / n_steps

        if fraction < 1/3:
            shape = "square"
        elif fraction < 2/3:
            shape = "diamond"
        else:
            shape = "line"

        goals = formation_goals(
            center,
            heading,
            FORMATIONS[shape]
        )

        history.append(goals)

    return np.array(history)


def plot_formation_run(history: np.ndarray):
    """
    Plot the leader's path and all 3 followers' paths, provided for
    you. history shape: (n_steps, 4, 2).
    """
    labels = ['leader', 'follower 1', 'follower 2', 'follower 3']
    plt.figure(figsize=(8, 6))
    for i in range(history.shape[1]):
        plt.plot(history[:, i, 0], history[:, i, 1], label=labels[i], linewidth=1)
    # Mark the formation-switch boundaries for visual reference
    n = history.shape[0]
    for frac in (1 / 3, 2 / 3):
        idx = int(n * frac)
        plt.scatter(history[idx, :, 0], history[idx, :, 1], marker='x', color='black', zorder=5)
    plt.title('Formation control: square -> diamond -> line')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()
    plt.axis('equal')
    plt.savefig('formation_run.png')
    plt.close()
    print('Saved formation_run.png')


def main():
    history = run_formation(N_STEPS, DT)
    plot_formation_run(history)


if __name__ == '__main__':
    main()
