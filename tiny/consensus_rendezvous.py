#!/usr/bin/env python3
"""
Day 2 build-tiny: consensus rendezvous, standalone (no ROS, no Gazebo).

Simulates N robots converging to a common point via the consensus
protocol, across different communication graph topologies. Get this
right in plain numpy first -- if it works here, the math is right, and
any bug you hit later inside ROS2 is an integration bug, not an
algorithm bug.

Run:
    python3 consensus_rendezvous.py
"""
import numpy as np
import matplotlib.pyplot as plt

# Starting positions -- the plan's example corners.
START_POSITIONS = np.array([[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]])
N_STEPS = 50
EPSILON = 0.6


def build_adjacency(n: int, topology: str) -> np.ndarray:
    """
    Build the n x n adjacency matrix for the given topology.
    Supported: 'ring', 'complete', 'star', 'line'.

    """

    A = np.zeros((n,n),dtype=float)

    if topology == "ring":
        for i in range(n):
            A[i, (i-1) % n] = 1
            A[i,(i+1) % n] =1

    elif topology == "complete":
        for i in range(n):
            for j in range(n):
                if i != j:
                    A[i,j] = 1

    elif topology == "star":

        for i in range(1,n):
            A[0,i] = 1
            A[i,0] = 1

    elif topology == "line":
        for i in range(n):
            if i > 0:
                A[i,i - 1] = 1

            if i < n - 1:
                A[i, i+1] = 1   
    else:
        raise ValueError(
            f"Unknown topology: {topology}."
            f"Use 'ring', 'complete', 'star', 'line'."
        )
    
    return A

def laplacian(A: np.ndarray) -> np.ndarray:
    """
    L = D - A, where D is diagonal and D[i,i] = degree of robot i
    = sum of row i in A (same definition you used by hand).

    """
    D = np.diag(A.sum(axis = 1))

    return D - A


def fiedler_value(L: np.ndarray) -> float:
    """
    Return the second-smallest eigenvalue of L.
    """

    eigenvalues = np.linalg.eigvalsh(L)

    return eigenvalues[1]


def run_consensus(A: np.ndarray, epsilon: float, n_steps: int) -> np.ndarray:
    """
    Run the consensus protocol for n_steps and return the FULL
    trajectory history (every intermediate position, not just the
    final one -- you need this for both plots below).

    """

    positions = START_POSITIONS.copy()

    history = [positions.copy()]

    L = laplacian(A)

    for _ in range(n_steps):
        positions = positions - epsilon * (L @ positions)
        history.append(positions.copy())
    return np.array(history)

def max_dist_from_centroid(history: np.ndarray) -> np.ndarray:
    """
    For each timestep, the max distance of any robot from the centroid
    (mean position of all robots) at that timestep. Returns a 1D array
    of length n_steps+1 -- this is what the convergence-comparison plot
    uses.
    """

    max_distances = []

    for positions in history:

        centroid = positions.mean(axis=0)

        distances = np.linalg.norm(positions - centroid, axis = 1)

        max_distances.append(distances.max())

    return np.array(max_distances)


# ---------------------------------------------------------------------------
# Plotting -- plumbing, not the interesting part. Provided as-is.
# ---------------------------------------------------------------------------

def plot_single_run(history: np.ndarray, topology: str):
    """Plot all robots' (x, y) trajectories over time for one topology."""
    n = history.shape[1]
    plt.figure()
    for i in range(n):
        plt.plot(history[:, i, 0], history[:, i, 1],
                  marker='o', markersize=2, label=f'robot {i}')
    plt.title(f'Consensus rendezvous -- {topology} topology')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()
    plt.axis('equal')
    plt.savefig(f'consensus_{topology}.png')
    plt.close()
    print(f'Saved consensus_{topology}.png')


def plot_convergence_comparison(results: dict):
    """
    results: {topology_name: max_dist_array}
    Overlay all topologies' convergence curves on one plot -- this is
    the plan's Day 2 deliverable, and the empirical proof of the
    Fiedler-value ranking you derived by hand (complete fastest, line
    slowest).
    """
    plt.figure()
    for topology, max_dist in results.items():
        plt.plot(max_dist, label=topology)
    plt.xlabel('timestep')
    plt.ylabel('max distance from centroid')
    plt.title('Convergence speed by topology')
    plt.legend()
    plt.savefig('convergence_comparison.png')
    plt.close()
    print('Saved convergence_comparison.png')


def main():
    n = 4
    topologies = ['ring', 'complete', 'star', 'line']
    results = {}

    for topology in topologies:
        A = build_adjacency(n, topology)
        L = laplacian(A)
        fv = fiedler_value(L)
        print(f'{topology}: Fiedler value = {fv:.4f}')

        history = run_consensus(A, EPSILON, N_STEPS)
        plot_single_run(history, topology)
        results[topology] = max_dist_from_centroid(history)

    plot_convergence_comparison(results)

    # -------------------------------------------------------------------
    # Once everything above works, the plan wants you to experiment:
    #   - Try EPSILON = 0.6 on the ring or star. Watch it overshoot and
    #     oscillate instead of converging smoothly. Why? Because epsilon
    #     needs to stay under 1 / max_degree for stability -- you can
    #     derive that bound yourself by thinking about what happens to
    #     the update rule when a robot has many neighbors all pulling
    #     at once with too strong a gain.
    #   - Try n=8 or n=16 robots. Does the Fiedler-value ranking between
    #     topologies still hold?
    # Not required to touch main() for this -- just call the functions
    # above with different arguments from a scratch script or a REPL.
    # -------------------------------------------------------------------


if __name__ == '__main__':
    main()
