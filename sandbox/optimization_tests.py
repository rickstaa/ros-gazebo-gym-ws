"""Small timeit script I used to test several optimizations."""
import timeit


if __name__ == "__main__":

    # Test dictionary pre-initialization
    setup_non_initialized = """
import numpy as np
action_space_joints = [
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7"
]
action = np.array([1.2, 1.4, 3.4, 6.4, 3.2, 4.5, 6.9])
    """
    not_initialized = """
test = dict(zip(action_space_joints, action.copy()))
    """
    initialized = """
test = {key: val for key, val in zip(action_space_joints, action.copy())}
    """

    # Preinitialize dictionary
    print(
        "not_initialized        ",
        timeit.Timer(not_initialized, setup_non_initialized).timeit(1000),
    )
    print(
        "initialized        ",
        timeit.Timer(initialized, setup_non_initialized).timeit(1000),
    )
