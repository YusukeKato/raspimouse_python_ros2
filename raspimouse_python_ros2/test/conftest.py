"""Test setup."""

import multiprocessing

# The node and action tests start ROS / DDS threads in the pytest process.
# ament_flake8 runs flake8 with a multiprocessing pool, and forking such a
# multi-threaded process (the default before Python 3.14, e.g. Jazzy on
# Ubuntu 24.04) can crash the workers and hang the test run. Start workers as
# fresh interpreters instead.
multiprocessing.set_start_method('spawn', force=True)
