#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2020, KU Leuven
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of KU Leuven nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Jeroen De Maeyer

from __future__ import print_function
from six.moves import input  # Python 3 compatible alternative for raw_input

from ompl_constrained_planning_tutorial import ConstrainedPlanningTutorial


def solve(move_group, start_state, pose_goal, path_constraints):
    """ Convenience function not used by the main tutorial """
    move_group.set_start_state(start_state)
    move_group.set_pose_target(pose_goal)

    # Don't forget the path constraints! That's the whole point of this tutorial.
    move_group.set_path_constraints(path_constraints)

    # And let the planner find a solution.
    # The move_group node should automatically visualize the solution in Rviz if a path is found.
    move_group.plan()

    # Clear the path constraints for our next experiment
    move_group.clear_path_constraints()


def run_vertical_plane_example():
    """ Run an example where we want to keep the end-effector on a vertical plane. """
    tutorial = ConstrainedPlanningTutorial()
    tutorial.remove_all_markers()

    tutorial.add_obstacle()

    start_state = tutorial.create_start_state()
    pose_goal = tutorial.create_pose_goal_under_obstacle()
    pcm = tutorial.create_vertical_plane_constraints()

    # We need two wrap the constraints in a generic `Constraints` message.
    path_constraints = moveit_msgs.msg.Constraints()
    path_constraints.position_constraints.append(pcm)
    path_constraints.name = "use_equality_constraints"

    tutorial.solve(start_state, pose_goal, path_constraints)

    print("============ Press enter to continue with the second planning problem.")
    input()
    tutorial.remove_all_markers()
    tutorial.remove_obstacle()
    print("Done!")


def main():
    """ Catch interupt when the user presses `ctrl-c`. """
    try:
        run_vertical_plane_example()
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
