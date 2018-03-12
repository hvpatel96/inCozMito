#!/usr/bin/env python3

# Copyright (c) 2016 Anki, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License in the file LICENSE.txt or at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

'''Make Cozmo drive in a square.

This script combines the two previous examples (02_drive_and_turn.py and
03_count.py) to make Cozmo drive in a square by going forward and turning
left 4 times in a row.
'''

import cozmo, time
from cozmo.util import degrees, distance_mm, speed_mmps


def cozmo_program(robot: cozmo.robot.Robot):
    # Use a "for loop" to repeat the indented code 4 times
    # Note: the _ variable name can be used when you don't need the value
    # for _ in range(4):
    #     robot.drive_straight(distance_mm(150), speed_mmps(50)).wait_for_completed()
    #     robot.turn_in_place(degrees(90)).wait_for_completed()
    # count = 0
    # while True:
    #     print(count)
    #     if count <= 52:
    #         robot.drive_wheels(10, 25, duration=1)
    #         time.sleep(0.05)
    #         count += 1
    #     elif count > 50:
    #         robot.drive_wheels(25, 10, duration=1)
    #         time.sleep(0.05)
    #         count += 1
    #     if count == 100:

    #         break
    robot.play_anim_trigger(cozmo.anim.Triggers.BouncerGetIn).wait_for_completed()

cozmo.run_program(cozmo_program)
