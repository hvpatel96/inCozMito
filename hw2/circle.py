import cozmo
import time

def cozmo_program(robot: cozmo.robot.Robot):
    # robot.say_text("I'm a gangstah robot!").wait_for_completed()
    # robot.move_head(-5)
    # Tell Cozmo to drive the left wheel at 25 mmps (millimeters per second),
    # and the right wheel at 50 mmps (so Cozmo will drive Forwards while also
    # turning to the left
    robot.drive_wheels(20, 80)

    # wait for 3 seconds (the head, lift and wheels will move while we wait)
    time.sleep(10)


cozmo.run_program(cozmo_program)