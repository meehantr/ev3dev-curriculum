#!/usr/bin/env python3
"""
This module lets you practice EV3 drive motor commands using timed delays.

Use input prompts to get the target speed (degrees per second) and desired number of inches from the user.
Then make the EV3 drive a given number of inches at the target speed.  You will need to do some experiments
to collect data, then make an equation to figure out the time needed given the distance and speed.

Note: If future modules you will learn different (BETTER) ways to drive a given distance.

Authors: David Fisher and Thomas Meehan.
"""  # done: 1. PUT YOUR NAME IN THE ABOVE LINE.

# done: 2. Get a yardstick or tape measure to do some testing with
# /examples/motors/drive_input_speed.py
#   Have your whole team work this activity together.
#   For your first test run drive_input_speed (via python drive_input_speed from that folder using SSH).
#   Enter these values for your first test and record the distance below as XXX for 100.
#     Enter a speed for the left motor (0 to 900 dps): 100
#     Enter a speed for the right motor (0 to 900 dps): 100
#     Enter a time to drive (seconds): 10
#
#   For speeds 100, 200, 300, 400 record how many inches the robot moves in 10 seconds, then divide that number by 10 to
#     get the speed of the robot in inches per second.  Record those value below.
#   For example if you used the drive_input_speed.py example to do a test at 200 degrees per second (dps) for 10 seconds
#     and it went 30 inches, then 200 dps = 3 inches per second, which you would record below.
#   For speeds 500 to 900 use only 5 seconds for your test to avoid driving too far.  Just divide the distance by 5 to
#     get the number of inches traveled in a single second.  Note, you 'could' do 1 second, but doing 5 or 10 then
#     dividing by 5 or 10 is a much more accurate average speed test.
#
#  Record your calculated speed conversions here:
#   Tests @ 10 seconds:
#     100 degrees / second  -->  traveled 12 inches  -->  1.2 inches / second
#     200 degrees / second  -->  traveled 22.5 inches  -->  2.25 inches /
#     second
#     300 degrees / second  -->  traveled 31.5 inches  -->  3.15 inches /
#     second
#     400 degrees / second  -->  traveled 42 inches  -->  4.2 inches / second
#   Tests @ 5 seconds:
#     500 degrees / second  -->  traveled 25.75 inches  -->  5.15 inches /
#     second
#     600 degrees / second  -->  traveled 32 inches  -->  6.4 inches / second
#     700 degrees / second  -->  traveled 37 inches  -->  7.4 inches / second
#     800 degrees / second  -->  traveled 41 inches  -->  8.2 inches / second
#     900 degrees / second  -->  traveled 41 inches  -->  8.2 inches / second
#  (probably no faster than 800)
#
# done: 3. Make an equation
#   Derive from that information a way to convert a given degrees per second speed into an inches / second speed.
#     If you plotted the data with degrees / second on the x axis and inches per second on the y axis you would find the
#       data is fairly linear, so you could use a    y = m * x + b   line approximation formula.  Excel could even help
#       make the value for m and b perfect, but that is overkill.  You have permission to set b = 0 and just pick an m
#       that would roughly fit most of your data.  Put your value for m below and think about if it most fits:
#
#       m = .0094
#       speed_in_inches_per_second = m * speed_in_degrees_per_second + 0
#
#     Eventually your goal is to make an equation that will allow users to input any distance in inches and any speed in
#     degrees per second, then output the time needed to drive the correct distance at that speed.  So eventually you
#     will be making a formula like this...
#
#   Time (seconds) = Distance (inches, as input from the user) / Speed (inches/second, as converted based on user input)
#
#   Note: To repeat again, in later modules you will learn different (better) ways to travel a given distance using
#     motor encoders, so just make a simple rough approximation here, since later we'll do it better in a different way.
#
# TODO: 3. Copy the content of the /examples/motors/drive_input_speed.py program and place it below these comments.
#   Change the initial print and speak commands to reflect this module, like this...
#    print("--------------------------------------------")
#    print("  Timed Driving")
#    print("--------------------------------------------")
#    ev3.Sound.speak("Timed Driving").wait()

import ev3dev.ev3 as ev3
import time


def main():
    print("--------------------------------------------")
    print("  Timed Driving")
    print("--------------------------------------------")
    ev3.Sound.speak("Timed Driving").wait()

    # Connect two large motors on output ports B and C
    left_motor = ev3.LargeMotor(ev3.OUTPUT_B)
    right_motor = ev3.LargeMotor(ev3.OUTPUT_C)

    # Check that the motors are actually connected
    assert left_motor.connected
    assert right_motor.connected

    time_s = 1  # Any value other than 0.
    while time_s != 0:
        left_sp = int(
            input("Enter a speed (0 to 900 dps): "))
        right_sp = left_sp
        dist_to_travel = int(input("Enter a distance to drive (inches): "))
        if left_sp == 0 or dist_to_travel == 0:
            break
        left_motor.run_forever(speed_sp=left_sp)
        right_motor.run_forever(speed_sp=right_sp)
        time_s = dist_to_travel / (left_sp * .0094)
        time.sleep(time_s)
        left_motor.stop()
        right_motor.stop(stop_action="brake")

    print("Goodbye!")
    ev3.Sound.speak("Goodbye").wait()


# ----------------------------------------------------------------------
# Calls  main  to start the ball rolling.
# ----------------------------------------------------------------------
main()

# done: 4. Change the input questions from:
#   Enter a speed for the left motor (0 to 900 dps):
#   Enter a speed for the right motor (0 to 900 dps):
#   Enter a time to drive (seconds):
# to:
#   Enter a speed (0 to 900 dps):
#   Distance to travel (inches):
# done: 5. Write the code necessary to make the robot drive at that speed going
#  roughly that distance.
#   Note, in this module, you are REQUIRED to use the pattern...
#      run_forever()
#      time.sleep(some_amount)
#      stop()
#   You may NOT use the advanced motor commands at this time like: run_to_abs_pos, run_to_rel_pos, or run_timed.
# done: 6. Modify the program so that it will exit immediately if the answer to
#    any   question is 0.
# done: 7. Formally test your work. When you think you have the problem
# complete run these tests to be sure:
#   200 dps 24 inches (make sure it drives within 6 inches of the target distance)
#   400 dps 24 inches (make sure it drives within 6 inches of the target distance)
#   800 dps 24 inches (make sure it drives within 6 inches of the target distance)
#   400 dps 12 inches (make sure it drives within 3 inches of the target distance)
#   400 dps 36 inches (make sure it drives within 9 inches of the target distance)
# Do more tests if you see fit.  Ideally you should be +/- 25% of the target goal.
#
# done: 8. Call over a TA or instructor to sign your team's checkoff sheet and
# do a code review.
#
#  Observation you should make, the pattern run_forever-->time.sleep-->stop naturally blocks code execution until done.
