"""
  Library of EV3 robot functions that are useful in many different applications. For example things
  like arm_up, arm_down, driving around, or doing things with the Pixy camera.

  Add commands as needed to support the features you'd like to implement.  For organizational
  purposes try to only write methods into this library that are NOT specific to one tasks, but
  rather methods that would be useful regardless of the activity.  For example, don't make
  a connection to the remote control that sends the arm up if the ir remote control up button
  is pressed.  That's a specific input --> output task.  Maybe some other task would want to use
  the IR remote up button for something different.  Instead just make a method called arm_up that
  could be called.  That way it's a generic action that could be used in any task.
"""

import ev3dev.ev3 as ev3
import math
import time
import mqtt_remote_method_calls as com


class Snatch3r(object):
    """Commands for the Snatch3r robot that might be useful in many different programs."""

    def __init__(self):
        self.color_sensor = ev3.ColorSensor()
        assert self.color_sensor
        self.touch_sensor = ev3.TouchSensor()
        assert self.touch_sensor
        self.ir_sensor = ev3.InfraredSensor()
        assert self.ir_sensor
        self.pixy = ev3.Sensor(driver_name="pixy-lego")
        assert self.pixy
        self.left_motor = ev3.LargeMotor(ev3.OUTPUT_B)
        self.right_motor = ev3.LargeMotor(ev3.OUTPUT_C)

    def loop_forever(self):
        self.running = True
        while self.running:
            time.sleep(0.1)

    def drive_inches(self, inches_to_target, speed_in_dps):
        left_motor = ev3.LargeMotor(ev3.OUTPUT_B)
        right_motor = ev3.LargeMotor(ev3.OUTPUT_C)

        degrees_per_inch = 90
        position_sp = inches_to_target * degrees_per_inch

        left_motor.run_to_rel_pos(position_sp=position_sp,
                                  speed_sp=speed_in_dps)
        right_motor.run_to_rel_pos(position_sp=position_sp, speed_sp=
        speed_in_dps)

        left_motor.wait_while(ev3.Motor.STATE_RUNNING)

        ev3.Sound.beep().wait()

    def turn_degrees(self, degrees_to_turn, turn_speed):
        left_motor = ev3.LargeMotor(ev3.OUTPUT_B)
        right_motor = ev3.LargeMotor(ev3.OUTPUT_C)
        value_for_motors = degrees_to_turn * 5.3349
        left_motor.run_to_rel_pos(position_sp=value_for_motors,
                                  speed_sp=turn_speed)
        right_motor.run_to_rel_pos(position_sp=-value_for_motors,
                                   speed_sp=turn_speed)

        left_motor.wait_while(ev3.Motor.STATE_RUNNING)

        ev3.Sound.beep().wait()

    def arm_calibration(self):
        """
        Runs the arm up until the touch sensor is hit then back to the bottom again, beeping at both locations.
        Once back at in the bottom position, gripper open, set the absolute encoder position to 0.  You are calibrated!
        The Snatch3r arm needs to move 14.2 revolutions to travel from the touch sensor to the open position.

        Type hints:
          :type arm_motor: ev3.MediumMotor
          :type touch_sensor: ev3.TouchSensor
        """
        arm_motor = ev3.MediumMotor(ev3.OUTPUT_A)
        assert arm_motor.connected
        touch_sensor = ev3.TouchSensor()
        assert touch_sensor
        arm_motor.run_forever(speed_sp=900)
        while not touch_sensor.is_pressed:
            time.sleep(0.01)
        arm_motor.stop(stop_action=ev3.Motor.STOP_ACTION_BRAKE)
        ev3.Sound.beep().wait()
        arm_revolutions_for_full_range = 14.2
        degrees_for_arm_revolution = arm_revolutions_for_full_range * 360
        arm_motor.run_to_rel_pos(position_sp=-degrees_for_arm_revolution)
        arm_motor.wait_while(ev3.Motor.STATE_RUNNING)
        ev3.Sound.beep().wait()
        arm_motor.position = 0

    def arm_up(self):
        """
        Moves the Snatch3r arm to the up position.

        Type hints:
          :type arm_motor: ev3.MediumMotor
          :type touch_sensor: ev3.TouchSensor
        """
        arm_motor = ev3.MediumMotor(ev3.OUTPUT_A)
        assert arm_motor.connected
        touch_sensor = ev3.TouchSensor()
        assert touch_sensor
        arm_motor.run_forever(speed_sp=900)
        while not touch_sensor.is_pressed:
            time.sleep(0.01)
        arm_motor.stop(stop_action=ev3.Motor.STOP_ACTION_BRAKE)
        ev3.Sound.beep().wait()

    def arm_down(self):
        """
        Moves the Snatch3r arm to the down position.

        Type hints:
          :type arm_motor: ev3.MediumMotor
        """
        arm_motor = ev3.MediumMotor(ev3.OUTPUT_A)
        assert arm_motor.connected
        arm_motor.run_to_abs_pos(position_sp=0, speed_sp=900)
        arm_motor.wait_while(ev3.Motor.STATE_RUNNING)
        ev3.Sound.beep().wait()

    def shutdown(self):
        left_motor = ev3.LargeMotor(ev3.OUTPUT_B)
        right_motor = ev3.LargeMotor(ev3.OUTPUT_C)
        left_motor.stop()
        right_motor.stop()

        ev3.Leds.set_color(ev3.Leds.LEFT, ev3.Leds.GREEN)
        ev3.Leds.set_color(ev3.Leds.RIGHT, ev3.Leds.GREEN)

        self.running = False

        print('Goodbye!')
        ev3.Sound.speak('Goodbye').wait()

    def seek_beacon(robot):
        beacon_seeker = ev3.BeaconSeeker(channel=1)
        forward_speed = 300
        turn_speed = 100
        left_motor = ev3.LargeMotor(ev3.OUTPUT_B)
        right_motor = ev3.LargeMotor(ev3.OUTPUT_C)

        while not robot.touch_sensor.is_pressed:

            current_heading = beacon_seeker.heading
            current_distance = beacon_seeker.distance

            if math.fabs(current_heading) < 2:
                print("On the right heading. Distance: ", current_distance)

                if current_distance == 0:
                    robot.drive_inches(5, 100)
                    left_motor.stop()
                    right_motor.stop()
                    return True
                elif current_distance > 0:
                    print('Starting to drive towards beacon')
                    left_motor.run_forever(speed_sp=forward_speed)
                    right_motor.run_forever(speed_sp=forward_speed)
            if math.fabs(current_heading) > 2 & int(math.fabs(
                    current_heading)) < 10:
                print('Turning to find beacon', 'Beacon Heading:',
                      current_heading)
                if current_heading < 0:
                    left_motor.run_forever(speed_sp=-turn_speed)
                    right_motor.run_forever(speed_sp=turn_speed)
                if current_heading > 0:
                    left_motor.run_forever(speed_sp=turn_speed)
                    right_motor.run_forever(speed_sp=-turn_speed)
            if math.fabs(current_heading) > 10:
                if current_heading > 0:
                    left_motor.run_forever(speed_sp=turn_speed)
                    right_motor.run_forever(speed_sp=-turn_speed)
                if current_heading < 0:
                    left_motor.run_forever(speed_sp=-turn_speed)
                    right_motor.run_forever(speed_sp=turn_speed)
            time.sleep(0.2)
        return False

    def go_forward(self, left_speed, right_speed):
        self.left_motor.run_forever(speed_sp=left_speed)
        self.right_motor.run_forever(speed_sp=right_speed)

    def turn_left(self, left_speed, right_speed):
        self.left_motor.run_forever(speed_sp=-left_speed)
        self.right_motor.run_forever(speed_sp=right_speed)

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()

    def turn_right(self, left_speed, right_speed):
        self.left_motor.run_forever(speed_sp=left_speed)
        self.right_motor.run_forever(speed_sp=-right_speed)

    def go_backwards(self, left_speed, right_speed):
        self.left_motor.run_forever(speed_sp=-left_speed)
        self.right_motor.run_forever(speed_sp=-right_speed)

    def organize_color_retrieve(self, button_state, list_of_modes,
                                color_input):
        list_of_names = ['red', 'blue', 'green']
        print(color_input)
        if button_state:
            self.pixy.mode = list_of_modes[color_input]

            print(self.pixy.mode, 'Color:', list_of_names[color_input])
            if self.pixy.value(3) > 5 and self.pixy.value(4) > 15:
                print('color detected')
                self.drive_inches(10, 300)
                color_reached = True
            else:
                print('color not found')

                return

            if color_reached:
                self.left_motor.stop()
                self.right_motor.stop()
                print('color reached')

                self.arm_up()

                self.turn_degrees(95, 300)

            self.organize_color_dropoff(color_input)

    def organize_color_dropoff(self, color_input):
        print(color_input)
        if color_input == 0:
            color_to_seek = ev3.ColorSensor.COLOR_RED
        elif color_input == 1:
            color_to_seek = ev3.ColorSensor.COLOR_BLUE
        elif color_input == 2:
            color_to_seek = ev3.ColorSensor.COLOR_GREEN
        else:
            color_to_seek = ev3.ColorSensor.COLOR_WHITE
        print(color_to_seek)
        light_intensity = self.color_sensor.reflected_light_intensity

        while light_intensity > 15:
            self.left_motor.run_forever(speed_sp=200)
            self.right_motor.run_forever(speed_sp=200)
            light_intensity = self.color_sensor.reflected_light_intensity
            time.sleep(0.1)

        print('reached turn')

        self.left_motor.stop()
        self.right_motor.stop()

        self.turn_degrees(95, 200)
        print('turned')

        while self.color_sensor.color != color_to_seek:
            self.left_motor.run_forever(speed_sp=200)
            self.right_motor.run_forever(speed_sp=200)

        self.left_motor.stop()
        self.right_motor.stop()

        self.turn_degrees(-90, 300)

        self.arm_down()
