#!/usr/bin/env python3
"""Check the robot's GPIO setup and wiring without ROS.

Run on the Pi, with ROS stopped (two programs can't hold the same pin):

    python3 ~/LLM-Controlled-Robot/tools/gpio_check.py            # library, permissions, sensors
    python3 ~/LLM-Controlled-Robot/tools/gpio_check.py --motors   # also spin each wheel (lift it!)

Pin numbers are BCM and match motor_driver_node, ultrasonic_publisher and cliff_publisher.
"""
import argparse
import grp
import os
import sys
import time

MOTORS = {  # wheel: (forward pin, reverse pin, PWM enable pin)
    'left': (22, 17, 18),
    'right': (23, 24, 19),
}
US_TRIG, US_ECHO = 4, 27
CLIFF = {'front_left': 5, 'front_right': 6}   # level 1 = no floor (TCRT5000 LED off)

failures = []


def ok(msg):
    print(f'  OK    {msg}')


def bad(msg, fix):
    failures.append(msg)
    print(f'  FAIL  {msg}\n        -> {fix}')


def check_library():
    print('GPIO library')
    try:
        import RPi.GPIO as GPIO
    except Exception as e:  # ImportError, or RuntimeError when not on a Pi
        bad(f'cannot import RPi.GPIO ({e})', 'sudo apt install python3-rpi-lgpio')
        return None
    path = GPIO.__file__
    if path.startswith('/usr/local') or '/.local/' in path:
        bad(f'RPi.GPIO comes from pip ({path}); it hides the Ubuntu-compatible version',
            'sudo pip uninstall --break-system-packages RPi.GPIO  (and the same without sudo)')
    else:
        ok(f'RPi.GPIO from {path}')
    try:
        import lgpio  # noqa: F401  (present when RPi.GPIO is the rpi-lgpio version)
        ok('using rpi-lgpio (works on Ubuntu 24.04)')
    except ImportError:
        bad('this is the classic RPi.GPIO, which often fails on Ubuntu 24.04',
            'sudo apt remove python3-rpi.gpio && sudo apt install python3-rpi-lgpio')
    return GPIO


def check_permissions():
    print('Permissions')
    groups = {grp.getgrgid(g).gr_name for g in os.getgroups()}
    if os.geteuid() == 0:
        print('  NOTE  running as root: this hides permission problems the robot will hit. '
              'Run without sudo.')
    elif 'dialout' in groups:
        ok('your user is in the dialout group')
    else:
        bad('your user is not in the dialout group (or you have not logged in again since)',
            'sudo usermod -aG dialout $USER, then log out and back in (or reboot)')
    chip = '/dev/gpiochip0'
    if not os.path.exists(chip):
        bad(f'{chip} does not exist', 'check you are on the Pi, running Ubuntu 24.04')
    elif os.access(chip, os.R_OK | os.W_OK):
        ok(f'{chip} is readable and writable')
    else:
        bad(f'no access to {chip}', 'add the udev rule from the README ("GPIO setup"), reboot')


def claim(GPIO, pin, mode, label):
    try:
        if mode == 'out':
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        elif mode == 'in_pullup':
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        else:
            GPIO.setup(pin, GPIO.IN)
        return True
    except Exception as e:
        bad(f'GPIO{pin} ({label}): {e}',
            'stop ROS / other robot programs (pkill -f ros2), check with gpioinfo, '
            'then check permissions above')
        return False


def check_ultrasonic(GPIO):
    print('Front ultrasonic (HC-SR04)')
    if not (claim(GPIO, US_TRIG, 'out', 'ultrasonic TRIG')
            and claim(GPIO, US_ECHO, 'in', 'ultrasonic ECHO')):
        return
    readings = []
    for _ in range(3):
        GPIO.output(US_TRIG, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(US_TRIG, GPIO.LOW)
        t0 = time.perf_counter()
        while GPIO.input(US_ECHO) == 0:
            if time.perf_counter() - t0 > 0.03:
                break
        else:
            rise = time.perf_counter()
            while GPIO.input(US_ECHO) == 1 and time.perf_counter() - rise < 0.03:
                pass
            readings.append((time.perf_counter() - rise) * 343.0 / 2.0)
        time.sleep(0.06)
    if readings:
        ok('distance ahead: ' + ', '.join(f'{d:.2f} m' if d < 4.0 else 'clear' for d in readings)
           + '  (move your hand in front of it and run again to see it change)')
    else:
        bad('no echo from the ultrasonic sensor',
            'check VCC 5 V, GND, TRIG on pin 7 (GPIO4), ECHO via the divider on pin 13 (GPIO27)')


def check_cliff(GPIO):
    print('Edge (cliff) sensors, if fitted')
    for name, pin in CLIFF.items():
        if claim(GPIO, pin, 'in_pullup', f'cliff {name}'):
            level = GPIO.input(pin)
            state = 'NO FLOOR (or not connected)' if level else 'floor seen'
            print(f'  ..    {name} on GPIO{pin}: {state}')
    print('        With the robot on the floor both should say "floor seen"; lift the front '
          'and both should change.')


def check_motors(GPIO):
    print('Motors (wheels must be off the ground)')
    for wheel, (fwd, rev, en) in MOTORS.items():
        if not all(claim(GPIO, p, 'out', f'{wheel} motor') for p in (fwd, rev, en)):
            continue
        pwm = GPIO.PWM(en, 1000)
        pwm.start(0)
        for label, high, low in (('forward', fwd, rev), ('backward', rev, fwd)):
            print(f'  ..    {wheel} wheel {label} for 1 s')
            GPIO.output(low, GPIO.LOW)
            GPIO.output(high, GPIO.HIGH)
            pwm.ChangeDutyCycle(80)
            time.sleep(1.0)
            pwm.ChangeDutyCycle(0)
            GPIO.output(high, GPIO.LOW)
            time.sleep(0.5)
        pwm.stop()
    print('        If a wheel turned the wrong way, set invert_left / invert_right in '
          'robot.yaml. If one never turned, check its IN/EN wires and the L298N power.')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--motors', action='store_true',
                        help='also spin each wheel forward and back (lift the robot first)')
    args = parser.parse_args()

    GPIO = check_library()
    check_permissions()
    if GPIO is None:
        return 1
    try:
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
    except Exception as e:
        bad(f'cannot open the GPIO chip ({e})',
            'fix the permissions above; "Cannot determine SOC peripheral base address" means '
            'the classic RPi.GPIO is installed instead of python3-rpi-lgpio')
        return 1
    try:
        check_ultrasonic(GPIO)
        check_cliff(GPIO)
        if args.motors:
            check_motors(GPIO)
        else:
            print('Motors: skipped (add --motors, with the wheels off the ground)')
    finally:
        GPIO.cleanup()
    print()
    if failures:
        print(f'{len(failures)} problem(s) found. Fix the first one, then run this again.')
        return 1
    print('GPIO looks good.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
