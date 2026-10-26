import time
from lib.robot_consts import Port, Sensor, Button, Light

DIST_OBSTACLE_MM = 150
PUSH_PWR = 100
PUSH_TIMEOUT_MS = 5000
LINE_THRESHOLD = 20      # reflection below this = line (tune)
BACK_PWR = -50
BACK_MS = 300
LOOP_MS = 10

robot.init_motor(Port.M1)
robot.init_motor(Port.M2)
robot.init_sensor(sensor_type=Sensor.OC_COLOR, port=Port.S1)
robot.init_sensor(sensor_type=Sensor.OC_LASER, port=Port.S2)


def clamp(x, lo=-100, hi=100):
    return max(lo, min(hi, int(x)))


def set_motors(m1, m2):
    robot.motors[Port.M1].set_power(clamp(m1))
    robot.motors[Port.M2].set_power(clamp(m2))


def push_box():
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < PUSH_TIMEOUT_MS:
        if robot.sensors.light[Port.S1].reflection() < LINE_THRESHOLD:
            break
        if robot.buttons.pressed_since()[Button.LEFT]:
            break
        set_motors(PUSH_PWR, PUSH_PWR)
        time.sleep_ms(LOOP_MS)

    set_motors(0, 0)
    set_motors(BACK_PWR, BACK_PWR)
    time.sleep_ms(BACK_MS)
    set_motors(0, 0)


while True:
    if robot.buttons.pressed_since()[Button.LEFT]:
        break

    dist = robot.sensors.laser[Port.S2].distance_short()  # mm
    if 0 < dist < DIST_OBSTACLE_MM:
        print("Box detected:", dist)
        push_box()

    time.sleep_ms(LOOP_MS)

set_motors(0, 0)
