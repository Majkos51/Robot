import time
from lib.robot_consts import Port, Sensor, Button, Light

REGUL_PERIOD_MS = 10
Kp, Ki, Kd = 1.8, 0.1, 16

light_error_sum, light_error_prev = 0, 0

# --- prahy pro objetí lahve (ladit na trati) ---
DIST_OBSTACLE_MM = 180   # kdy začít objetí
AVOID_SIDE = 1           # 1 = objetí doprava, -1 = doleva
PWR_TURN = 45
PWR_FWD = 50

# časy manévru (ms) – ladit podle velikosti lahve a rychlosti
T_TURN1 = 350
T_FWD1 = 700
T_TURN2 = 350
T_FIND_LINE_MAX = 2500

STATE_FOLLOW = 0
STATE_AVOID = 1
state = STATE_FOLLOW

robot.init_motor(Port.M1)
robot.init_motor(Port.M2)
robot.init_sensor(sensor_type=Sensor.OC_COLOR, port=Port.S1)
robot.init_sensor(sensor_type=Sensor.OC_LASER, port=Port.S2)

light_base_int = 33
motor_base_pwr = 60


def clamp(x, lo=-100, hi=100):
    return max(lo, min(hi, int(x)))


def set_motors(m1, m2):
    robot.motors[Port.M1].set_power(clamp(m1))
    robot.motors[Port.M2].set_power(clamp(m2))


def drive_ms(m1, m2, duration_ms):
    set_motors(m1, m2)
    time.sleep_ms(duration_ms)
    set_motors(0, 0)


def find_line(timeout_ms):
    """Jede dopředu, dokud nevidí čáru (tmavší než base + rezerva)."""
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
        light = robot.sensors.light[Port.S1].reflection()
        if light < light_base_int - 5:  # čára je tmavší
            return True
        set_motors(PWR_FWD, PWR_FWD)
        time.sleep_ms(REGUL_PERIOD_MS)
        buttons = robot.buttons.pressed_since()
        if buttons[Button.LEFT]:
            return False
    return False


def avoid_bottle():
    """Jednoduché objetí: odbočka → rovně → zpět → hledání čáry."""
    global light_error_sum, light_error_prev
    light_error_sum = 0
    light_error_prev = 0

    # 1) odbočka od lahve
    drive_ms(
        PWR_TURN * AVOID_SIDE,
        -PWR_TURN * AVOID_SIDE,
        T_TURN1,
    )
    # 2) kolem lahve
    drive_ms(PWR_FWD, PWR_FWD, T_FWD1)
    # 3) zpět k čáře
    drive_ms(
        -PWR_TURN * AVOID_SIDE,
        PWR_TURN * AVOID_SIDE,
        T_TURN2,
    )
    # 4) jet, dokud nenajde čáru
    find_line(T_FIND_LINE_MAX)


print("base:", light_base_int)

while True:
    buttons = robot.buttons.pressed_since()
    if buttons[Button.LEFT]:
        break

    if state == STATE_FOLLOW:
        dist = robot.sensors.laser[Port.S2].distance_short()  # mm
        # print(dist)

        if 0 < dist < DIST_OBSTACLE_MM:
            state = STATE_AVOID
            continue

        light_intensity = robot.sensors.light[Port.S1].reflection()
        light_error = light_base_int - light_intensity
        P = Kp * light_error
        I = Ki * light_error_sum
        D = Kd * (light_error - light_error_prev)
        motor_pwr = P + I + D

        light_error_prev = light_error
        light_error_sum += light_error
        # anti-windup I
        if light_error_sum > 500:
            light_error_sum = 500
        elif light_error_sum < -500:
            light_error_sum = -500

        set_motors(motor_base_pwr + motor_pwr, motor_base_pwr - motor_pwr)
        time.sleep_ms(REGUL_PERIOD_MS)

    elif state == STATE_AVOID:
        avoid_bottle()
        state = STATE_FOLLOW

set_motors(0, 0)
