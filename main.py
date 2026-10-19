import time
from lib.robot_consts import Port, Sensor, Button, Light

#perioda mereni hodnoty
REGUL_PERIOD_MS = 10
#hodnota svetla ke ktere robot miri
light_base_int = 0

Kp, Ki, Kd = 1.8, 0.1, 16

#akumulace erroru a predchozi error
light_error_sum, light_error_prev = 0, 0


#iniciliziace
robot.init_motor(Port.M1)
robot.init_motor(Port.M2)
robot.init_sensor(sensor_type = Sensor.OC_COLOR, port = Port.S1)


light_base_int = robot.sensors.light[Port.S1].reflection()
light_base_int = 33
motor_base_pwr = 60


print(light_base_int)

while True:
    #mereni
    light_intensity = robot.sensors.light[Port.S1].reflection()
    #print(light_intensity)
    
    #Výpočet PID regulace
    light_error = light_base_int - light_intensity
    P = Kp * light_error
    I = Ki * light_error_sum
    D = Kd * (light_error - light_error_prev)
    #zmena vykonu
    motor_pwr = P + I + D

    #historie chyb
    light_error_prev = light_error
    light_error_sum += light_error
    motor1 = motor_base_pwr + motor_pwr
    motor2 = motor_base_pwr - motor_pwr    
        
    #nastaveni vykonu motoru
    robot.motors[Port.M2].set_power(motor2)
    robot.motors[Port.M1].set_power(motor1)
    
    #ukonceni programu
    buttons = robot.buttons.pressed_since()
    if buttons[Button.LEFT]:
        break
    
    #pockame pred dalsim zmerenim
    time.sleep_ms(REGUL_PERIOD_MS)

#vypnuti motoru
robot.motors[Port.M1].set_power(0)
robot.motors[Port.M2].set_power(0)
