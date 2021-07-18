"""drum_orchestra controller."""

# Import some classes of the controller module and OSC communication
from drum_orchestra_utils import (
    init_robot, init_motors, init_sensors, switch_case, set_velocity
)
from controller import (Motor, PositionSensor, TouchSensor)

target_positions = [-1.88, -2.14, -2.38, -1.51]
counter = 0
i = 0
argument = 0
speed = 1.0


robot, timestep, client = init_robot()
hand_motors, ur_motors = init_motors(robot)
set_velocity(ur_motors, speed)
distance_sensor, touch_sensor, position_sensor = init_sensors(robot, timestep)

while robot.step(timestep) != -1:
    if counter <= 0:
        if switch_case(argument) == "WAITING":
            counter = 8
            for i in range(len(target_positions)):
                Motor.setPosition(ur_motors[i], target_positions[i])
                i += 1
            print("Rotating arm \n")
            argument = 1

        elif switch_case(argument) == "ROTATING":
            if PositionSensor.getValue(position_sensor) < -1.7:
                print("Collision \n")
                argument = 2
                touch_value = TouchSensor.getValue(touch_sensor)
                print("Detecting a collision of:", round(touch_value, 2), "N")
                client.send_message("/controllers/robotic_arm", touch_value)

        elif switch_case(argument) == "COLLISION":
            for i in range(len(ur_motors)):
                Motor.setPosition(ur_motors[i], 0.0)
                i += 1
            print("Rotating arm back \n")
            argument = 3

        elif switch_case(argument) == "ROTATING_BACK":
            if PositionSensor.getValue(position_sensor) > -0.7:
                argument = 0
                print("Waiting \n")

    counter -= 1

    pass
