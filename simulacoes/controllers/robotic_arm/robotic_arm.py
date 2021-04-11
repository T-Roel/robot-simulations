"""robotic_arm controller."""

# Import some classes of the controller module and OSC communication
from controller import Robot
from controller import Motor
from controller import DistanceSensor
from controller import PositionSensor
from controller import TouchSensor
from pythonosc.udp_client import SimpleUDPClient

def switch_case(argument):
    switcher = {
        "WAITING": 0,
        "GRASPING": 1,
        "ROTATING": 2,
        "COLLISION": 3,
        "ROTATING_BACK": 4
    }
    
    return switcher.get(argument, "nothing")


robot = Robot()
counter = 0
i = 0
argument = "WAITING"
target_positions = [-1.88, -2.14, -2.38, -1.51]
speed = 1.0


# set the time step of the current world.
timestep = int(robot.getBasicTimeStep())
ip = "127.0.0.1"
port = 57120
client = SimpleUDPClient(ip, port)


# Insert a getDevice-like function in order to get the
# instance of a device of the robot.
hand_motors = [robot.getDevice("finger_1_joint_1"),
               robot.getDevice("finger_2_joint_1"),
               robot.getDevice("finger_middle_joint_1")]


ur_motors = [robot.getDevice("shoulder_lift_joint"),
             robot.getDevice("elbow_joint"),
             robot.getDevice("wrist_1_joint"),
             robot.getDevice("wrist_2_joint")]


for i in range(4):
    Motor.setVelocity(ur_motors[i], speed)
    i += 1


distance_sensor = robot.getDevice("distance sensor")
DistanceSensor.enable(distance_sensor, timestep)


touch_sensor = robot.getDevice("force")
TouchSensor.enable(touch_sensor, timestep)


position_sensor = robot.getDevice("wrist_1_joint_sensor")
PositionSensor.enable(position_sensor, timestep)


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    if(counter <=0):
    
        if(switch_case(argument) == 0):
            if(DistanceSensor.getValue(distance_sensor) < 500):
                counter = 8
                argument = "GRASPING"
                print("Gasping can\n")
                for i in range(len(hand_motors)):
                    Motor.setPosition(hand_motors[i], 0.85)
                    i += 1
                    
        elif(switch_case(argument) == 1):
            for i in range(len(target_positions)):
                Motor.setPosition(ur_motors[i], target_positions[i])
                i += 1
            print("Rotating arm \n")
            argument = "ROTATING"
            
        elif(switch_case(argument) == 2):
            if(PositionSensor.getValue(position_sensor) < -1.6):
                counter = 8
                print("Collision \n")
                argument = "COLLISION"
                touch_value = TouchSensor.getValue(touch_sensor)
                print("Detecting a collision of:", round(touch_value, 2), "N")
                client.send_message("/controllers/robotic_arm", touch_value)
                
        elif(switch_case(argument) == 3):
            for i in range(len(ur_motors)):
                Motor.setPosition(ur_motors[i], 0.0)
                i += 1
            print("Rotating arm back \n")
            argument = "ROTATING_BACK"
            
        elif(switch_case(argument) == 4):
            if(PositionSensor.getValue(position_sensor) > -0.5):
                argument = "WAITING"
                print("Waiting \n")
                
    counter -= 1
    
    pass

