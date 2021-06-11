"""drum_orchestra controller."""

# Import some classes of the controller module and OSC communication
from controller import Robot
from controller import Motor
from controller import DistanceSensor
from controller import PositionSensor
from controller import TouchSensor
from pythonosc.udp_client import SimpleUDPClient


# defining swith case function
def switch_case(argument):
    switcher = {
        0: "WAITING",
        1: "ROTATING",
        2: "COLLISION",
        3: "ROTATING_BACK"
    }
    
    return switcher.get(argument, "nothing")


robot = Robot()
counter = 0
i = 0
argument = 0
target_positions = [-1.88, -2.14, -2.38, -1.51]
speed = 1.0


# set the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# set the ip and port to send OSC Messages to SuperCollider
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


for i in range(len(ur_motors)):
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
        if(switch_case(argument) == "WAITING"):
            counter = 8
            for i in range(len(target_positions)):
                Motor.setPosition(ur_motors[i], target_positions[i])
                i += 1
            print("Rotating arm \n")
            argument = 1
            
        elif(switch_case(argument) == "ROTATING"):
            if(PositionSensor.getValue(position_sensor) < -1.7):
                print("Collision \n")
                argument = 2
                touch_value = TouchSensor.getValue(touch_sensor)
                print("Detecting a collision of:", round(touch_value, 2), "N")
                client.send_message("/controllers/robotic_arm", touch_value)
                
        elif(switch_case(argument) == "COLLISION"):
            for i in range(len(ur_motors)):
                Motor.setPosition(ur_motors[i], 0.0)
                i += 1
            print("Rotating arm back \n")
            argument = 3
            
        elif(switch_case(argument) == "ROTATING_BACK"):
            if(PositionSensor.getValue(position_sensor) > -0.7):
                argument = 0
                print("Waiting \n")
                
    counter -= 1
    pass

# Enter here exit cleanup code.
