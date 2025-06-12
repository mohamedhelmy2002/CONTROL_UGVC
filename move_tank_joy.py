import sim 
import time 
import pygame
# from speed stear to left and right speeed 
def motor_speeds(steer, speed):
    # steer: positive means turn right, negative means turn left
    # speed: base forward/backward speed

    # Combine speed and steer for differential drive
    left_speed = speed + steer
    right_speed = speed - steer

    print(f"Left Motor Speed: {left_speed}", end=" | ")
    print(f"Right Motor Speed: {right_speed}")
    return left_speed, right_speed

# === Smooth speed update ===
def gradual_speed_update(target_speed, steer, step_value=10, step_time=0.1):
    global current_speed

    while current_speed != target_speed:
        diff = target_speed - current_speed
        if abs(diff) < step_value:
            current_speed = target_speed
        else:
            current_speed += step_value if diff > 0 else -step_value
        #print(current_speed,steer)
        motor_speeds(steer, current_speed)
        #send_to_esp(current_speed, steer)
        time.sleep(step_time)
    motor_speeds(steer, current_speed)
    #print(current_speed,steer)
    return steer,current_speed

# === Joystick Setup ===
pygame.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()
current_speed = 0  # Global current speed

BASE_SPEED = 400
speed_levels = [0.2, 0.35, 0.5, 0.65, 0.75, 0.85, 1.0]
speed_index = 0
DEADZONE = 0.1
last_button_state = False

# make connection 
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,8000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')
    # intial parameter 
    # define robot 
    _,robot_handel=sim.simxGetObjectHandle(clientID,'/robot',sim.simx_opmode_oneshot_wait)
    # acuators 
    e,left_back_handle= sim.simxGetObjectHandle(clientID,'./left_back',sim.simx_opmode_oneshot_wait)
    e,right_back_handle= sim.simxGetObjectHandle(clientID,'./right_back',sim.simx_opmode_oneshot_wait)
    sj_l_handles = [sim.simxGetObjectHandle(clientID, f'sj_l_{i}', sim.simx_opmode_blocking)[1] for i in range(1, 7)]
    sj_r_handles = [sim.simxGetObjectHandle(clientID, f'sj_r_{i}', sim.simx_opmode_blocking)[1] for i in range(1, 7)]

    # intial acuator velocity 
    left_velocity= sim.simxSetJointTargetVelocity(clientID,left_back_handle,0,sim.simx_opmode_streaming)
    right_veloctiy= sim.simxSetJointTargetVelocity(clientID,right_back_handle,0,sim.simx_opmode_streaming)
    
    #camera
    errorCode,cam=sim.simxGetObjectHandle(clientID,'camera',sim.simx_opmode_oneshot_wait)

    # start controlling 
    startTime=time.time()
    while time.time()-startTime < 60:
        pygame.event.pump()

        steering = joystick.get_axis(0)
        speed = joystick.get_axis(1)*-1

        # Deadzone filtering
        if abs(speed) < DEADZONE:
            speed = 0
        if abs(steering) < DEADZONE:
            steering = 0

        # Handle speed level button
        button_pressed = joystick.get_button(0)
        if button_pressed and not last_button_state:
            speed_index = (speed_index + 1) % len(speed_levels)
            print(f"Speed level changed to {int(speed_levels[speed_index] * 100)}%")
        last_button_state = button_pressed

        level = speed_levels[speed_index]
        max_speed = BASE_SPEED * level

        speed_to_send = int(speed * max_speed)
        steer_to_send = int(steering * max_speed)
        
        st,sp=gradual_speed_update(speed_to_send, steer_to_send)
        left_sp,right_sp=motor_speeds(st,sp)
        # real time paramter
        #get object position and oriantation
        _,positions=sim.simxGetObjectPosition(clientID,robot_handel,-1,sim.simx_opmode_streaming)
        _,angles=sim.simxGetObjectOrientation(clientID,robot_handel,-1,sim.simx_opmode_streaming)
        #print(angles)
       # print(angles)
            
        # control
        # add forces in joints
        left_back_f=sim.simxSetJointMaxForce(clientID,left_back_handle, 125,sim.simx_opmode_streaming) 
        right_back_f =sim.simxSetJointMaxForce(clientID,right_back_handle,125 ,sim.simx_opmode_streaming )
        for handle in sj_l_handles + sj_r_handles:
            d=sim.simxSetJointForce(clientID, handle, 0, sim.simx_opmode_streaming)
        # velocity 
        left_velocity= sim.simxSetJointTargetVelocity(clientID,left_back_handle,left_sp,sim.simx_opmode_streaming)
        right_veloctiy= sim.simxSetJointTargetVelocity(clientID,right_back_handle,right_sp,sim.simx_opmode_streaming)
        
            
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
