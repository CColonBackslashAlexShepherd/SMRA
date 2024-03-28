import serial
import time
import RPi.GPIO as GPIO
import threading
import math
import numpy as np
from matplotlib import pyplot as plt
import scipy as sp
#GPIO pin constants

DIR = 20 # Direction GPIO Pin
STEP = 21 # Step GPIO Pin
CW = 1 # Clockwise Rotation
CCW = 0 # Counterclockwise Rotation

# GPIO setup

GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)
GPIO.output(DIR, CW)

#motor control values

frequency = 0
direction = 1
mot_enable = False

#control variables and constants

feedback_max = 1023 #maximum value we read from our potentiometer
data_slope = 0.30405 #the derivative of degrees to the arbitrary data measurement
angle_offset = 0#11.5384#-156#-113.5 #offset the angle measured by the potentiometer
k = 1/20  #proportional control coefficient

measured_angle = 0 #angle measured from feedback data
target_angle = 90 #target angle in degrees
rawdata = 0
target_settling_limit = 2 #the range of degrees where the error is acceptable enough to stop moving the motor
# k = 1/90 gives 100% maximum motor velocity
# k = 1/180 gives 50% maximum motor velocity
# k = 1/360 gives 25% maximum motor velocity
# the sign of k will determine the direction the motor should rotate

#GPT-3.5 generated function
def save_to_csv(array1, array2, filename):
    """
    Save two arrays to a CSV file.

    Parameters:
    - array1: numpy array or list
        First array to be saved.
    - array2: numpy array or list
        Second array to be saved. Must have the same length as array1.
    - filename: str
        Filename for the CSV file.

    Returns:
    - None
    """
    
    # Check if arrays have the same length
    if len(array1) != len(array2):
        raise ValueError("Arrays must have the same length.")

    # Stack arrays horizontally
    data = np.column_stack((array1, array2))
    
    # Save data to CSV file
    np.savetxt(filename, data, delimiter=',', header='Array1,Array2', comments='')

# Example usage:
array1 = np.linspace(0, 10, 100)
array2 = np.sin(array1)
save_to_csv(array1, array2, 'data.csv')

#stepper motor control function. This will run in a different thread
def control_stepper():
    while True:
        global direction, lock
        
        try:
            T = 1/frequency
        except:
            T = -1
        if T >= 0 and mot_enable == True:
            # Drive stepper motor forward
            GPIO.output(DIR, direction)  # Set direction
        
            GPIO.output(STEP, GPIO.HIGH)
            time.sleep(0.5*T)
            GPIO.output(STEP, GPIO.LOW)
            time.sleep(0.5*T)
        
#serial command send
def send_command(command):
    ser.write(command.encode()) # Send command to Arduino

#serial read function
def receive_response():
    return ser.readline().decode().strip() # Read response from Arduino

#update our feedback data from the arduino (acting as our ADC)
def update_feedback():
    command = 'r' #r - send potentiometer data
    send_command(command)
    
    # Receive data from arduino
    data = receive_response()
    
    try:
        fdata = float(data)
    except:
        fdata = 0
    
    feedback_decoder(fdata)

#convert the data from the arduino into an angle measurement
def feedback_decoder(raw):
    global measured_angle, rawdata
    rawdata = raw
    #measured_angle = max_potentimeter_angle*(raw/feedback_max)+angle_offset
    measured_angle = data_slope*raw+angle_offset
def controlLoop():
    #find current error
    error = (target_angle - measured_angle)
    
    if abs(error) < target_settling_limit/2:
        
        motor_velocity = 0
    else:
        #calculate motor velocity
        motor_velocity = k*error
    
    #clip motor velocity at 100%
    if motor_velocity > 2:
        motor_velocity = 2
    elif motor_velocity < -2:
        motor_velocity = -2
    
        
    return motor_velocity
    
def move_motor(velocity):
    global frequency, direction
    frequency = abs(velocity)*250
    if (velocity < 0):
        direction = 1
    else:
        direction = 0
    #print("f = {}, d = {}".format(frequency,direction))
# Main loop
def main_loop():
    global frequency
    global direction

    update_feedback()
    motor_velocity = controlLoop()
    print(measured_angle)
    move_motor(motor_velocity)

# Initialize serial communication with Arduino
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1) # Use the correct USB port
# Flush input buffer
ser.reset_input_buffer()

# Create and start the thread for stepper motor control
stepper_thread = threading.Thread(target=control_stepper)
stepper_thread.start()


target_angle = 90
measurement_summation = 0
mot_enable = False

count = 0

N = 2000

t = np.zeros(N)
t0 = time.time()


targetvalue = np.zeros(N)

send_command('r')
receive_response()

measured = np.zeros(N)

stop_control = False


while True:
    main_loop()
    if count <= N-1:
        t[count] = time.time() - t0
        measured[count] = measured_angle
        targetvalue[count] = target_angle
    else:
        stop_control = True
    count += 1
    
    if count % 500 == 0:
        if target_angle == 45:
            target_angle = 135
        else:
            target_angle = 45
    
    if stop_control == True:
        mot_enable = False
        break
save_to_csv(t,measured,"measuredvtime")
w = sp.fft.fftfreq(N,1/100)*math.pi
measured_fft = np.fft.fft(measured)
plt.figure()
plt.semilogx(w, np.abs(measured_fft))  # Magnitude plot
plt.xlabel('Frequency (rad/s)')
plt.ylabel('Magnitude')
plt.title('Frequency Response (Magnitude)')
plt.grid(True)
plt.show()
# ~ fig, ax = plt.subplots()
# ~ ax.plot(t,measured)
# ~ ax.plot(t,targetvalue)
# ~ plt.show()

