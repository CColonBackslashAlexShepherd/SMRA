from machine import Pin, ADC, UART ,PWM
import time
import ustruct
import _thread
import math

#emergency stop parameter to kill the program.
E_STOP = False

#Constants for ADC to angle conversion
#min_adc and max_adc must be recalibrated whenever the potentiometer is adjusted

min_adc = 400   #adc reading with the actuator at 0 degrees
max_adc = 7121  #adc reading with the actuator at 180 degrees
min_angle = 0   
max_angle = 180

#maximum allowable frequency for the steps sent to the motor

MAX_FREQUENCY = 10000

#set up the dir and step outputs for the stepper motor controller
DIR = Pin(20,Pin.OUT)
STEP = PWM(Pin(21,Pin.OUT))

#the minimum possible PWM frequency is 8Hz, so we initialize to that
STEP.freq(8)

#0% step frequency effectively disables the motor controller
STEP.duty_ns(0)

#variables for frequency going to the motor controller, the direction of the motor controller, and a boolean enable variable to disable movement
frequency = 0
direction = 1
mot_enable = False

# Configure the ADC pin
pot = ADC(27)


def read_ADC(DEBUG_MODE=False):
#Takes: an optional debug mode parameter
#returns: the calcualted angle of the actuator

    # Read the raw ADC value
    adc_value = pot.read_u16()

    #calculate the angle of the actuator with the adc reading and the predefined constants
    angle = min_angle + ((adc_value - min_adc) / (max_adc - min_adc)) * (max_angle - min_angle)
    #angle = adc_value*ADC_SCALE_FACTOR -ADC_ANGLE_OFFSET#converts 16 bit value into degrees

    #print some information to evaluate the calculation if debug mode is enabled
    if DEBUG_MODE == True:
        print("ADC RAW: {}, angle: {}".format(adc_value,angle))
    
    #stop the program if there's an nonsensical reading from the pot.
    if angle > 360:
        global E_STOP

        #this will kill the main loop.
        E_STOP = True
        print("Warning! Stopping due to angle: {} Degrees".format(angle))
        angle = 90 #safest angle for the control loop

    #if we are not in debug mode, we set the angle to its minimum and maximum if we are overshooting by a small amount.
    if angle < 0.01 and not DEBUG_MODE:
        angle = 0
    if angle > 180 and not DEBUG_MODE:
        angle = 180
    
    return angle

def set_step_frequency(frequency):
#takes: the absolute value of the frequency calculated by the control_stepper function
#returns: nothning, but the program will set the frequency of the motor controller.

    if frequency < 8: #is the frequency below the minimum?
        STEP.freq(8)#minimum PWM value
        STEP.duty_u16(0) #0% duty cycle

    elif frequency > MAX_FREQUENCY: #is the frequency above the maximum?
        STEP.freq(MAX_FREQUENCY)
        STEP.duty_u16(32767) #50% duty cycle
    
    else:
        STEP.freq(frequency)
        STEP.duty_u16(32767) #50% duty cycle

def control_stepper(direction,frequency):
#takes: direction (boolean value) and frequency (float)
#returns: nothing, but sets the direction and frequency of the stepper motor controller

    try:
        T = 1/frequency #convert the frequency to period.
    except:
        T = -1 #set the period to -1 if the frequency is 0 (divide by zero error)
    if T >= 0 and mot_enable == True: #if the motor is enabled, and the period is not -1 (-1 would imply an infinite period based on the above code)
        
        DIR.value(not direction)  # Set direction
        set_step_frequency(frequency) #Set the frequency of the motor controller through the function

def enable_stepper():
#this function will set the duty cycle of the motor to 50% and change the global variable so that other functions can tell the motor is enabled
    global mot_enable
    mot_enable = True
    STEP.duty_u16(32767) #50% duty cycle

def disable_stepper():
#this function will set the duty cycle of the motor to 0% and change the global variable so that other functions can tell the motor is disabled
    global mot_enable
    mot_enable = False
    STEP.duty_u16(0) #0% duty cycle

def main():

    #PID parameters
    kp = 115 #approaches instability at kp = 115
    ki = 0
    kd = 0

    #initial conditions
    enable_stepper()
    direct=1
    control_stepper(1,0)
    count = 0
    t_max = 10

    R = 90
    integral = 0
    E_prev = 0
    t0 = time.time_ns()
    t_previous = 0

    try:
        while E_STOP == False:
            
            #if the program is told to go to a non logical angle
            if R < 0 or R > 180:
                E_STOP = True
                print("Quitting, Invalid Target Angle")
            Y = read_ADC(DEBUG_MODE=False)
            E = R-Y
            integral += E

            t_now = float(time.time_ns()-t0)*10**-9
            U = E*kp + integral*ki + (E-E_prev)/(t_now-t_previous)*kd
            t_previous = t_now
            E_prev = E

            f = abs(U)
            direction = (U>0)
            
            print("{} {} {} {} {}".format(t_now,Y,E,U,R))
            
            control_stepper(direction,round(f))
            
            if t_now >= t_max*1/2:
                R = 135
            elif t_now >= t_max/4:
                R = 45
            
            if t_now >= t_max:
                E_STOP = True
            count += 1
            time.sleep(0.01)
        
            #if direct == 1:
            #    direct =0
            #else:
            #    direct = 1
    except KeyboardInterrupt:
        print("program killed")
        disable_stepper()
    finally:
        disable_stepper()

if __name__ == '__main__':
    main()