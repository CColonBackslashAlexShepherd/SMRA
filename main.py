from machine import Pin,ADC,UART,PWM
import time
import ustruct
import _thread
import math

#ADC_SCALE_FACTOR = 6*270/65536
#ADC_ANGLE_OFFSET = 10#78.2
E_STOP = False

min_adc = 400
max_adc = 7121  # Assuming a 10-bit ADC
min_angle = 0
max_angle = 180  # Assuming a potentiometer with a 270-degree range

MAX_FREQUENCY = 10000

DIR = Pin(20,Pin.OUT)
STEP = PWM(Pin(21,Pin.OUT))
#minimum PWM frequency is 8
STEP.freq(8)
STEP.duty_ns
frequency = 0
direction = 1
mot_enable = False

# Configure ADC
pot = ADC(27)
# Configure serial communication
uart = UART(0, baudrate=9600, tx=None, rx=None)


def read_ADC(DEBUG_MODE=False):
    # Read ADC value
    adc_value = pot.read_u16()

    
    angle = min_angle + ((adc_value - min_adc) / (max_adc - min_adc)) * (max_angle - min_angle)
    #angle = adc_value*ADC_SCALE_FACTOR -ADC_ANGLE_OFFSET#converts 16 bit value into degrees

    if DEBUG_MODE == True:
        print("ADC RAW: {}, angle: {}".format(adc_value,angle))
    #stop the program if there's an issue with the pot
    if angle > 360:
        global E_STOP
        E_STOP = True
        print("Warning! Stopping due to angle: {} Degrees".format(angle))
        angle = 90 #safest angle for the control loop

    if angle < 0.01 and not DEBUG_MODE:
        angle = 0
    if angle > 180 and not DEBUG_MODE:
        angle = 180
    
    return angle

def set_step_frequency(frequency):
    if frequency < 8: #is the frequency below the minimum?
        STEP.freq(8)#minimum PWM value
        STEP.duty_u16(0) #0% duty cycle
    elif frequency > MAX_FREQUENCY:
        STEP.freq(MAX_FREQUENCY)
        STEP.duty_u16(32767)
    else:
        STEP.freq(frequency)
        STEP.duty_u16(32767) #50% duty cycle

#stepper motor control function. This will run in a different thread
def control_stepper(direction,frequency):

    #global direction, lock
    try:
        T = 1/frequency
    except:
        T = -1
    if T >= 0 and mot_enable == True:
        # Drive stepper motor forward
        DIR.value(not direction)  # Set direction
        set_step_frequency(frequency)

def enable_stepper():
    global mot_enable
    mot_enable = True
    STEP.duty_u16(32767)

def disable_stepper():
    global mot_enable
    mot_enable = False
    STEP.duty_u16(0)

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
        kp = 115 #approaches instability at kp = 115
        ki = 0#0.1
        kd = 0

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