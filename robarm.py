#! /user/bin/python

import serial
from signal import pause
from time import sleep
import pigpio

# Wait until bluetooth device is connected
def bluetooth_status():
    while True:
        try:
            ser = serial.Serial("/dev/rfcomm0", baudrate=9600, timeout=3)
            break
        except:
            status = 'not connected'

bluetooth_status()
# Sleep time- Normal Mode
delay = .001
# Sleep time- Replay Mode
sPeed = [.001]
ser = serial.Serial(
    port='/dev/rfcomm0',
    baudrate=9600)
# Send Greeting and state to arduino device.
ser.write("Hi Boss, I am at your service :)\n".encode())
sleep(4)
ser.write("Manual Mode\n".encode())
# list to store "SAVE" Trigger
save = [0]

# List to store positions in "SAVE" Mode.
servo01Spos = []
servo02Spos = []
servo03Spos = []
servo04Spos = []
servo05Spos = []
servo06Spos = []

# Read postions from file.
with open('servo01Spos.txt', 'r') as filehandle:
    for i in filehandle:
        x = i
        servo01Spos.append(int(x))
with open('servo02Spos.txt', 'r') as filehandle:
    for i in filehandle:
        x = i
        servo02Spos.append(int(x))

with open('servo03Spos.txt', 'r') as filehandle:
    for i in filehandle:
        x = i
        servo03Spos.append(int(x))
with open('servo04Spos.txt', 'r') as filehandle:
    for i in filehandle:
        x = i
        servo04Spos.append(int(x))
with open('servo05Spos.txt', 'r') as filehandle:
    for i in filehandle:
        x = i
        servo05Spos.append(int(x))
with open('servo06Spos.txt', 'r') as filehandle:
    for i in filehandle:
        x = i
        servo06Spos.append(int(x))

# sevrvo motors GPIO PINs assignment
servo01 = 17
servo02 = 18
servo03 = 19
servo04 = 20
servo05 = 21
servo06 = 22
# "bluedot.btcomm BluetoothServer" Setup
pwm = pigpio.pi()
pwm.set_mode(servo01, pigpio.OUTPUT)
pwm.set_mode(servo02, pigpio.OUTPUT)
pwm.set_mode(servo03, pigpio.OUTPUT)
pwm.set_mode(servo04, pigpio.OUTPUT)
pwm.set_mode(servo05, pigpio.OUTPUT)
pwm.set_mode(servo06, pigpio.OUTPUT)

pwm.set_PWM_frequency(servo01, 50)
pwm.set_PWM_frequency(servo02, 50)
pwm.set_PWM_frequency(servo03, 50)
pwm.set_PWM_frequency(servo04, 50)
pwm.set_PWM_frequency(servo05, 50)
pwm.set_PWM_frequency(servo06, 50)
# Servos' initial positions
servo01ppos = 1000
servo02ppos = 1500
servo03ppos = 1000
servo04ppos = 1000
servo05ppos = 1000
servo06ppos = 1000
pwm.set_servo_pulsewidth(servo01, servo01ppos)
pwm.set_servo_pulsewidth(servo02, servo02ppos)
pwm.set_servo_pulsewidth(servo03, servo03ppos)
pwm.set_servo_pulsewidth(servo04, servo04ppos)
pwm.set_servo_pulsewidth(servo05, servo05ppos)
pwm.set_servo_pulsewidth(servo06, servo06ppos)
servo01pos = servo01ppos
servo02pos = servo02ppos
servo03pos = servo03ppos
servo04pos = servo04ppos
servo05pos = servo05ppos
servo06pos = servo06ppos

while 1:
    received_dataIn = ser.readline()
    sleep(.003)
    dataIn_left = ser.inWaiting()
    received_dataIn == ser.readline(dataIn_left)
    dataIn = str(received_dataIn)
    
    # Save mode on.
    if dataIn[2:-3] == "SAVE":
        save[0] = 1
        ser.write("Saving positions\n".encode())
    # Clear saved postions when RESET Pressed
    if dataIn[2:-3] == "RESET":
        save[0] = 0

        servo01Spos.clear()
        servo02Spos.clear()
        servo03Spos.clear()
        servo04Spos.clear()
        servo05Spos.clear()
        servo06Spos.clear()
        open('servo01Spos.txt', 'w').close()
        open('servo02Spos.txt', 'w').close()
        open('servo03Spos.txt', 'w').close()
        open('servo04Spos.txt', 'w').close()
        open('servo05Spos.txt', 'w').close()
        open('servo06Spos.txt', 'w').close()
        ser.write("All 'SAVED' positions cleared.".encode())
        sleep(2)
        ser.write("Manual Mode".encode())

    # Append postions to list - "save" once SAVE is pressed
    if save[0] == 1:
        servo01Spos.append(servo01ppos)
        servo02Spos.append(servo02ppos)
        servo03Spos.append(servo03ppos)
        servo04Spos.append(servo04ppos)
        servo05Spos.append(servo05ppos)
        servo06Spos.append(servo06ppos)

        # Write saved postions to file.
        with open('servo01Spos.txt', 'w') as filehandle:
            for i in servo01Spos:
                filehandle.write('%s\n' % i)
        with open('servo02Spos.txt', 'w') as filehandle:
            for i in servo02Spos:
                filehandle.write('%s\n' % i)
        with open('servo03Spos.txt', 'w') as filehandle:
            for i in servo03Spos:
                filehandle.write('%s\n' % i)
        with open('servo04Spos.txt', 'w') as filehandle:
            for i in servo04Spos:
                filehandle.write('%s\n' % i)
        with open('servo05Spos.txt', 'w') as filehandle:
            for i in servo05Spos:
                filehandle.write('%s\n' % i)
        with open('servo06Spos.txt', 'w') as filehandle:
            for i in servo06Spos:
                filehandle.write('%s\n' % i)
    # Extract servo positions from incoming serial dataIn amd map to 600 -2400
    if dataIn[2] == 's' and dataIn[3] != 's' and dataIn[3] != 0:
        angle_float = float(dataIn[4:-3])
        angle = int(angle_float)
        
    # servo01 move
    if dataIn[2:4] == "s1":
        servo01pos = angle
        if servo01ppos > servo01pos:
            j = servo01ppos
            while j >= servo01pos:
                    j -= 1
                    pwm.set_servo_pulsewidth(servo01, j)
                    sleep(delay)

        if servo01ppos < servo01pos:
            j = servo01ppos
            while j <= servo01pos:
                    j += 1
                    pwm.set_servo_pulsewidth(servo01, j)
                    sleep(delay)

    servo01ppos = servo01pos

    # servo02 move
    if dataIn[2:4] == "s2":
        servo02pos = angle
        if servo02ppos > servo02pos:
            j = servo02ppos
            while j >= servo02pos:
                    j -= 1
                    pwm.set_servo_pulsewidth(servo02, j)
                    sleep(delay)

        if servo02ppos < servo02pos:
            j = servo02ppos
            while j <= servo02pos:
                    j += 1
                    pwm.set_servo_pulsewidth(servo02, j)
                    sleep(delay)

    servo02ppos = servo02pos
    # servo03 move
    if dataIn[2:4] == "s3":
        servo03pos = angle
        if servo03ppos > servo03pos:
            j = servo03ppos
            while j >= servo03pos:
                    j -= 1
                    pwm.set_servo_pulsewidth(servo03, j)
                    sleep(delay)

        if servo03ppos < servo03pos:
            j = servo03ppos
            while j <= servo03pos:
                    j += 1
                    pwm.set_servo_pulsewidth(servo03, j)
                    sleep(delay)

    servo03ppos = servo03pos
    # servo04 move
    if dataIn[2:4] == "s4":
        servo04pos = angle
        if servo04ppos > servo04pos:
            j = servo04ppos
            while j >= servo04pos:
                    j -= 1
                    pwm.set_servo_pulsewidth(servo04, j)
                    sleep(delay)

        if servo04ppos < servo04pos:
            j = servo04ppos
            while j <= servo04pos:
                    j += 1
                    pwm.set_servo_pulsewidth(servo04, j)
                    sleep(delay)

    servo04ppos = servo04pos
    # servo05 move
    if dataIn[2:4] == "s5":
        servo05pos = angle
        if servo05ppos > servo05pos:
            j = servo05ppos
            while j >= servo05pos:
                    j -= 1
                    pwm.set_servo_pulsewidth(servo05, j)
                    sleep(delay)

        if servo05ppos < servo05pos:
            j = servo05ppos
            while j <= servo05pos:
                    j += 1
                    pwm.set_servo_pulsewidth(servo05, j)
                    sleep(delay)

    servo05ppos = servo05pos
    # servo06 move
    if dataIn[2:4] == "s6":
        servo06pos = angle
        if servo06ppos > servo06pos:
            j = servo06ppos
            while j >= servo06pos:
                    j -= 1
                    pwm.set_servo_pulsewidth(servo06, j)
                    sleep(delay)

        if servo06ppos < servo06pos:
            j = servo06ppos
            while j <= servo06pos:
                    j += 1
                    pwm.set_servo_pulsewidth(servo06, j)
                    sleep(delay)

    servo06ppos = servo06pos

    def auto_run():
        # Mapping serial dataIn values for speed control
        range_in_min = 0
        range_in_max = 100
        range_out_min = .003
        range_out_max = .0003
        
        ser.write("Running saved moves\n".encode())

        bluetoothserial = serial.Serial('/dev/rfcomm0',
                            baudrate=9600, timeout=.0001)
        
        # replay loop starts here.
        while True:
            pwm.set_servo_pulsewidth(servo01, servo01Spos[0])
            sleep(.03)
            pwm.set_servo_pulsewidth(servo02, servo02Spos[0])
            sleep(.03)
            pwm.set_servo_pulsewidth(servo03, servo03Spos[0])
            sleep(.03)
            pwm.set_servo_pulsewidth(servo04, servo04Spos[0])
            sleep(.03)
            pwm.set_servo_pulsewidth(servo05, servo05Spos[0])
            sleep(.03)
            pwm.set_servo_pulsewidth(servo06, servo06Spos[0])
            sleep(.03)
            
            u = len(servo01Spos) - 2
            for i in range(u):
                dataIn = str(bluetoothserial.readline())
                
                if servo01Spos[i] > servo01Spos[i+1]:
                    j = servo01Spos[i]
                    while j >= servo01Spos[i+1]:
                        data = str(bluetoothserial.readline())
                        if data[2:-3] == 'PAUSE':
                            while data[2:-3] != 'RUN':
                                data = str(bluetoothserial.readline())
                                if data[2:-3] == 'STOP':
                                    ser.write("Manual Mode\n".encode())
                                    
                                    return

                        # Speed control
                        if data[2:4] == 'ss' and len(data) > 3:
                            Delay = float(data[4:-3])
                            sPeed[0] = (Delay - range_in_min) *\
                            (range_out_max-range_out_min) /\
                            (range_in_max-range_in_min) +\
                            range_out_min
                        pwm.set_servo_pulsewidth(servo01, j)
                        j -= 1
                        sleep(sPeed[0])
                       
                if servo01Spos[i] < servo01Spos[i+1]:
                    j = servo01Spos[i]
                    while j <= servo01Spos[i+1]:
                        data = str(bluetoothserial.readline())
                        if data[2:-3] == 'PAUSE':
                            while data[2:-3] != 'RUN':
                                data = str(bluetoothserial.readline())
                                if data[2:-3] == 'STOP':
                                    ser.write("Manual Mode\n".encode())
                                    
                                    return

                        # Speed control
                        if data[2:4] == 'ss' and len(data) > 3:
                            Delay = float(data[4:-3])
                            sPeed[0] = (Delay - range_in_min) *\
                            (range_out_max-range_out_min) /\
                            (range_in_max-range_in_min) +\
                            range_out_min
                        pwm.set_servo_pulsewidth(servo01, j)
                        j += 1
                        sleep(sPeed[0])
                            
                if servo02Spos[i] > servo02Spos[i+1]:
                    j = servo02Spos[i]
                    while j >= servo02Spos[i+1]:
                        data = str(bluetoothserial.readline())
                        if data[2:-3] == 'PAUSE':
                            while data[2:-3] != 'RUN':
                                data = str(bluetoothserial.readline())
                                if data[2:-3] == 'STOP':
                                    ser.write("Manual Mode\n".encode())
                                    
                                    return

                        # Speed control
                        if data[2:4] == 'ss' and len(data) > 3:
                            Delay = float(data[4:-3])
                            sPeed[0] = (Delay - range_in_min) *\
                            (range_out_max-range_out_min) /\
                            (range_in_max-range_in_min) +\
                            range_out_min
                        pwm.set_servo_pulsewidth(servo02, j)
                        j -= 1
                        sleep(sPeed[0])
                        
                if servo02Spos[i] < servo02Spos[i+1]:
                    j = servo02Spos[i]
                    while j <= servo02Spos[i+1]:
                        data = str(bluetoothserial.readline())
                        if data[2:-3] == 'PAUSE':
                            while data[2:-3] != 'RUN':
                                data = str(bluetoothserial.readline())
                                if data[2:-3] == 'STOP':
                                    ser.write("Manual Mode\n".encode())
                                    
                                    return

                        # Speed control
                        if data[2:4] == 'ss' and len(data) > 3:
                            Delay = float(data[4:-3])
                            sPeed[0] = (Delay - range_in_min) *\
                            (range_out_max-range_out_min) /\
                            (range_in_max-range_in_min) +\
                            range_out_min
                        pwm.set_servo_pulsewidth(servo02, j)
                        j += 1
                        sleep(sPeed[0])
                        
                if servo03Spos[i] > servo03Spos[i+1]:
                    j = servo03Spos[i]
                    while j >= servo03Spos[i+1]:
                        data = str(bluetoothserial.readline())
                        if data[2:-3] == 'PAUSE':
                            while data[2:-3] != 'RUN':
                                data = str(bluetoothserial.readline())
                                if data[2:-3] == 'STOP':
                                    ser.write("Manual Mode\n".encode())
                                    
                                    return

                        # Speed control
                        if data[2:4] == 'ss' and len(data) > 3:
                            Delay = float(data[4:-3])
                            sPeed[0] = (Delay - range_in_min) *\
                            (range_out_max-range_out_min) /\
                            (range_in_max-range_in_min) +\
                            range_out_min 
                        pwm.set_servo_pulsewidth(servo03, j)
                        j -= 1
                        sleep(sPeed[0])
                        
                if servo03Spos[i] < servo03Spos[i+1]:
                    j = servo03Spos[i]
                    while j <= servo03Spos[i+1]:
                        data = str(bluetoothserial.readline())
                        if data[2:-3] == 'PAUSE':
                            while data[2:-3] != 'RUN':
                                data = str(bluetoothserial.readline())
                                if data[2:-3] == 'STOP':
                                    ser.write("Manual Mode\n".encode())
                                    
                                    return

                        # Speed control
                        if data[2:4] == 'ss' and len(data) > 3:
                            Delay = float(data[4:-3])
                            sPeed[0] = (Delay - range_in_min) *\
                            (range_out_max-range_out_min) /\
                            (range_in_max-range_in_min) +\
                            range_out_min
                        pwm.set_servo_pulsewidth(servo03, j)
                        j += 1
                        sleep(sPeed[0])
                        
                if servo04Spos[i] > servo04Spos[i+1]:
                    j = servo04Spos[i]
                    while j >= servo04Spos[i+1]:
                        data = str(bluetoothserial.readline())
                        if data[2:-3] == 'PAUSE':
                            while data[2:-3] != 'RUN':
                                data = str(bluetoothserial.readline())
                                if data[2:-3] == 'STOP':
                                    ser.write("Manual Mode\n".encode())
                                    
                                    return

                        # Speed control
                        if data[2:4] == 'ss' and len(data) > 3:
                            Delay = float(data[4:-3])
                            sPeed[0] = (Delay - range_in_min) *\
                            (range_out_max-range_out_min) /\
                            (range_in_max-range_in_min) +\
                            range_out_min
                        pwm.set_servo_pulsewidth(servo04, j)
                        j -= 1
                        sleep(sPeed[0])
                       
                if servo04Spos[i] < servo04Spos[i+1]:
                    j = servo04Spos[i]
                    while j <= servo04Spos[i+1]:
                        data = str(bluetoothserial.readline())
                        if data[2:-3] == 'PAUSE':
                            while data[2:-3] != 'RUN':
                                data = str(bluetoothserial.readline())
                                if data[2:-3] == 'STOP':
                                    ser.write("Manual Mode\n".encode())
                                    
                                    return

                        # Speed control
                        if data[2:4] == 'ss' and len(data) > 3:
                            Delay = float(data[4:-3])
                            sPeed[0] = (Delay - range_in_min) *\
                            (range_out_max-range_out_min) /\
                            (range_in_max-range_in_min) +\
                            range_out_min
                        pwm.set_servo_pulsewidth(servo04, j)
                        j += 1
                        sleep(sPeed[0])
                        
                if servo05Spos[i] > servo05Spos[i+1]:
                    j = servo05Spos[i]
                    while j >= servo05Spos[i+1]:
                        data = str(bluetoothserial.readline())
                        if data[2:-3] == 'PAUSE':
                            while data[2:-3] != 'RUN':
                                data = str(bluetoothserial.readline())
                                if data[2:-3] == 'STOP':
                                    ser.write("Manual Mode\n".encode())
                                    
                                    return

                        # Speed control
                        if data[2:4] == 'ss' and len(data) > 3:
                            Delay = float(data[4:-3])
                            sPeed[0] = (Delay - range_in_min) *\
                            (range_out_max-range_out_min) /\
                            (range_in_max-range_in_min) +\
                            range_out_min
                        pwm.set_servo_pulsewidth(servo05, j)
                        j -= 1
                        sleep(sPeed[0])
                       
                if servo05Spos[i] < servo05Spos[i+1]:
                    j = servo05Spos[i]
                    while j <= servo05Spos[i+1]:
                        data = str(bluetoothserial.readline())
                        if data[2:-3] == 'PAUSE':
                            while data[2:-3] != 'RUN':
                                data = str(bluetoothserial.readline())
                                if data[2:-3] == 'STOP':
                                    ser.write("Manual Mode\n".encode())
                                    
                                    return

                        # Speed control
                        if data[2:4] == 'ss' and len(data) > 3:
                            Delay = float(data[4:-3])
                            sPeed[0] = (Delay - range_in_min) *\
                            (range_out_max-range_out_min) /\
                            (range_in_max-range_in_min) +\
                            range_out_min
                        pwm.set_servo_pulsewidth(servo05, j)
                        j += 1
                        sleep(sPeed[0])
                        
                if servo06Spos[i] > servo06Spos[i+1]:
                    j = servo06Spos[i]
                    while j >= servo06Spos[i+1]:
                        data = str(bluetoothserial.readline())
                        if data[2:-3] == 'PAUSE':
                            while data[2:-3] != 'RUN':
                                data = str(bluetoothserial.readline())
                                if data[2:-3] == 'STOP':
                                    ser.write("Manual Mode\n".encode())
                                    
                                    return

                        # Speed control
                        if data[2:4] == 'ss' and len(data) > 3:
                            Delay = float(data[4:-3])
                            sPeed[0] = (Delay - range_in_min) *\
                            (range_out_max-range_out_min) /\
                            (range_in_max-range_in_min) +\
                            range_out_min
                        pwm.set_servo_pulsewidth(servo06, j)
                        j -= 1
                        sleep(sPeed[0])
                       

                if servo06Spos[i] < servo06Spos[i+1]:
                    j = servo06Spos[i]
                    while j <= servo06Spos[i+1]:
                        data = str(bluetoothserial.readline())
                        if data[2:-3] == 'PAUSE':
                            while data[2:-3] != 'RUN':
                                data = str(bluetoothserial.readline())
                                if data[2:-3] == 'STOP':
                                    ser.write("Manual Mode\n".encode())
                                    
                                    return

                        # Speed control
                        if data[2:4] == 'ss' and len(data) > 3:
                            Delay = float(data[4:-3])
                            sPeed[0] = (Delay - range_in_min) *\
                            (range_out_max-range_out_min) /\
                            (range_in_max-range_in_min) +\
                            range_out_min
                        pwm.set_servo_pulsewidth(servo06, j)
                        j += 1
                        sleep(sPeed[0])
                                                      
        return
    # When RUN is pressed check if moves are saved and call auto_run function.
    equal1 = 0
    equal2 = 0
    equal3 = 0
    equal4 = 0
    equal5 = 0
    equal6 = 0
    
    if dataIn[2:-3] == "RUN":
        if len(servo01Spos) == 0:
            ser.write("No Moves saved, please STOP, SAVE some and RUN".encode())

        for i in servo01Spos:
            if i != servo01Spos[0]:
                equal1 = 1
                break
        for i in servo02Spos:
            if i != servo02Spos[0]:
                equal2 = 1
                break
        for i in servo03Spos:
            if i != servo03Spos[0]:
                equal3 = 1
                break
        for i in servo04Spos:
            if i != servo04Spos[0]:
                equal4 = 1
        for i in servo05Spos:
            if i != servo05Spos[0]:
                equal5 = 1
        for i in servo06Spos:
            if i != servo06Spos[0]:
                equal6 = 1
                break

        if (equal1+equal2+equal3+equal4+equal5+equal6) > 0:
            equal1 = 0
            equal2 = 0
            save[0] = 0
            auto_run()
        else:
            ser.write("No Moves saved,please STOP, SAVE some and RUN".encode())

