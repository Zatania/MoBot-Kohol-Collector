import serial
import time
import smbus
import math
import sys
import RPi.GPIO as io

##### GLOBAL VARIABLES #####

# compass values
desired_heading = 0
compass_heading = 0
compass_dev = 5
HEADING_A = 0
HEADING_B = 0
check_pass = 0
X_OFFSET = 397
Y_OFFSET = 654

# bluetooth values
blueToothVal = 0

# GPS values
GPS_Course = 0
Distance_To_Home = 0 # variable to store distance to destination
ac = 0 # gps array counter
waypoints_count = 0 # gps waypoint counter
Home_LATarray = [0] * 50 # variable to store destination latitude
Home_LONarray = [0] * 50 # variable to store destination longitude
increment = 0

###########################

######## SERIAL DECLARATIONS ########

# Bluetooth serial connection
bluetooth = serial.Serial('/dev/ttyUSB0', 9600)
time.sleep(2)
""" # Arduino serial connection
arduino = serial.Serial('/dev/ttyACM0', 9600)
time.sleep(2) """

#####################################

############### COMPASS DECLARATIONS #################

# HMC5883L address
ADDRESS = 0x1E

# Register addresses
CONFIG_A = 0x00
CONFIG_B = 0x01
MODE = 0x02
X_MSB = 0x03
Z_MSB = 0x05
Y_MSB = 0x07

# Initialize I2C bus
bus = smbus.SMBus(1)  # For Raspberry Pi 2/3, use bus 1

# Write to configuration registers to set sample rate and gain
bus.write_byte_data(ADDRESS, CONFIG_A, 0x70)  # Set to 8 samples @ 15Hz
bus.write_byte_data(ADDRESS, CONFIG_B, 0x20)  # 1.3 gain LSb / Gauss 1090 (default)
bus.write_byte_data(ADDRESS, MODE, 0x00)  # Continuous measurement mode

#####################################################

########## GPS DECLARATIONS ####################
ser = serial.Serial ("/dev/ttyUSB1")
gpgga_info = "$GPGGA,"
GPGGA_buffer = 0
NMEA_buff = 0

############## STEERING FUNCTIONS ########################

# Close all serial
""" def close():
    arduino.close()
    print("Arduino Serial Closed")
# Set Speed
def set_speed(speed):
    arduino.write(bytes(f"SP{speed}\n", 'utf-8'))
    print("Set Speed")
# Set Turn Speed
def set_turn_speed(speed):
    arduino.write(bytes(f"TSP{speed}\n", 'utf-8'))
    print("Set Turn Speed")
# Send Forward Command
def forward():
    arduino.write(b"F\n")
    print("Forward")
# Send Reverse Command
def reverse():
    arduino.write(b"B\n")
    print("Reverse")
# Send Left Command
def left():
    arduino.write(b"L\n")
    print("Left")
# Send Slow Left Command
def slowLeft():
    arduino.write(b"l\n")
    print("Slow Left")
# Send Right Command
def right():
    arduino.write(b"R\n")
    print("Right")
# Send Slow Right Command
def slowRight():
    arduino.write(b"r\n")
    print("Slow Right")
# Send Stop Command
def stop():
    arduino.write(b"S\n")
    print("Stop Car") """

##### BT7690 STEERING ##################
io.setmode(io.BCM)
io.setwarnings(False)

PWM_MAX = 100

######## BT7690 ##################
# LEFT MOTOR
L_L_EN = 21 # leftmotor_in1_pin
L_R_EN = 20 # leftmotor_in2_pin
L_L_PWM = 12 # leftmotorpwm_pin_l
L_R_PWM = 16 # leftmotorpwm_pin_r

# RIGHT MOTOR
R_L_EN = 26 # rightmotor_in1_pin
R_R_EN = 19 # rightmotor_in2_pin
R_L_PWM = 13 # rightmotorpwm_pin_l
R_R_PWM = 6 # rightmotorpwm_pin_r

leftmotor_in1_pin = L_L_EN
leftmotor_in2_pin = L_R_EN
rightmotor_in1_pin = R_L_EN
rightmotor_in2_pin = R_R_EN

io.setup(leftmotor_in1_pin, io.OUT)
io.setup(leftmotor_in2_pin, io.OUT)
io.setup(rightmotor_in1_pin, io.OUT)
io.setup(rightmotor_in2_pin, io.OUT)

io.output(leftmotor_in1_pin, True)
io.output(leftmotor_in2_pin, True)
io.output(rightmotor_in1_pin, True)
io.output(rightmotor_in2_pin, True)

leftmotorpwm_pin_l = L_L_PWM 
leftmotorpwm_pin_r = L_R_PWM
rightmotorpwm_pin_l = R_L_PWM
rightmotorpwm_pin_r = R_R_PWM

io.setup(leftmotorpwm_pin_l, io.OUT)
io.setup(leftmotorpwm_pin_r, io.OUT)
io.setup(rightmotorpwm_pin_l, io.OUT)
io.setup(rightmotorpwm_pin_r, io.OUT)

leftmotorpwm_l = io.PWM(leftmotorpwm_pin_l,100)
leftmotorpwm_r = io.PWM(leftmotorpwm_pin_r,100)
rightmotorpwm_l = io.PWM(rightmotorpwm_pin_l,100)
rightmotorpwm_r = io.PWM(rightmotorpwm_pin_r,100)

leftmotorpwm_l.start(0)
leftmotorpwm_r.start(0)
rightmotorpwm_l.start(0)
rightmotorpwm_r.start(0)

leftmotorpwm_l.ChangeDutyCycle(0)
leftmotorpwm_r.ChangeDutyCycle(0)
rightmotorpwm_l.ChangeDutyCycle(0)
rightmotorpwm_r.ChangeDutyCycle(0)
############################################

def setMotorLeft(power):
   int(power)
   if power < 0:
      # Rueckwaertsmodus fuer den linken Motor
      #setMotorMode("leftmotor", "reverse")
      pwm = -int(PWM_MAX * power)
      if pwm > PWM_MAX:
         pwm = PWM_MAX
      leftmotorpwm_l.ChangeDutyCycle(pwm)
      leftmotorpwm_r.ChangeDutyCycle(0)	  
   elif power > 0:
      # Vorwaertsmodus fuer den linken Motor
      #setMotorMode("leftmotor", "forward")
      pwm = int(PWM_MAX * power)
      if pwm > PWM_MAX:
         pwm = PWM_MAX
      leftmotorpwm_l.ChangeDutyCycle(0)
      leftmotorpwm_r.ChangeDutyCycle(pwm)
   else:
      # Stoppmodus fuer den linken Motor
      leftmotorpwm_l.ChangeDutyCycle(0)
      leftmotorpwm_r.ChangeDutyCycle(0)

def setMotorRight(power):
   int(power)
   if power < 0:
      # Rueckwaertsmodus fuer den rechten Motor
      #setMotorMode("rightmotor", "reverse")
      pwm = -int(PWM_MAX * power)
      if pwm > PWM_MAX:
         pwm = PWM_MAX
      rightmotorpwm_l.ChangeDutyCycle(pwm)
      rightmotorpwm_r.ChangeDutyCycle(0)
   elif power > 0:
      # Vorwaertsmodus fuer den rechten Motor
      #setMotorMode("rightmotor", "forward")
      pwm = int(PWM_MAX * power)
      if pwm > PWM_MAX:
         pwm = PWM_MAX
      rightmotorpwm_l.ChangeDutyCycle(0)
      rightmotorpwm_r.ChangeDutyCycle(pwm)
   else:
      # Stoppmodus fuer den rechten Motor
      rightmotorpwm_l.ChangeDutyCycle(0)
      rightmotorpwm_r.ChangeDutyCycle(0)

def forward():
    setMotorLeft(1)
    setMotorRight(1)
    print("FORWARD")
def reverse():
    setMotorLeft(-1)
    setMotorRight(-1)
    print("BACKWARD")
def left():
    setMotorLeft(-1)
    setMotorRight(1)
    print("LEFT")
def right():
    setMotorLeft(1)
    setMotorRight(-1)
    print("RIGHT")
def stop():
    setMotorLeft(0)
    setMotorRight(0)
    print("STOP")

################################3
# Send Compass Turn Right Command
def compass_right():
    global desired_heading, compass_heading, blueToothVal, compass_dev
    stop()
    compass_heading = get_compass()
    desired_heading = (desired_heading + 90)

    if desired_heading >= 360:
        desired_heading -= 360

    while abs(desired_heading - compass_heading) >= compass_dev:
        compass_heading = get_compass()
        bluetooth()
        x = (desired_heading - 359)
        y = (desired_heading - (x))
        z = (y - 360)

        if ((z <= 180) and (z>=0)):
            slowLeft()
        else:
            slowRight()
    stop()
# Send Compass Turn Left Command
def compass_left():
    global desired_heading, compass_heading, blueToothVal, compass_dev
    stop()
    compass_heading = get_compass()
    desired_heading = (desired_heading - 90)

    if desired_heading < 0:
        desired_heading += 360

    while abs(desired_heading - compass_heading) >= compass_dev:
        compass_heading = get_compass()
        bluetooth()
        x = (desired_heading - 359)
        y = (desired_heading - (x))
        z = (y - 360)

        if z <= 180:
            slowLeft()
        else:
            slowRight()
    stop()
# Send Compass Forward Command
def compass_forward():
    global desired_heading, compass_heading, blueToothVal, compass_dev
    while blueToothVal == 9:
        compass_heading = get_compass()
        bluetooth()
        if blueToothVal == 5:
            break
        
        if abs(desired_heading - compass_heading) <= compass_dev:
            forward()
        else:
            x = desired_heading - 359
            y = desired_heading - x
            z = y - 360

            if z <= 180 and z>= 0:
                slowLeft()
                #obstacle detection
            else:
                slowRight()
                #obstacle detection

###############################################################

######### GET INFO #################
def get_info():
    latitude, longitude = get_position()
    heading = get_compass()

    bluetooth.write(bytes(f"Latitude: {latitude:.5f}, Longitude: {longitude:.5f}, Heading: {heading:.2f}°\n", 'utf-8'))

########## BLUETOOTH FUNCTIONS #####################

def bluetooth_commands():
    global blueToothVal, ac
    if bluetooth.in_waiting > 0:
        blueData = bluetooth.readline().decode().rstrip('\n')
        bluetooth.reset_input_buffer()

        #For debugging purposes
        print("Received Bluetooth Data:", blueData)

        # Convert blueData to integer
        blueToothVal = int(blueData)

        #For debugging purposes
        print("BlueTooth Value:", blueToothVal)

        if blueToothVal == 1:
            forward()
        elif blueToothVal == 2:
            reverse()
        elif blueToothVal == 3:
            left()
        elif blueToothVal == 4:
            right()
        elif blueToothVal == 5:
            stop()
        elif blueToothVal == 6:
            print("Set Waypoint")
            # Call function to set waypoint
            set_waypoint()
        elif blueToothVal == 7:
            print("Go Waypoint")
            # Call function to go to waypoint
            go_waypoint()
        elif blueToothVal == 8:
            print("Turn Around")
            # Call function to turn around
            # Function Removed
        elif blueToothVal == 9:
            print("Compass Forward")
            # Call function to move forward according to compass
            # Function Removed
        elif blueToothVal == 10:
            print("Set Heading")
            # Call function to set heading
        elif blueToothVal == 11:
            print("Get GPS Info")
            # Call function to get GPS info
            # Function Removed
            get_info()
        elif blueToothVal == 12:
            print("Compass Turn Right")
            # Call function to turn right according to compass
            # Function Removed
        elif blueToothVal == 13:
            print("Compass Turn Left")
            # Call function to turn left according to compass
            # Function Removed
        elif blueToothVal == 14:
            print("Calibrate Compass")
            # Call function to calibrate compass
            # Function Removed
        elif blueToothVal == 15:
            print("Ping Toggle")
            # Call function to toggle ping sensor
        elif blueToothVal == 16:
            print("Clear Waypoints")
            # Call function to clear waypoints
            clear_waypoints()
        elif blueToothVal == 17:
            print("Waypoints Complete")
            ac = 0
            # Call function to check if waypoints are complete

        # Slider value for turn speed
        if blueToothVal >= 1000:
            speed = blueToothVal - 1000
            print("Turn Speed set to:", speed)
            # Call function to set speed
            stop()
            set_turn_speed(speed)
        
        # Slider value for speed
        if blueToothVal >= 2000:
            speed = blueToothVal - 2000
            print("Speed set to:", speed)
            # Call function to set speed
            stop()
            set_speed(speed)

##########################################

################ COMPASS AND GPS FUNCTIONS ####################

# Function to read signed 16-bit value (little endian)
def read_raw_data(addr):
    # Read raw 16-bit value
    high = bus.read_byte_data(ADDRESS, addr)
    low = bus.read_byte_data(ADDRESS, addr+1)
    
    # Combine them to get a 16-bit value
    value = (high << 8) + low
    if value > 32768:  # Adjust for 2's complement
        value = value - 65536
    return value
 
def compute_heading(x, y):
    # Calculate heading in radians
    heading_rad = math.atan2(y, x)
    
    # Adjust for declination angle (e.g. 0.22 for ~13 degrees)
    declination_angle = 0.22
    heading_rad += declination_angle
    
    # Correct for when signs are reversed.
    if heading_rad < 0:
        heading_rad += 2 * math.pi
 
    # Check for wrap due to addition of declination.
    if heading_rad > 2 * math.pi:
        heading_rad -= 2 * math.pi
 
    # Convert radians to degrees for readability.
    heading_deg = heading_rad * (180.0 / math.pi)
    
    return heading_deg

def get_compass():
    x = (read_raw_data(X_MSB)) * 0.92
    y = (read_raw_data(Y_MSB)) * 0.92
    z = read_raw_data(Z_MSB)

    heading = compute_heading(x,y)
    compass_heading = heading

    print(f"X: {x} uT, Y: {y} uT, Z: {z} uT, Heading: {heading:.2f}°")
    
    return compass_heading

def set_heading():
    global desired_heading, HEADING_A, HEADING_B, compass_heading
    for _ in range(6):
        compass_heading = get_compass()

    desired_heading = compass_heading  # Set the desired heading to equal the current compass heading
    HEADING_A = compass_heading  # Set Heading A to current compass heading
    HEADING_B = compass_heading + 180  # Set Heading B to current compass heading + 180

    if HEADING_B >= 360:  # Ensure heading is between 0 and 360 degrees
        HEADING_B -= 360

def print_waypoints():
    global waypoints_count, Home_LATarray, Home_LONarray
    if waypoints_count > 0:
        print("Printing Waypoints:")
        for i in range(waypoints_count):
            print("Waypoint", i, ": LAT =", Home_LATarray[i], ", LON =", Home_LONarray[i])
    else:
        print("No waypoints set yet.")

def set_waypoint():
    global waypoints_count, ac, Home_LATarray, Home_LONarray
    if waypoints_count >= 0:
        print("GPS WAYPOINT ", waypoints_count, " SET")
        latitude, longitude = get_position()

        Home_LATarray[waypoints_count] = latitude
        Home_LONarray[waypoints_count] = longitude

        #debugging
        print("HOME LAT: ", Home_LATarray[waypoints_count])
        print("HOME LON: ", Home_LONarray[waypoints_count])

        waypoints_count += 1
        ac += 1

        print_waypoints()
    else:
        print("WAYPOINTS FULL")

def clear_waypoints():
    global waypoints_count, ac, Home_LATarray, Home_LONarray
    Home_LATarray = [0] * len(Home_LATarray)
    Home_LONarray = [0] * len(Home_LONarray)

    waypoints_count = 0
    ac = 0

    print("GPS WAYPOINTS CLEARED")
    bluetooth.write(b"GPS WAYPOINTS CLEARED\n")

def go_waypoint():
    global ac, compass_heading, blueToothVal, Distance_To_Home, GPS_Course, Home_LATarray, Home_LONarray
    #debugging purposes
    print("GOING TO WAYPOINT ", ac)
    print("START COMPASS_HEADING = ", compass_heading)
    while True:
        bluetooth_commands()
        if blueToothVal == 5:
            break

        compass_heading = get_compass()
        print("NEW COMPASS_HEADING = ", compass_heading)
        latitude, longitude = get_position()

        #debugging
        print("Latitude: ", latitude)
        print("Longitude: ", longitude)
        print("Home_LATarray[ac]: ", Home_LATarray[ac])
        print("Home_LONarray[ac]: ", Home_LONarray[ac])
        Distance_To_Home = distance_between(latitude, longitude, Home_LATarray[ac], Home_LONarray[ac])
        GPS_Course = course_to(latitude, longitude, Home_LATarray[ac], Home_LONarray[ac])

        print("Distance to Home: ", Distance_To_Home)
        print("GPS Course: ", GPS_Course)

        if Distance_To_Home == 0:
            stop()
            print("Destination Reached")
            ac += 1
            break
        
        if abs(GPS_Course - compass_heading) <= 15:
            forward()
        else:
            x = (GPS_Course - 360)
            y = (compass_heading - x)
            z = (y - 360)

            if z <= 180 and z >= 0:
                slowLeft()
            else:
                slowRight()

###############################################################

############ GPS FUNCTIONS ####################

def distance_between(lat1, lon1, lat2, lon2):
    """
    Calculate the great circle distance between two points
    on the earth (specified in decimal degrees)
    """
    # Convert decimal degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    distance = 6371000 * c  # Radius of Earth in meters
    return distance

def course_to(lat1, lon1, lat2, lon2):
    """
    Calculate the initial bearing (course) between two points
    """
    # Convert decimal degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

    # Calculate the bearing
    dlon = lon2 - lon1
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    bearing = math.atan2(y, x)
    bearing = math.degrees(bearing)
    return (bearing + 360) % 360  # Normalize to range [0, 360)

#############################################################

########################## GPS POSITION ###########################

def convert_to_degrees(raw_value):
    decimal_value = raw_value / 100.00
    degrees = int(decimal_value)
    mm_mmmm = (decimal_value - int(decimal_value)) / 0.6
    position = degrees + mm_mmmm
    return position

def get_position():
    while True:
        received_data = str(ser.readline())  # read NMEA string received
        GPGGA_data_available = received_data.find(gpgga_info)  # check for NMEA GPGGA string
        if GPGGA_data_available > 0:
            GPGGA_buffer = received_data.split("$GPGGA,", 1)[1]  # store data coming after "$GPGGA,d" string
            NMEA_buff = GPGGA_buffer.split(',')
            nmea_latitude = NMEA_buff[1]  # extract latitude from GPGGA string
            nmea_longitude = NMEA_buff[3]  # extract longitude from GPGGA string
            latitude = float(nmea_latitude)
            latitude = convert_to_degrees(latitude)  # Round latitude to 3 decimals
            longitude = float(nmea_longitude)
            longitude = convert_to_degrees(longitude)  # Round longitude to 3 decimals
            print("Latitude:", latitude, "Longitude:", longitude, '\n')

            return latitude, longitude

##################################################################
if __name__ == "__main__":
    try:
        while True:
           bluetooth_commands()
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
        rpwm.stop()
        lpwm.stop()
        io.cleanup()
        exit()