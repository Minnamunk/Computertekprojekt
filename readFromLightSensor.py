
import smbus
import time
bus = smbus.SMBus(1)


# ISL29125 address, 0x44(68)
# Select configuation-1register, 0x01(01)
# 0x0D(13) Operation: RGB, Range: 360 lux, Res: 16 Bits
bus.write_byte_data(0x44, 0x01, 0x0D)

time.sleep(1)

print("Reading colour values and displaying them in a new window\n")
def getAndUpdateColour():
    # Read the data from the sensor
    # Insert code here
    data = bus.read_i2c_block_data(0x44, 0x09, 6)
    # Convert the data to green, red and blue int values
    # Insert code here
    green = (data[1] *256) + data[0]
    red = (data[3] *256) + data[2]
    blue = (data[5] * 256) + data[4]
    blue = blue *1.65
    ground = "unsure"
    # Output data to the console RGB values
    # Uncomment the line below when you have read the red, green and blue values
    total = green + red + blue
    green_part = green / total
    blue_part = blue / total
    red_part = red / total
    print("RGB(%f %f %f)" % (red_part, green_part, blue_part))
    if(green_part > red_part and blue_part < green_part and green_part>= 0.4 and red_part <= 0.27):
        ground = "green"
    elif(blue_part > red_part and blue_part > green_part and blue_part>= 0.4):
        ground = "blue"
    elif(red_part > blue_part and red_part > green_part and red_part>= 0.4):
        ground = "red"
    print(ground)
    return ground



getAndUpdateColour()
