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
        data = bus.read_i2c_block_data(0x44, 0x09, 6)
        # Convert the data to green, red and blue int values
        # Insert code here
        green = (data[1] << 8) | data[0]
        red = (data[3] << 8) | data[2]
        blue = (data[5] << 8) | data[4]

        # Output data to the console RGB values
        # Uncomment the line below when you have read the red, green and blue values
        print("RGB(%d %d %d)" % (red, green, blue))
        if(20<red<100 and 40<green<175 and 20<blue<70):
                print("green")
        elif(100<red<400 and 40<green<250 and 10<blue<75):
                print("red")
        elif(40<red<100 and 70<green<210 and 80<blue<150):
                print("blue")

getAndUpdateColour()