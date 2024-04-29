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
        ground = "unsure"
        print("RGB(%d %d %d)" % (red, green, blue))
        if(20<red<150 and 140<green<215 and 20<blue<90):
                ground="floor"
        elif(200<red<300 and 100<green<250 and 20<blue<95):
                ground="red"
        elif(300<red<450 and 400<green<500 and 150<blue<210):
                ground="white"

        print(ground)
        return ground

getAndUpdateColour()