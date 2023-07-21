import serial

command = serial.Serial('/dev/ttyUSB0',4800, timeout=5 )

while True:
    input = ser.readLine()
    splitInputs = input.split(",")

    if splitInputs[0] == "$GPGGA":
        lat = splitInputs[2]
        lat_direction = splitInputs[3]
        long = splitInputs[4]
        long_direction = splitInputs[5]
        print("Latitude: {}{}, Longitude: {}{}".format(lat,lat_direction,long,long_direction))
