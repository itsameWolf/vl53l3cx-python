import smbus2 as smbus

i2c = smbus.SMBus(1)

print("bus opened")
print(i2c)

