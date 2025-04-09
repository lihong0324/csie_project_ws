import smbus
bus = smbus.SMBus(1)
for addr in range(0x03, 0x77):
    try:
        bus.write_quick(addr)
        print("Found device at address:", hex(addr))
    except:
        pass