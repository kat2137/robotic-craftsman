
from arm import ST_PORT_LINUX, ST_BAUD, ST_SERVOS, STBus

# elbow is not calibrated yet, so it is not offered here
TUNABLE = ["wrist rotate", "wrist tilt"]


def main():
    bus = STBus()
    if not bus.connect(ST_PORT_LINUX, ST_BAUD):
        quit()
    servos = {name: bus.servo(name) for name in TUNABLE}

    while True:
        cmd = input(f"servo id:")
        if cmd == "q":
            bus.close()
            break
        accel = int(input(f"acceleration:"))
        pos = int(input(f"position:"))
        speed = int(input(f"speed:"))
        names = list(servos)
        if cmd in names:
            try:
                servos[cmd].move(pos, speed, accel)
                servos[cmd].wait()
            except ValueError:
                print("?")
        else:
            print (f"Error: servo id {cmd} not found. Valid ids are: {names}")


if __name__ == "__main__":
    main()
