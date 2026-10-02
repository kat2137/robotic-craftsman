from arm import ST_PORT_LINUX, ST_BAUD, STBus


def main():
    bus = STBus()
    if not bus.open_port(ST_PORT_LINUX):
        raise SystemExit("failed to open port")
    if not bus.set_baud(ST_BAUD):
        raise SystemExit("failed to set baud")

    found = bus.ping_scan(range(1, 21))
    for sid, model, pos in found:
        print(f"ID {sid}  model {model}  pos {pos}")

    print(f"{len(found)} servo(s) found")
    bus.close()


if __name__ == "__main__":
    main()
