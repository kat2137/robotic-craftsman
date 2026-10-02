from arm import ST_PORT_LINUX, ST_BAUD, STBus


def main():
    bus = STBus()
    bus.open_port(ST_PORT_LINUX)
    bus.set_baud(ST_BAUD)
    bus.broadcast(2048, 300, 50)   # 254 = broadcast
    bus.close()


if __name__ == "__main__":
    main()
