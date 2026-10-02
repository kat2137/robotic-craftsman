from arm import FingerBoard, US_MID


def main():
    board = FingerBoard().connect()
    move_us = board.move_us

    ch = int(input("channel: "))
    us = US_MID
    move_us(ch, us)
    print("commands: number = go to that us | +N / -N = adjust | c = change channel | q = quit")
    while True:
        cmd = input(f"[ch {ch} @ {us} us] > ").strip()
        if cmd == "q":
            break
        elif cmd == "c":
            ch = int(input("channel: "))
        elif cmd.startswith(("+", "-")):
            us += int(cmd)
            move_us(ch, us)
        else:
            try:
                us = int(cmd)
                move_us(ch, us)
            except ValueError:
                print("?")


if __name__ == "__main__":
    main()
