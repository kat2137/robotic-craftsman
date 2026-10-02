import time
from arm import FingerBoard, US_MIN, US_MID, US_MAX

CHANNELS = [1, 2, 3, 4, 5, 6]


def main():
    board = FingerBoard().connect()
    move_us = board.move_us
    try:
        for ch in CHANNELS:
            print(f"\n--- channel {ch} ---")
            move_us(2, US_MID); time.sleep(2)
            move_us(2, US_MIN); time.sleep(2)
            move_us(2, US_MAX); time.sleep(2)
            move_us(2, US_MID); time.sleep(2)
        print("\ndone")
    except KeyboardInterrupt:
        for ch in CHANNELS:
            board.release_channel(ch)
        print("\nstopped")


if __name__ == "__main__":
    main()
