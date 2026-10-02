from arm import ST_SERVOS

# st_servo limits
LIMITS = {cfg["id"]: (cfg["min"], cfg["max"]) for cfg in ST_SERVOS.values() if cfg["min"] is not None}


def main():
    # NOTE: packetHandler and the ADDR_* constants are not defined here (unchanged from before).
    for sid, (lo, hi) in LIMITS.items():
        packetHandler.unLockEprom(sid)
        packetHandler.write2ByteTxRx(sid, ADDR_MIN_ANGLE_LIMIT, lo)
        packetHandler.write2ByteTxRx(sid, ADDR_MAX_ANGLE_LIMIT, hi)
        packetHandler.LockEprom(sid)


if __name__ == "__main__":
    main()
