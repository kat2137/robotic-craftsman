# st_servo limits
LIMITS = {
    1: (2000, 4095),   # wrist rotate
    2: (2200, 3000),   # wrist tilt
}

for sid, (lo, hi) in LIMITS.items():
    packetHandler.unLockEprom(sid)
    packetHandler.write2ByteTxRx(sid, ADDR_MIN_ANGLE_LIMIT, lo)
    packetHandler.write2ByteTxRx(sid, ADDR_MAX_ANGLE_LIMIT, hi)
    packetHandler.LockEprom(sid)