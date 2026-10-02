from scservo_sdk import *

port = PortHandler('/dev/ttyACM0')
packet = sms_sts(port)

if not port.openPort():
    raise SystemExit("failed to open port")
if not port.setBaudRate(1000000):
    raise SystemExit("failed to set baud")

found = 0
for sid in range(1, 21):
    model, comm, err = packet.ping(sid)
    if comm == COMM_SUCCESS:
        pos, comm2, err2 = packet.ReadPos(sid)
        print(f"ID {sid}  model {model}  pos {pos}")
        found += 1

print(f"{found} servo(s) found")
port.closePort()
