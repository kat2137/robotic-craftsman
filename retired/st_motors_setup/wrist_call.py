from scservo_sdk import *
port = PortHandler('/dev/ttyACM0')
packet = sms_sts(port)
port.openPort()
port.setBaudRate(1000000)
packet.WritePosEx(0xFE, 2048, 300, 50)   # 254 = broadcast
port.closePort()