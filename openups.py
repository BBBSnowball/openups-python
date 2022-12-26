# License: LGPL (or MIT if you think that the parts that I have taken from mini-box/ups
# don't have the level of creativity to be relevant for copyright. All the relevant parts
# should be in the documentation of the USB protocol if there was any. However, I have
# also kept some of the names and code structure.)

import usb.core
import struct, time, math, sys
from enum import Enum

DEBUG = False

USB_ENDPOINT_IN	 = 0x80
USB_ENDPOINT_OUT = 0x00
USBHID_SEND_TIMEOUT = 1000
USBHID_RECV_TIMEOUT = 1000

OPENUPS_GET_ALL_VALUES = 0x81
OPENUPS_RECV_ALL_VALUES = 0x82
OPENUPS_GET_ALL_VALUES_2 = 0x85
OPENUPS_RECV_ALL_VALUES_2 = 0x86

ETA_CHG = 0.90

# https://github.com/mini-box/ups/blob/97a1ab2cc3ef0a2700d3ffc7658504210f86b4e1/src/devices.h#L13
MINIBOX_VENDOR_ID	= 0x04d8
DCDCUSB_PRODUCT_ID 	= 0xd003
OPENUPS_PRODUCT_ID	= 0xd004
OPENUPS2_PRODUCT_ID	= 0xd005
NUCUPS_PRODUCT_ID	= 0xd007

OPENUPS_DEVICE_IDS = [
	{
		"name": "dcdcusb",
		"desc": "DCDC USB",
		"vendorId": MINIBOX_VENDOR_ID,
		"productId": DCDCUSB_PRODUCT_ID,
		"max_transfer_size": 24,
		"cmds": [{ "cmd": "set_vout", "desc": "Set Output Voltage" }]
	},
	{
		"name": "openups",
		"desc": "Open UPS",
		"vendorId": MINIBOX_VENDOR_ID,
		"productId": OPENUPS_PRODUCT_ID,
		"max_transfer_size": 32,
		"cmds": []
	},
	{
		"name": "openups2",
		"desc": "Open UPS2",
		"vendorId": MINIBOX_VENDOR_ID,
		"productId": OPENUPS2_PRODUCT_ID,
		"max_transfer_size": 32,
		"cmds": []
	},
	{
		"name": "nucups",
		"desc": "DC-DC NUC UPS",
		"vendorId": MINIBOX_VENDOR_ID,
		"productId": NUCUPS_PRODUCT_ID,
		"max_transfer_size": 32,
		"cmds": []
	}
]

class OpenUPSTimeValue(object):
	__slots__ = ("value",)

	def __init__(self, value):
		self.value = value

	@classmethod
	def from_hms(cls, hours, minutes, seconds):
		return cls(seconds + minutes*60 + hours*3600)

	@property
	def is_never(self):
		return self.value == 0xffff

	@property
	def hours(self):
		return self.value / 3600

	@property
	def minutes(self):
		return (self.value / 60) % 60

	@property
	def seconds(self):
		return self.value % 60

	def __repr__(self):
		if self.is_never():
			return "OpenUPSTimeValue.NEVER"
		else:
			return "OpenUPSTimeValue.from_hms(%d, %d, %d)" % (self.hours, self.minutes, self.seconds)

	def __str__(self):
		if self.is_never():
			return "never"
		else:
			return "%02d:%02d:%02d" % (self.hours, self.minutes, self.seconds)

OpenUPSTimeValue.NEVER = OpenUPSTimeValue(0xffff)

# https://github.com/mini-box/ups/blob/97a1ab2cc3ef0a2700d3ffc7658504210f86b4e1/src/lib/HIDOpenUPS.cpp#L13
g_OpenUPS_memConstants = (
	0.03545, #Vin				 #0
	0.02571, #Vout				 #1
	0.00857, #Vbat+chg+cell		 #2
	12.411,  #A chg Hi range	 #3
	0.8274,  #A chg Low range	 #4
	16.113	 #A discharge		 #5
)


g_OpenUPS_memTerm = (
	0x31,0x40,0x53,0x68,0x82,
	0xA0,0xC3,0xE9,0x113,0x13F,
	0x16E,0x19F,0x1CF,0x200,0x22F,
	0x25C,0x286,0x2AE,0x2D3,0x2F4,
	0x312,0x32D,0x345,0x35A,0x36D,
	0x37E,0x38C,0x399,0x3A5,0x3AF,
	0x3B7,0x3BF,0x3C6,0x3CC
)

def convertThermistorValue(raw):
	pos = len(g_OpenUPS_memTerm)-1
	for x in reversed(g_OpenUPS_memTerm):
		if raw >= x:
			break
		pos -= 1
	if pos >= len(g_OpenUPS_memTerm)-1:
		return 125
	elif pos < 0:
		return -40
	else:
		t1 = pos*5-40
		t2 = (pos+1)*5-40
		if raw == g_OpenUPS_memTerm[pos]:
			return t1

		d1 = g_OpenUPS_memTerm[pos]
		d2 = g_OpenUPS_memTerm[pos+1]

		dtemp = (raw - d1)*(t2-t1)*1.0/(d2-d1)
		
		return math.ceil(dtemp) + t1

class OpenUPSPowerState(Enum):
	batpowered = 1
	vinpowered = 2
	usbonly = 3

class OpenUPSStatus(object):
	def set_from(self, dev, msg):
		if DEBUG:
			#sys.stderr.write(repr(list(msg)) + "\n")
			sys.stderr.write(", ".join("%02x"%x for x in msg) + "\n")

		if dev.idProduct != OPENUPS_PRODUCT_ID:
			raise Exception("not supported because I don't have this device to test it")

		if msg[0] == OPENUPS_RECV_ALL_VALUES and False:
			# see https://github.com/mini-box/ups/blob/97a1ab2cc3ef0a2700d3ffc7658504210f86b4e1/src/lib/HIDDCDCUSB.cpp#L183
			# -> This would be for DCDC, not OpenUPS - I cannot test this.
			self.msg = msg
			self.nTimeCfg = (msg[1] >> 5) & 0x7
			self.nVoltageCfg = (msg[1] >> 2) & 0x7
			self.mode = msg[1] & 3
			self.mode_str = ("Dumb", "Automotive", "Script", "UPS")[self.mode]
			self.state = msg[2]
			if self.state == 0:
				self.state_str = "offline"
			elif self.state == 1:
				self.state_str = "usb"
			elif self.state == 2:
				self.state_str = "batpowered"
			elif self.state == 3:
				self.state_str = "vinpowered"
			else:
				self.state_str = "%d" % self.state
			self.volt_in = msg[3] * 0.1558
			self.volt_ign = msg[4] * 0.1558
			self.volt_out = msg[5] * 0.1170

			self.flagsStatus1 = msg[6]
			self.flagsStatus2 = msg[7]
			self.flagsVoltage = msg[8]
			self.flagsTimer   = msg[9]

			self.PowerSwitch = bool(self.flagsStatus1 & 0x4)
			self.Output = bool(self.flagsStatus1 & 0x8)
			self.AuxVin = bool(self.flagsStatus1 & 0x10)

			self.ScriptPointer = msg[10]

			self.m_strTimerWait = OpenUPSTimeValue(struct.unpack_from(">H", msg, offset=11))
			self.m_strTimerVout = OpenUPSTimeValue(struct.unpack_from(">H", msg, offset=13))
			self.m_strTimerVAux = OpenUPSTimeValue(struct.unpack_from(">H", msg, offset=15))
			self.m_strTimerPwSwitch = OpenUPSTimeValue(struct.unpack_from(">H", msg, offset=17))
			self.m_strTimerOffDelay = OpenUPSTimeValue(struct.unpack_from(">H", msg, offset=19))
			self.m_strTimerHardOff = OpenUPSTimeValue(struct.unpack_from(">H", msg, offset=21))

			#self.version = msg[23]
			self.versionMajor = (msg[23] >> 5) & 0x07
			self.versionMinor = msg[23] & 0x1F
			self.version = "%d.%d" % (self.versionMajor, self.versionMinor)

		elif msg[0] == OPENUPS_RECV_ALL_VALUES:
			# see https://github.com/mini-box/ups/blob/97a1ab2cc3ef0a2700d3ffc7658504210f86b4e1/src/lib/HIDOpenUPS.cpp#L289
			self.versionMajor = (msg[31] >> 4) & 0x0F
			self.versionMinor = msg[31] & 0x0F
			self.version = "%d.%d" % (self.versionMajor, self.versionMinor)

			xs = struct.unpack_from("<HHHHHHHHHHH", msg, 1)
			self.volt_in  = xs[0] * g_OpenUPS_memConstants[0]
			self.volt_out = xs[1] * g_OpenUPS_memConstants[1]
			self.volt_bat = xs[2] * g_OpenUPS_memConstants[2]
			self.volt_cells = (x * g_OpenUPS_memConstants[2] for x in xs[3:3+6])

			#self.m_nOtherState[4] = msg[23]
			#self.m_nOtherState[5] = msg[24]
			#self.m_nOtherState[6] = msg[25]
			self.m_nOtherState = (0, 0, 0) + tuple(msg[23:26])

			self.current_charge = xs[9] * g_OpenUPS_memConstants[4]/1000.0
			
			if (msg[24]&(1<<6)) != 0:
				self.state = OpenUPSPowerState.batpowered
				
				self.current_discharge = xs[10] * g_OpenUPS_memConstants[5]/1000.0
				self.current_in = 0
			elif (msg[24]&(1<<5)) != 0:
				self.state = OpenUPSPowerState.vinpowered

				self.outbuckboost_input_current = xs[10] * g_OpenUPS_memConstants[5]/1000.0

				self.current_in = self.current_charge * self.volt_bat / (self.volt_in * ETA_CHG) + self.outbuckboost_input_current
				self.current_discharge = 0
			else:
				#only usb
				self.state = OpenUPSPowerState.usb

				self.current_discharge = 0
				self.current_in = 0

			self.temperature = convertThermistorValue(struct.unpack_from("<H", msg, 26)[0])

		elif msg[0] == OPENUPS_RECV_ALL_VALUES_2:
			self.outputPower = struct.unpack_from("<I", msg, 1)[0] * 0.000001

		else:
			raise Exception("wrong type of reply: 0x%02x instead of 0x%02x" % (msg[0], OPENUPS_RECV_ALL_VALUES))

	def __repr__(self):
		return "OpenUPSStatus(state=%s (%d), vin=%.02fV, vbat=%.02fV, vout=%.02fV, c_in=%.02fA, c_charge=%.02fA, c_discharge=%.02fA, temp=%.02f degC)" % (
			self.state.name, self.state.value, self.volt_in, self.volt_bat, self.volt_out,
			self.current_in, self.current_charge, self.current_discharge,
			self.temperature)

	def print_for_shell(self, prefix):
		print("%sSTATE=%d" % (prefix, self.state))
		print("%sSTATE_STR=%s" % (prefix, self.state_str))
		print("%sVOLT_IN_mV=%d" % (prefix, int(self.volt_in * 1000)))
		print("%sVOLT_BAT_mV=%d" % (prefix, int(self.volt_bat * 1000)))
		print("%sVOLT_OUT_mV=%d" % (prefix, int(self.volt_out * 1000)))
		print("%sCURRENT_IN_mA=%d" % (prefix, int(self.current_in * 1000)))
		print("%sCURRENT_CHARGE_mA=%d" % (prefix, int(self.current_charge * 1000)))
		print("%sCURRENT_DISCHARGE_mA=%d" % (prefix, int(self.current_discharge * 1000)))
		print("%sTEMPERATURE_CELSIUS=%d" % (prefix, int(self.temperature)))

class OpenUPS(object):
	def __init__(self, dev=None):
		if dev is None:
			for info in OPENUPS_DEVICE_IDS:
				self.dev = dev = usb.core.find(idVendor=info["vendorId"], idProduct=info["productId"])
				if self.dev:
					self.info = info
					break
			if not dev:
				raise Exception("device not found")
		dev.set_configuration()
		cfg = dev.get_active_configuration()
		self.intf = cfg[(0,0)]
		self.intf.set_altsetting()

		#self.ep_out = None
		#self.ep_in = None
		#for ep in self.intf.endpoints():
		#	if ep.bEndpointAddress == USB_ENDPOINT_OUT+1:
		#		self.ep_out = ep
		#	elif ep.bEndpointAddress == USB_ENDPOINT_IN+1:
		#		self.ep_in = ep
		#	else:
		#		print("ignoring endpoint 0x%x: %r" % (ep.bEndpointAddress, ep))
		#if not self.ep_out or not self.ep_in:
		#	raise Exception("endpoint missing on USB device")

	def __repr__(self):
		return "OpenUPS(%r)" % (self.dev,)

	def writeInterrupt(self, bytes, use_transfer_size):
		max_transfer_size = self.info["max_transfer_size"]
		if use_transfer_size and len(bytes) < max_transfer_size:
			bytes += b"\0"*(max_transfer_size - len(bytes))
		#self.dev.interruptWrite(USB_ENDPOINT_OUT+1, bytes, USBHID_SEND_TIMEOUT)
		#self.ep_out.write(bytes, timeout=USBHID_SEND_TIMEOUT)
		self.dev.write(USB_ENDPOINT_OUT+1, bytes, USBHID_SEND_TIMEOUT)

	def sendMessage(self, *_bytes):
		_bytes = bytes(_bytes)
		self.writeInterrupt(_bytes, True)

	def readInterrupt(self):
		#return self.dev.interruptRead(USB_ENDPOINT_IN + 1, self.info["max_transfer_size"], USBHID_RECV_TIMEOUT)
		#return self.ep_in.read(self.info["max_transfer_size"], timeout=USBHID_RECV_TIMEOUT)
		return self.dev.read(USB_ENDPOINT_IN + 1, self.info["max_transfer_size"], timeout=USBHID_RECV_TIMEOUT)

	def getStatus(self, status=OpenUPSStatus()):
		self.sendMessage(OPENUPS_GET_ALL_VALUES, 0)
		time.sleep(0.001)
		reply = self.readInterrupt()
		if reply[0] != OPENUPS_RECV_ALL_VALUES:
			print(repr(list(reply)))
			self.sendMessage(OPENUPS_GET_ALL_VALUES, 0)
			reply = self.readInterrupt()
		status.set_from(self.dev, reply)
		time.sleep(0.001)

		self.sendMessage(OPENUPS_GET_ALL_VALUES_2, 0)
		time.sleep(0.001)
		reply = self.readInterrupt()
		if reply[0] != OPENUPS_RECV_ALL_VALUES_2:
			print(repr(list(reply)))
			self.sendMessage(OPENUPS_GET_ALL_VALUES_2, 0)
			reply = self.readInterrupt()
		status.set_from(self.dev, reply)

		return status

if __name__ == "__main__":
	if len(sys.argv) == 1:
		print(repr(OpenUPS().getStatus()))
	elif len(sys.argv) == 2 and sys.argv[1] == "--shell":
		OpenUPS().getStatus().print_for_shell("")
	elif len(sys.argv) == 3 and sys.argv[1] == "--shell":
		OpenUPS().getStatus().print_for_shell(sys.argv[2])
	elif len(sys.argv) == 2 and sys.argv[1] == "--monitor":
		ups = OpenUPS()
		status = OpenUPSStatus()
		while True:
			status = ups.getStatus(status)
			print(repr(status))
			time.sleep(1)
	else:
		raise Exception("invalid arguments")
