import pyvisa, serial
from warnings import warn
import os, sys, time, pdb, traceback

class LabInstrument(object):
	'''
	General lab instrument. Intended as a superclass for devices which are meant
	to be programmatically controlled through GPIB.

	This could technically be reworked for devices controlled over RS-232 as well, 
	but it would be a lot of rewriting functions.
	'''
	def __init__(self, rm):
		self.rm = rm
		self.rsrc = None
		self.is_open = False

	def open_prologix(self, ip_addr="192.168.1.4", gpib_addr=15):
		'''
		Inputs:
			ip_addr: String. The IP address of the Prologix.
			gpib_addr: Integer. The GPIB address associated
				with the device hooked up to the Prologix.
		Returns:
			None.
		Notes:
			Sets self.rsrc to the Prologix (which then passes
			on commands via Ethernet to the GPIB-controlled box).
		'''
		self.rsrc = self.rm.open_resource(f"TCPIP::{ip_addr}::1234::SOCKET")

		self.rsrc.write_termination = '\r\n' # Per last page of the manual
		self.rsrc.read_termination = '\r\n'

		self.rsrc.write("++mode 1") 			# Prologix controller mode
		self.rsrc.write(f"++addr {gpib_addr}")	# GPIB address
		self.rsrc.write("++auto 1")				# Automatic read after write for query
		self.rsrc.write("++read_tmo_ms 500")	# Extend read timeout
		self.rsrc.write("++eos 3") 				# No additional termination character
		self.rsrc.write("++eoi 1")				# Assert EOI after last byte
		self.rsrc.write("++eot_enable 0")		# No EOT character on EOI

		self.is_open = True

	def open_gpib(self):
		is_correct = False
		while not is_correct:
			# List resources for user selection by index
			num_rsrc = len(self.rm.list_resources())
			print('Index: Resource ID')
			for i, x in enumerate(self.rm.list_resources()):
				print(f'{i}: {x}')

			while True:
				idx_str = input('Choose Device Index: ')
				try:
					rsrc = self.rm.open_resource(self.rm.list_resources()[int(idx_str)])
					break
				except Exception as e:
					print(e)

			# Sanity checking that it's the correct device
			print(rsrc.query("*IDN?"))
			is_correct_raw = input('Is this the correct device? (y/n): ').lower()
			is_correct = (is_correct_raw == 'y')

			# If not correct, close the connection and prompt again
			if not is_correct:
				rsrc.close()
			else:
				self.rsrc = rsrc
				self.is_open = True

	def get_rsrc(self):
		return self.rsrc

	def get_rm(self):
		return self.rm

	def write(self, msg):
		if not self.is_open:
			warn('Attempting to write to a closed resource')
		return self.rsrc.write(msg)

	def query(self, msg):
		if not self.is_open:
			warn('Attempting to query a closed resource')
		return self.rsrc.query(msg)

	def close_prologix(self):
		if self.is_open and self.rsrc != None:
			self.rsrc.write("++mode 0") # Back to device control
		self.close()

	def close(self):
		if self.is_open:
			if self.rsrc != None:
				self.rsrc.close()
			self.is_open = False
		else:
			warn(f'Resource {self.rsrc} not open')

	def __str__(self):
		return f"{type(self).__name__} : {self.rsrc}"


class KeysightE3631A(LabInstrument):
	def close(self):
		self.rsrc.write(f'OUTP OFF')	# Turn off all outputs
		super().close()

class Keysight33500B(LabInstrument):
	def close(self):
		# Clear before closing connection
		self.rsrc.write("*RST")
		super().close()


class DG535(LabInstrument):
	def close(self):
		# Clear instrument
		self.rsrc.write('CL')

		super().close()

def run_main():
	######################################################
	### Scratch: Experimenting w/ Connection to E3631A ###
	######################################################
	if False:
		rm = pyvisa.ResourceManager()
		psu = KeysightE3631A(rm)
		psu.open_prologix(ip_addr="192.168.1.14", gpib_addr=1)

		print(f"ID String: {psu.query('*IDN?')}")
		print(f"Self Test: {psu.query('*TST?')}")

		cmd_lst = []

		for cmd in cmd_lst:
			psu.write(cmd)

		psu.close()

	#####################################################
	### Scratch: Experimenting w/ Connection to DG535 ###
	#####################################################
	if True:
		# Connect to DG535
		print("Connecting...")
		# Connect to DG535
		rm = pyvisa.ResourceManager()
		dg535 = DG535(rm)
		dg535.open_prologix(ip_addr="192.168.1.108", gpib_addr=15)

		print("Connected!")

		# Various statuses
		dg535.write("CL")
		print(f"Error Status: {dg535.query('ES')}")
		print(f"Instrument Status: {dg535.query('IS')}")

		# cmd_lst = [
		# 	#"DT 2,1,1e-6", 	# Set the delay of channel A = T0 + 1us
		# 	"DT 3,2,1e-6", 	# Set the delay of channel B = A + 1us
		# 	"TZ 4,0",		# Set channel A termination impedance to 50ohm load
		# 	# "TZ 3,0",		# Set channel B termination impedance to 50ohm load
		# 	"OM 4,3",		# Channel A output VARiable
		# 	# "OM 3,3",		# Channel B output VARiable
		# 	f"OA 4,1",		# Channel A amplitude +1V
		# 	# "OA 3,0.5",		# Channel B amplitude +0.5V
		# 	f"OO 4,0.0",		# Channel A offset +0.5V
		# 	# "OO 3,0.25",	# Channel B offset +0.25V
		# 	"TM 0",			# Internal rate generator is trigger source
		# 	"TR 0,10000",	# Internal trigger mode 10kHz
		# ]
		print("Sending commands...")
		cmd_lst = [
			# "CL",
			#"DT 2,1,0.5E-7;",			# Set channel A delay = T0 + 50  ns
			"TM 2;DT 3,1,2.0E-7; OM 4,3; OO 4,1.2; OA 4,1.1; TZ 4,1; SS;"
			#"SS;",
			#"DT 3,1,2.0E-7;",			# Set channel B delay = T0 + 200 ns
			# "OM 4,3",					# AB output VARiable
			# f"OA 4,1.1",				# AB channel amplitude
			# "OO 4,0",					# AB channel offset
			# "TZ 4,1",					# Set AB output impedance to High-Z
			#"TM 0;",						# Set the triiger to be in Single shot mode
			#"SS",						# Send pulse
			#"SS",						# Send pulse
			#"DS Hi Roberto2!;"
		] 

		for cmd in cmd_lst:
			dg535.write(cmd)
			print("Sent: ", cmd)

		print("Closing connection...")
		dg535.close()

		print("Connection closed!")

if __name__ == "__main__":
	try:
		run_main()
	except:
		extype, value, tb = sys.exc_info()
		traceback.print_exc()
		pdb.post_mortem(tb)