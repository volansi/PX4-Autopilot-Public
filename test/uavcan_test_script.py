import os
import serial
import time
import itertools


def write_to_serial(ser,msg):
	print(msg)
	if ser:
		ser.write(bytes(msg + "\n",'utf-8'))


class CAN_Test_Single():

	def fill_single(self, name, lst, index):
		self.test_name = name + "_" + str(index+1)
		self.pub_param = lst[0]
		self.pub_id = lst[1]
		self.freq_param = lst[2]
		self.freq = lst[3][index]
		self.message_size = lst[4]

	def __str__(self):
		s = self.test_name + ','
		s += self.pub_param + ','
		s += str(self.pub_id) + ','
		s += self.freq_param + ','
		s += str(self.freq) + ','
		s += str(self.message_size) + '\n'
		return s

class CAN_Test_Composite():

	def fill_composite(self, name, lst, freq, index):
		total_tests = len(freq)
		template_test = []


		self.test_name = name  + "_" + str(index)

		for l in lst:
			t = CAN_Test_Single()
			t.fill_single(name,l,0)
			t.freq = -111
			template_test.append(t)
		self.test = template_test

		for i in range(len(template_test)):
			self.test[i].test_name = name  + "_" + str(index)
			self.test[i].freq = freq[index][i]

	def __str__(self):
		s = "\n"
		for t in self.test:
			s += str(t)
		return s


class CAN_Test:
	def __init__(self, name, settings, ser) -> None:
		self.name = name
		self.settings = settings
		self.ser = ser
		self.test_index = 0
		self.__expand_tests()

	def test_quantity(self):
		total_tests = 0
		for i,s in enumerate(self.settings):
			if i == 0:
				total_tests += len(s[3])
			else:
				total_tests *= len(s[3])
		return total_tests

	def __expand_tests(self):

		self.tt = []
		if len(self.settings) == 1:
			for f in range(len(self.settings[0][3])):
				t = CAN_Test_Single()
				t.fill_single(self.name,self.settings[0],f)
				self.tt.append(t)
		else:
			freqs = []
			for i,s in enumerate(self.settings):
				freqs.append(s[3])
			res = []
			for r in itertools.product(freqs[0],freqs[1],freqs[2]): res.append([r[0] , r[1], r[2]])

			for f in range(len(res)):
				tc = CAN_Test_Composite()
				tc.fill_composite(self.name,self.settings,res,f)
				self.tt.append(tc)

	def clean_slate(self, wait=0):
		write_to_serial(self.ser,'reboot')
		time.sleep(wait)

	def apply_test_settings(self, setting):
		write_to_serial(self.ser ,'uavcan_v1 start')
		time.sleep(3)
		write_to_serial(self.ser ,'param set ' + setting.pub_param + " " + str(setting.pub_id))
		write_to_serial(self.ser ,'param set ' + setting.freq_param + " " + str(setting.freq))

	def run_all_tests(self):
		for t in self.tt:
			print("Running " + t.test_name)
			self.clean_slate(wait=10)
			self.apply_test_settings(t)
			time.sleep(60)


	def __str__(self):
		s = self.name + ':'
		s += " Next Test " + str(self.test_index+1) + " out of " + str(self.test_quantity()) + " tests\n"
		for t in self.tt:
			s += str(t)
		s += '\n'
		return s



# Each entry follow this format: [Test_Name,[[TEST_1]..[TEST_N]]]
test_grid = \
	[
		['TEST_1',[['UCAN1_D_PUB_SM',22,'UCAN1_DPUB_SM_HZ',[1,100,200,400], 12]]],
		# ['TEST_2',[['UCAN1_D_PUB_MD',23,'UCAN1_DPUB_MD_HZ',[1,100,200,400], 24]]],
		# ['TEST_3',[['UCAN1_D_PUB_LG',24,'UCAN1_DPUB_LG_HZ',[1,100,200,400], 58]]],
		# ['COMPOSITE_TEST_1',[
		# 	['UCAN1_D_PUB_SM',22,'UCAN1_DPUB_SM_HZ',[400], 12],
		# 	['UCAN1_D_PUB_MD',23,'UCAN1_DPUB_MD_HZ',[200], 24],
		# 	['UCAN1_D_PUB_LG',24,'UCAN1_DPUB_LG_HZ',[0.1,1], 58]
		# 	]
		# ]
	]



def test_procedure(ser,setting):
	write_to_serial(ser,'reboot')
	time.sleep(5)
	write_to_serial(ser,'uavcan_v1 start')
	write_to_serial(ser,'param set ' + setting[0] + " " + str(setting[1]))
	write_to_serial(ser,'param set ' + setting[2] + " " + str(setting[3]))



SERIALPORT = "/dev/ttyACM3"
BAUDRATE = 57600

ser = serial.Serial(SERIALPORT, BAUDRATE)  # open serial port
ser.close()
ser.open()
print(ser.name)         # check which port was really used

for test in test_grid:

	name = test[0]
	tests = test[1]
	tt = CAN_Test(name,tests,ser)
	tt.run_all_tests()

ser.close()

exit()



for k,v in test_grid.items():
	print(k)
	print(v)
	test_procedure(ser,v)
	break
ser.close()             # close port


# cmd = 'yakut monitor > test.log &'
# os.system(cmd)
