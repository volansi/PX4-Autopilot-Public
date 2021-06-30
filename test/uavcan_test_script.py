import os
import serial
import time
import itertools
import subprocess


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
	def __init__(self, name, settings, ser, sub) -> None:
		self.name = name
		self.settings = settings
		self.ser = ser
		self.test_index = 0
		self.sub = sub
		self.__expand_tests()
		self.init = True

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
		param_name = "uav_test_params"
		if self.init:
			write_to_serial(self.ser ,'uavcan_v1 start')
			time.sleep(3)
			write_to_serial(self.ser,'param set UCAN1_ESC_PUB 65535')
			write_to_serial(self.ser,'param set UCAN1_GPS_PUB 65535')
			write_to_serial(self.ser,'param set UCAN1_D_PUB_SM 65535')
			write_to_serial(self.ser,'param set UCAN1_D_PUB_MD 65535')
			write_to_serial(self.ser,'param set UCAN1_D_PUB_LG 65535')
			write_to_serial(self.ser,'param set UCAN1_DPUB_SM_HZ -1')
			write_to_serial(self.ser,'param set UCAN1_DPUB_MD_HZ -1')
			write_to_serial(self.ser,'param set UCAN1_DPUB_LG_HZ -1')
			write_to_serial(self.ser ,'uavcan_v1 stop')
			time.sleep(1)
			write_to_serial(self.ser,'param save ')
			time.sleep(1)
			# self.init = True

		write_to_serial(self.ser,'reboot')
		time.sleep(wait)

	def apply_test_settings(self, setting):
		write_to_serial(self.ser ,'uavcan_v1 start')
		time.sleep(3)
		write_to_serial(self.ser ,'param set ' + setting.pub_param + " " + str(setting.pub_id))
		write_to_serial(self.ser ,'param set ' + setting.freq_param + " " + str(setting.freq))

	def monitor_px4(self,folder_name, wait_for, ignore=False):
		total_msg = ""

		target = time.time() + wait_for
		while time.time() < target:
			b = self.ser.read(1)
			if len(b) < 1:
				continue
			try:
				# print(str(b), b.decode("ASCII"))
				total_msg += b.decode("ASCII")
			except:
				pass

		if not ignore:
			f = os.path.join(folder_name,'px4_console.log')
			with open(f,'a') as ff:
				ff.write(total_msg)


	def run_all_tests(self):
		for t in self.tt:
			print("Running " + t.test_name)
			self.clean_slate(wait=10)
			folder_name = '/home/shaun/.cantst/'+t.test_name + '_' + str(t.pub_id) + '_' + str(t.freq)

			os.system('rm -rf ' + folder_name)

			os.system('mkdir -p ' + folder_name)

			# Read any serial port data from the clean slate reboot (to clear it)
			self.monitor_px4(folder_name,10,ignore=True)

			sub_ = str(t.pub_id) + ":" + self.sub
			cmd = 'stdbuf -oL yakut sub ' + sub_ + ' > ' + folder_name + '/yakut.log &'
			print(cmd)
			os.system(cmd)

			# kick off the test
			self.apply_test_settings(t)
			self.monitor_px4(folder_name,40)

			# stop transmission and capture any final output
			write_to_serial(self.ser ,'uavcan_v1 stop')
			self.monitor_px4(folder_name,5)

			# time.sleep(40)
			cmd = 'pkill yakut'
			os.system(cmd)



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
		(['TEST_1',[['UCAN1_D_PUB_SM',22,'UCAN1_DPUB_SM_HZ',[10,50,100,200,400,600], 1]]],"dummy_data_types.reg.small.1.0"),
		(['TEST_2',[['UCAN1_D_PUB_MD',23,'UCAN1_DPUB_MD_HZ',[1,50,100,200,400,600], 8]]],"dummy_data_types.reg.medium.1.0"),
		(['TEST_3',[['UCAN1_D_PUB_LG',24,'UCAN1_DPUB_LG_HZ',[0.1,1,2,3,4,5,6,7,8,9,10,11,12], 512]]],"dummy_data_types.reg.vvlarge.1.0"),
		# ['COMPOSITE_TEST_1',[
		# 	['UCAN1_D_PUB_SM',22,'UCAN1_DPUB_SM_HZ',[400], 12],
		# 	['UCAN1_D_PUB_MD',23,'UCAN1_DPUB_MD_HZ',[200], 24],
		# 	['UCAN1_D_PUB_LG',24,'UCAN1_DPUB_LG_HZ',[0.1,1], 58]
		# 	]
		# ]
	]



SERIALPORT = "/dev/ttyACM5"
BAUDRATE = 57600

ser = serial.Serial(SERIALPORT, BAUDRATE, timeout=1)  # open serial port
ser.close()
ser.open()
print(ser.name)         # check which port was really used

for test_entry in test_grid:
	test,sub = test_entry
	name = test[0]
	tests = test[1]
	tt = CAN_Test(name,tests,ser,sub)
	tt.run_all_tests()
	break


ser.close()



