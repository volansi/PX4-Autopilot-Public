import argparse
from numpy.core.numeric import full
import pandas as pd
import os
import numpy as np
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser(description='Process tests.')
parser.add_argument('folder', metavar='f', type=str, nargs='+',
                    help='base folder containing all test data')

args = parser.parse_args()


def process_counts_log(folder_loc):
	file = os.path.join(folder_loc,"counts.log")
	counts = None
	if os.path.lexists(file):
		try:
			df = pd.read_csv(file, sep=':',skiprows=10)
		except:
			return [-1]
		counts = df.iloc[:,2:3]
	if not counts.empty:

		c = counts.to_numpy()
		d = c[1:] - c[:-1]

		return [np.max(d),np.sum(d[np.where(d>1)])]
	return [-1]


def process_times_log(folder_loc):
	file = os.path.join(folder_loc,"times.log")
	times = None
	if os.path.lexists(file):
		try:
			df = pd.read_csv(file, sep=':|,',skiprows=10)
		except:
			return [-1]
		times = df.iloc[:,3:4]
	if not times.empty:

		t = times.to_numpy()
		d = t[1:] - t[:-1]
		return [np.max(d),np.mean(d),np.std(d),np.var(d),d]
	return [-1]

def process_px4_console_log(folder_loc):
	file = os.path.join(folder_loc,"px4_console.log")

	file = open(file,mode='r')
	console_log = file.read()
	file.close()

	counted_messages_sent = console_log.count('(')
	counted_messages_discarded = console_log.count('#')
	print(counted_messages_sent,counted_messages_discarded)

def get_all_tests(folder):
	return [name for name in os.listdir(folder) if os.path.isdir(os.path.join(folder, name))]


def main():
	full_results_set = {}
	print(args.folder)
	tests = get_all_tests(args.folder[0])
	tests.sort()
	for test in tests:
		res = []
		full_path = os.path.join(args.folder[0],test)
		print(full_path)

		res.extend(process_counts_log(full_path))
		if res[0] != -1:
			delta = res[0]
			dropped = res[1]

		res.extend(process_times_log(full_path))
		full_results_set[full_path] = res
		if res[0] != -1:
			plt.hist(res[-1], bins=10)
			plt.xlabel('time (s)')
			plt.ylabel('count')
			plt.title('Histogram of time delta between messages')
			plt.savefig(os.path.join(full_path,"time_hist.png"))

		process_px4_console_log(full_path)

		if res[0] != -1:
			if dropped == 0:
				continue
			max_time_between_messages = res[2]
			avg_time_between_messages = res[3]
			print(dropped, max_time_between_messages, avg_time_between_messages, 1/avg_time_between_messages)

	# print(full_results_set)


if __name__ == "__main__":
	main()
