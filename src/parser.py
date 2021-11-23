import struct
import sys

from os import listdir
from os.path import isfile, join

if __name__ == "__main__":

	# File list
	files = ['LOG.BIN']
	t = 0

	for file in files :
		print(file)
		f = open(file, 'rb')
		f2 = open(file.replace('.BIN','.csv'), 'w')
		f2.write("data,\n")

		state = 0

		while True :
			x = f.read(38)
			if len(x) != 38 :
				break

			if x[0] != 0xAB :
				print("Wrong header!")
				break;

			if x[1] != 0xCD :
				print("Wrong header!")
				break;

			for n in range(18) :
				d = struct.unpack('<H', bytearray([x[2+2*n], x[3+2*n]]))[0]
				f2.write(str(d) + ',')
			f2.write('\n')

		f.close()
		f2.close()
