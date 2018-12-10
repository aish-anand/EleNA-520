import numpy as np

class View(object):
	def _init_(self):
		self.controller = None

	def set_controller(self, controller):
		self.controller = controller

	def get_data(self):
		VALID_MODES = ['minimize', 'maximize']
		origin_lat = input("Please enter the Latitude of the Origin \n")
		origin_long = input("Please enter the Longitude of the Origin \n")
		print("Latitude of origin : ", float(origin_lat), " and Longitude of origin", float(origin_long))
		dest_lat = input("Please enter the Latitude of the Destination \n")
		dest_long = input("Please enter the Longitude of the Destination \n")
		print("Latitude of destination : ", float(dest_lat), " and Longitude of destination", float(dest_long))
		overhead = float(input("Please enter the percentage of shortest path between above points that you are willing to travel extra \n"))
		print("x% : ", overhead)
		mode = input("To minimize elevation, please type 'minimize' \nTo maximize elevation, please type 'maximize'\n")

		if mode not in VALID_MODES:
			raise ValueError("Mode invalid")
		return origin_lat, origin_long, dest_lat, dest_long, overhead, mode
