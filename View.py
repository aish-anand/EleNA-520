import numpy as np

class View(object):
	def _init_(self):
		self.controller = None

	def set_controller(self, controller):
		self.controller = controller

	def show_route(self, graph_projection, route, bbox):
		print("Showing route")
		# lat_long_list = []
		# gmap = gmplot.GoogleMapPlotter(37.7749295, -122.4194155, 13)
		# #gmap = self.from_geocode("San Francisco", apikey = 'AIzaSyDU_zAP8D2D9c54N9G5nMPYF52H5VZ_T4o')
		# for i in range(0, len(route)):
		# 	sum += route[i]
		# 	lat_long_list.append(tuple((graph_projection.node[route[i]]['y'], graph_projection.node[route[i]]['x'])))
		# top_attraction_lats, top_attraction_lons = zip(*lat_long_list)
		# gmap.scatter(top_attraction_lats, top_attraction_lons, '#3B0B39', size = 40, marker = False)
		# # Marker
		# hidden_gem_lat, hidden_gem_lon = 37.77, -122.426
		# gmap.marker(hidden_gem_lat, hidden_gem_lon, 'cornflowerblue')
		# hidden_gem_lat_2, hidden_gem_lon_2 = 37.773, -122.441
		# gmap.marker(hidden_gem_lat_2, hidden_gem_lon_2, 'cornflowerblue')
		# # Draw
		# gmap.draw("my_map1.html")
		ox.plot_graph_route(graph_projection, route, bbox = bbox, node_size=0)

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

	def show_stats(self, graph_projection, route):
		route_lengths = ox.get_route_edge_attributes(graph_projection, route, 'length')
		print('Total trip distance: {:,.0f} meters'.format(self.controller.get_total_length(graph_projection, route)))
		print('Total elevation change: {:,.0f}'.format(self.controller.get_total_elevation(graph_projection, route)))
