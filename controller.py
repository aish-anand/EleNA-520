import sys
import osmnx as ox
import networkx as nx
import numpy as np
from heapq import *
from itertools import count
import time
import pickle as pkl

class Controller(object):

	def _init_(self):
		self.model = None
		self.view = None

	def set_model(self, model):
		self.model = model

	def set_view(self, view):
		self.view = view

	# gets user input from view
	def get_input(self):
		#origin_lat, origin_long, dest_lat, dest_long, overhead, mode = self.view.get_data()
		# testing without input
		origin_lat, origin_long, dest_lat, dest_long, overhead, mode = 42.3524, -71.06643, 42.358226, -71.061260, 10, "minimize"
		#getting graph + graph with projections
		graph_projection, graph_orig = self.get_map()
		origin = ox.get_nearest_node(graph_orig,(float(origin_lat), float(origin_long)))  # (37.77, -122.426))
		destination = ox.get_nearest_node(graph_orig,(float(dest_lat), float(dest_long)))  # (37.773, -122.441))
		bbox = ox.bbox_from_point((float(origin_lat), float(origin_long)), distance=1500, project_utm=True)
		#setting model's attributes
		self.model.set_origin(origin)
		self.model.set_dest(destination)
		self.model.set_overhead(overhead)
		self.model.set_mode(mode)
		self.model.set_bbox(bbox)
		self.model.set_graph_projection(graph_projection)

	# gets graphs and adds elevation
	def get_map(self, place='Boston', new_place=False):
		# if new_place == False:
		# 	return pkl.load(open("graph_projected.pkl","rb")), pkl.load(open("graph.pkl","rb"))

		#downloading local map
		place = 'Boston'
		place_query = {'city':'Boston', 'state':'Massachusetts', 'country':'USA'}
		graph_orig = ox.graph_from_place(place_query, network_type='drive')

		#adding elevation data from GoogleMaps
		#Enter the API key here
		key = self.model.get_key()
		graph_orig = ox.add_node_elevations(graph_orig, api_key=key)
		graph_orig = ox.add_edge_grades(graph_orig)
		pkl.dump(graph_orig, open("graph.pkl","wb"))

		#projecting map on to 2D space
		graph_projection = ox.project_graph(graph_orig)
		pkl.dump(graph_projection, open("graph_projected.pkl","wb"))
		return graph_projection, graph_orig

	def get_path(self, came_from, origin, destination):
		route_by_length_minele = []
		p = destination
		route_by_length_minele.append(p)
		while p != origin:
		   p = came_from[p]
		   route_by_length_minele.append(p)
		route_by_length_minele = route_by_length_minele[::-1]
		return route_by_length_minele

	def get_cost(self, graph_projection, a, b):
		return graph_projection.edges[a, b, 0]['length']

	def get_elevation_cost(self, graph_projection, a, b):
		return (graph_projection.nodes[a]['elevation'] - graph_projection.nodes[b]['elevation'])

	def get_total_elevation(self, graph_projection, route):
		if not route:
			return 0
		elevation_cost = 0
		for i in range(len(route)-1):
			 elevation_data = self.get_elevation_cost(graph_projection, route[i], route[i+1])
			 if elevation_data > 0:
				 elevation_cost += elevation_data
		return elevation_cost

	def get_total_length(self, graph_projection, route):
		if not route:
			return 0
		cost = 0
		for i in range(len(route)-1):
			 cost += self.get_cost(graph_projection, route[i], route[i+1])
		return cost

	# define some edge impedence function here
	def impedence(self, length, grade):
		penalty = grade ** 2
		return length * penalty

	# add impedence and elevation rise values to each edge in the projected graph
	# use absolute value of grade in impedence function if you want to avoid uphill and downhill
	def add_impedence(self, graph_projection):
		for u, v, k, data in graph_projection.edges(keys=True, data=True):
			data['impedence'] = self.impedence(data['length'], data['grade_abs'])
			data['rise'] = data['length'] * data['grade']

	def get_shortest_path(self, graph, source=0, target=0, weight='length'):
		frontier = []
		heappush(frontier, (0, source))
		came_from = {}
		cost_so_far = {}
		came_from[source] = None
		cost_so_far[source] = 0
		
		while len(frontier) != 0:
			(val, current) = heappop(frontier)
			if current == target:
				break;
			for u, next, data in graph.edges(current, data=True):
				new_cost = cost_so_far[current]
				if weight == 'length':
					inc_cost = self.get_cost(graph, u, next)
				elif weight == 'elevation':
					inc_cost = self.get_elevation_cost(graph, u, next)
				if inc_cost > 0:
					new_cost += inc_cost
				if next not in cost_so_far or new_cost < cost_so_far[next]:
					cost_so_far[next] = new_cost
					priority = new_cost
					heappush(frontier, (priority, next))
					came_from[next] = current
		return self.get_path(came_from, source, target)

	def dijkstra_search(self, graph, start, goal, viable_cost, mode='minimize'):
		frontier = []
		heappush(frontier, (0, start))
		came_from = {}
		cost_so_far = {}
		cost_so_far_ele = {}
		came_from[start] = None
		cost_so_far[start] = 0
		cost_so_far_ele[start] = 0
		while len(frontier) != 0:
			(val, current) = heappop(frontier)
			if current == goal:
				if cost_so_far[current] <= viable_cost:
					break
			for u, next, data in graph.edges(current, data=True):
				new_cost = cost_so_far[current] + self.get_cost(graph, current, next)
				new_cost_ele = cost_so_far_ele[current]
				elevation_cost = self.get_elevation_cost(graph, current, next)
				if elevation_cost > 0:
					new_cost_ele = new_cost_ele + elevation_cost 
				if next not in cost_so_far or new_cost < cost_so_far[next]:
					cost_so_far_ele[next] = new_cost_ele
					cost_so_far[next] = new_cost
					priority = new_cost_ele
					if mode =='maximize':
						priority = priority
					heappush(frontier, (priority, next))
					came_from[next] = current
		return self.get_path(came_from, start, goal)

	# exhaustive
	def dfs_get_all_paths(self, graph, start, goal, max_length):
		paths = []
		def dfs(current, le, current_path, visited):
			if current == goal:
				if le > max_length:
					return
				else:
					current_path.append(current)
					paths.append(current_path)
					#print ("This path length:",length)
					#print ("path found")
					return
			if le > max_length:
				return
			for u, next_node, data in graph.edges(current, data=True):
				if next_node in visited:
					continue
				dfs(next_node, le + abs(self.get_cost(graph, current, next_node)), current_path + [current], visited + [next_node])
			return
		dfs(start, 0, [], [])
		print ("Total paths:", len(paths))
		
		min_val = sys.maxsize
		max_val = -1*sys.maxsize
		min_path, max_path = [], []
		for path in paths:
			elevation_data = self.get_total_elevation(graph, path)
			if min_val != min(elevation_data, min_val):
				min_val = elevation_data
				min_path = path
			if max_val != max(elevation_data, max_val):
				max_val = elevation_data
				max_path = path
		return min_path, max_path

	def dfs_get_all_paths_hybrid(self, graph_projection, origin, destination, can_travel, cutoff):
		print("inside dfs get all paths")
		paths = list(nx.all_simple_paths(graph_projection, source=origin, target=destination, cutoff=cutoff))
		min_val = sys.maxsize
		max_val = -1*sys.maxsize
		min_path, max_path = [], []
		for path in paths:
			print("Path is", path)
			elevation_data = self.get_total_elevation(graph_projection, path)
			length_data = self.get_total_length(graph_projection, path)
			if min_val != min(elevation_data, min_val) and length_data < can_travel:
				min_val = elevation_data
				min_path = path
			if max_val != max(elevation_data, max_val) and length_data < can_travel:
				max_val = elevation_data
				max_path = path
		return min_path, max_path

	def get_route(self):
		graph_projection = self.model.get_graph_projection()
		origin = self.model.get_origin()
		destination = self.model.get_dest()
		overhead = self.model.get_overhead()
		mode = self.model.get_mode()
		algo = self.model.get_algo()
		bbox = self.model.get_bbox()
		shortest_path = self.get_shortest_path(graph_projection, source=origin, target=destination, weight='length')
		print("Printing Statistics of Shortest path route")
		print(shortest_path)
		self.view.show_stats(graph_projection, shortest_path)

		shortest_elevation_path = self.get_shortest_path(graph_projection, source=origin, target=destination, weight='elevation')
		print("Printing Statistics of Shortest Elevation path route")
		self.view.show_stats(graph_projection, shortest_elevation_path)

		shortest_path_length = self.get_total_length(graph_projection, shortest_path)
		can_travel = ((100.0 + overhead)*shortest_path_length)/100.0
		# print("Distance you are willing to travel : ", can_travel )

		if algo == 1:
			#algo1
			t = time.time()
			route_minimize_elevation1 = self.dijkstra_search(graph_projection, origin, destination, can_travel, mode='minimize')
			print ("Algorithm 1 took :", time.time() - t, " seconds")
			print("Printing Statistics of our algorithm's minimum Elevation route")
			self.view.show_stats(graph_projection, route_minimize_elevation1)
			self.view.show_route(graph_projection, route_minimize_elevation1, bbox)

		if algo == 2:
			#algo2
			t = time.time()
			route_minimize_elevation2, route_maximize_elevation2 = self.dfs_get_all_paths(graph_projection, origin, destination, can_travel)
			print ("Algorithm 2 time:", time.time() - t)
			print("Printing Statistics of Minimum Elevation route")
			self.view.show_stats(graph_projection, route_minimize_elevation2)
			if mode == 'minimize':
				self.view.show_stats(graph_projection, route_minimize_elevation2)
				self.view.show_route(graph_projection, route_minimize_elevation2, bbox)
			elif mode == 'maximize':
				self.view.show_stats(graph_projection, route_maximize_elevation2)
				self.view.show_route(graph_projection, route_maximize_elevation2, bbox)

		if algo == 3:
			#algo3
			t = time.time()
			route_minimize_elevation3, route_maximize_elevation3 = self.dfs_get_all_paths_hybrid(graph_projection, origin, destination, can_travel, len(shortest_path)+4)
			print ("Algorithm 3 took :", time.time() - t, " seconds")
			print("Printing Statistics of Minimum Elevation route")
			if mode == 'minimize':
				self.view.show_stats(graph_projection, route_minimize_elevation3)
				self.view.show_route(graph_projection, route_minimize_elevation3, bbox)
			elif mode == 'maximize':
				self.view.show_stats(graph_projection, route_maximize_elevation3)
				self.view.show_route(graph_projection, route_maximize_elevation3, bbox)