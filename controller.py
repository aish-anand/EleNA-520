import sys
import osmnx as ox
import networkx as nx
import numpy as np
from heapq import *
from itertools import count
import time
#from model import *
import pickle as pkl

class Controller(object):
	def _init_(self):
		self.model = None
		self.view = None

	def set_model(self, model):
		self.model = model

	def set_view(self, view):
		self.view = view

	def get_input(self):
		origin_lat, origin_long, dest_lat, dest_long, overhead, mode = self.view.get_data()
		#origin_lat, origin_long, dest_lat, dest_long, overhead, mode = 37.77, -122.469, 37.74, -122.441, 10, "minimize"
		graph_projection, graph_orig = self.get_map()
		origin = ox.get_nearest_node(graph_orig, (float(origin_lat), float(origin_long)))  # (37.77, -122.426))
		destination = ox.get_nearest_node(graph_orig, (float(dest_lat), float(dest_long)))  # (37.773, -122.441))
		bbox = ox.bbox_from_point((float(origin_lat), float(origin_long)), distance=1500, project_utm=True)
		self.model.set_origin(origin)
		self.model.set_dest(destination)
		self.model.set_overhead(overhead)
		self.model.set_mode(mode)
		self.model.set_bbox(bbox)
		self.model.set_graph_projection(graph_projection)

	def get_map(self, place='Boston', new_place=False):
		if new_place == False:
			return pkl.load(open("graph_projected.pkl","rb")), pkl.load(open("graph.pkl","rb"))
		#Downloading Local map
		place = 'Boston'
		place_query = {'city':'Boston', 'state':'Massachusetts', 'country':'USA'}
		graph_orig = ox.graph_from_place(place_query, network_type='drive')

		#Adding Elevation data from GoogleMaps
		graph_orig = ox.add_node_elevations(graph_orig, api_key='AIzaSyDU_zAP8D2D9c54N9G5nMPYF52H5VZ_T4o')
		graph_orig = ox.add_edge_grades(graph_orig)
		graph_file = open("graph.pkl", "wb")
		pkl.dump(graph_orig, graph_file)
		graph_file.close()

		#projecting map on to 2D space
		graph_projection = ox.project_graph(graph_orig)
		file_projection = open("graph_projected.pkl", "wb")
		pkl.dump(graph_projection, file_projection)
		file_projection.close()
		return graph_projection, graph_orig

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
		can_travel = ((100.0 + overhead) * shortest_path_length) / 100.0
		print("Distance you are willing to travel : ", can_travel)

		if algo == 1:
			#Strategy1
			t = time.time()
			route_minimize_elevation1 = self.dijkstra_search(graph_projection, origin, destination, can_travel, mode='minimize')
			print ("Algorithm 1 took :", time.time()-t, " seconds")

		if algo == 2:
			#algo2
			t = time.time()
			route_minimize_elevation2, route_maximize_elevation2 = self.dfs_get_all_paths(graph_projection, origin, destination, can_travel)
			print ("Algorithm 2 time:", time.time() - t)

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
	
	def get_total_length(self, graph_projection, route):
		if not route:
			return 0
		cost = 0
		for i in range(len(route)-1):
			cost += self.get_cost(graph_projection, route[i], route[i + 1])
		return cost

	def get_cost(self, graph_projection, a, b):
		return graph_projection.edges[a, b, 0]['length']

	def get_elevation_cost(self, graph_projection, a, b):
		return (graph_projection.nodes[a]['elevation'] - graph_projection.nodes[b]['elevation'])

	def get_path(self, came_from, origin, destination):
		route_by_length_minele = []
		p = destination
		route_by_length_minele.append(p)
		while p != origin:
			p = came_from[p]
			route_by_length_minele.append(p)
		route_by_length_minele = route_by_length_minele[::-1]
		return route_by_length_minele

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
					if mode=='maximize':
						priority=priority
					heappush(frontier, (priority, next))
					came_from[next] = current
		return self.get_path(came_from, start, goal)

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