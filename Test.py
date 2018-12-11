import osmnx as ox
import networkx as nx
import numpy as np
from heapq import *
from itertools import count
import time
import _pickle as pkl
import time
from controller import *
ox.config(log_console = True, use_cache = True)

def get_map(place='Boston', key, new_place=False):
	if new_place == False:
		return pkl.load(open("graph_projected.pkl", "rb")), pkl.load(open("graph.pkl", "rb"))

	place_query = {'city': place, 'state': 'Massachusetts', 'country': 'USA'}
	G = ox.graph_from_place(place_query, network_type='drive')

	G = ox.add_node_elevations(G, api_key=key)
	G = ox.add_edge_grades(G)
	pkl.dump(G, open("graph.pkl", "wb"))
	G_proj = ox.project_graph(G)
	pkl.dump(G_proj, open("graph_projected.pkl", "wb"))
	return G_proj, G


def random_points_in_bbox(latitude, longitude, distance, num_points):
	bbox = ox.bbox_from_point((latitude, longitude), distance=distance)
	min_latitude = min(bbox[0], bbox[1])
	diff_latitude = abs(bbox[0] - bbox[1])
	min_longitude = min(bbox[2], bbox[3])
	diff_longitude = abs(bbox[2] - bbox[3])
	lat_long_pair = []
	for i in range(num_points):
		u_lat = np.random.uniform(0.0001, diff_latitude)
		lat = min_latitude + u_lat
		u_long = np.random.uniform(0.0001, diff_longitude)
		lgtd = min_longitude + u_long
		lat_long_pair.append([lat, lgtd])

	return lat_long_pair


def plot_results(s_dist, se_dist, algo_dist):
	x_axis = np.arange(len(s_dist))
	import matplotlib.pyplot as plt
	plt.plot(x_axis, s_dist, label="shortest_distance")
	plt.plot(x_axis, se_dist, label="shortest_elevation")
	plt.plot(x_axis, algo_dist, label="our_algo")
	plt.xticks(x_axis)
	plt.legend()
	plt.show()


def plot_results_from_dict(dic, plot_one=False, key_one=None, savename="comparison_chart", elev=False):
	import matplotlib.pyplot as plt
	x_axis = None
	max_len = dic["MAX"]
	y_ticks = np.arange(0, int(max_len + 200))
	len_1 = 0
	for key in dic.keys():
		if key != "MAX":
			len_1 = len(dic[key])
			x_axis = np.arange(len(dic[key]))
			break
	fig, ax = plt.subplots()
	print(dic)
	for key in dic.keys():
		if key != "MAX":
			assert len_1 == len(dic[key])
			ax.plot(x_axis, dic[key], label=key)
	ax.set_ylim((0, int(max_len + 200)))
	plt.xticks(x_axis)
	plt.xlabel("Destination")
	if not elev:
		plt.ylabel("Distance in m")
	else:
		plt.ylabel("elevationATION")
	plt.ylim((0, int(max_len + 1)))
	plt.legend()
	if not plot_one:
		plt.suptitle("Path comparison")
		plt.savefig(savename)
	else:
		plt.suptitle("Distance Graph")
		plt.savefig(key_one)


def set_max(max_curr, new_val):
	if new_val > max_curr:
		return new_val
	else:
		return max_curr


def compare_algorithms(G, G_proj, origin_lat_long, bbox_lat_long, bbox_dist, num_dest, extrapercent_travel, plot=False,
					   dump=True, dump_file="dist_dump.pkl", plot_individuals=True):
	c = Controller()
	origin = ox.get_nearest_node(G, (origin_lat_long[0], origin_lat_long[1]))
	print("Origin = ", origin)
	dest_list = random_points_in_bbox(bbox_lat_long[0], bbox_lat_long[1], bbox_dist, num_dest)
	max_distance_val = 0.0
	max_elevation_val = 0.0
	cost_data = []
	shortest = []
	shortest_elev = []
	algo_dist = []
	dic = { "hybrid_min_elevation": [], "hybrid_max_elevation": []}
	dic_e = {"hybrid_min_elevation": [], "hybrid_max_elevation": []}
	
	for each in dest_list:
		destination = ox.get_nearest_node(G, (each[0], each[1]))
		print("Destination = ", destination)
		route_actual = nx.shortest_path(G_proj, source=origin, target=destination, weight='length')
		total_actual = c.get_total_length(G_proj, route_actual)
		
		dij_elevation = c.get_total_elevation(G_proj, route_actual)
		
		max_distance_val = set_max(max_distance_val, total_actual)
		max_elevation_val = set_max(max_elevation_val, dij_elevation)
		dic["Shortest_Path"].append(total_actual)
		dic_e["Shortest_Path"].append(dij_elevation)

		route_actual_elev = nx.shortest_path(G_proj, source=origin, target=destination, weight='grade_abs')
		total_actual_elev = c.get_total_length(G_proj, route_actual_elev)
		
		shor_elevation = c.get_total_elevation(G_proj, route_actual_elev)
		
		max_distance_val = set_max(max_distance_val, total_actual_elev)
		max_elevation_val = set_max(max_elevation_val, shor_elevation)
		dic["Least_Elevation"].append(total_actual_elev)
		dic_e["Least_Elevation"].append(shor_elevation)

		extratravelpercent = extrapercent_travel
		can_travel = ((100 + extratravelpercent) * total_actual) / 100

		route_by_length1 = c.dijkstra_search(G_proj, origin, destination, can_travel)
		total_cost_mod_dijkstra = c.get_total_length(G_proj, route_by_length1)
		
		mod_dij_elevation = c.get_total_elevation(G_proj, route_by_length1)
		
		max_distance_val = set_max(max_distance_val, total_cost_mod_dijkstra)
		max_elevation_val = set_max(max_elevation_val, mod_dij_elevation)
		dic["Modified_Dijkstra"].append(total_cost_mod_dijkstra)
		dic_e["Modified_Dijkstra"].append(mod_dij_elevation)

		route_minimize_elevation2, route_maximize_elevation2 = c.dfs_get_all_paths(G_proj, origin, destination,
																				 can_travel)
		dfs1_min_elevation = c.get_total_elevation(G_proj, route_minimize_elevation2)
		dfs1_max_elevation = c.get_total_elevation(G_proj, route_maximize_elevation2)
		if len(route_minimize_elevation2) == 0:
			total_cost_dfs_gap_rmin2 = 0.0
		else:
			total_cost_dfs_gap_rmin2 = c.get_total_length(G_proj, route_minimize_elevation2)
		max_distance_val = set_max(max_distance_val, total_cost_dfs_gap_rmin2)
		max_elevation_val = set_max(max_elevation_val, dfs1_min_elevation)
		dic["DFS_1_min_elevation"].append(total_cost_dfs_gap_rmin2)
		dic_e["DFS_1_min_elevation"].append(dfs1_min_elevation)

		if len(route_maximize_elevation2) == 0:
			total_cost_dfs_gap_rmax2 = 0.0
		else:
			total_cost_dfs_gap_rmax2 = c.get_total_length(G_proj, route_maximize_elevation2)
		max_distance_val = set_max(max_distance_val, total_cost_dfs_gap_rmax2)
		max_elevation_val = set_max(max_elevation_val, dfs1_max_elevation)
		dic["DFS_1_max_elevation"].append(total_cost_dfs_gap_rmax2)
		dic_e["DFS_1_max_elevation"].append(dfs1_max_elevation)
		shortest_path = c.get_shortest_path(G_proj, source=origin, target=destination, weight='length')
		route_minimize_elevation3, route_maximize_elevation3 = c.dfs_get_all_paths_hybrid(G_proj, origin, destination,
																				   can_travel, len(shortest_path) + 4)
		dfs2_min_elevation = c.get_total_elevation(G_proj, route_minimize_elevation3)
		dfs2_max_elevation = c.get_total_elevation(G_proj, route_maximize_elevation3)

		if len(route_minimize_elevation3) == 0:
			total_cost_dfs_gap_rmin3 = 0.0
		else:
			total_cost_dfs_gap_rmin3 = c.get_total_length(G_proj, route_minimize_elevation3)
		max_distance_val = set_max(max_distance_val, total_cost_dfs_gap_rmin3)
		max_elevation_val = set_max(max_elevation_val, dfs2_min_elevation)
		dic["hybrid_min_elevation"].append(total_cost_dfs_gap_rmin3)
		dic_e["hybrid_min_elevation"].append(dfs2_min_elevation)
		if len(route_maximize_elevation3) == 0:
			total_cost_dfs_gap_rmax3 = 0.0
		else:
			total_cost_dfs_gap_rmax3 = c.get_total_length(G_proj, route_maximize_elevation3)
		max_distance_val = set_max(max_distance_val, total_cost_dfs_gap_rmax3)
		max_elevation_val = set_max(max_elevation_val, dfs2_max_elevation)
		dic["hybrid_max_elevation"].append(total_cost_dfs_gap_rmax3)
		dic_e["hybrid_max_elevation"].append(dfs2_max_elevation)
		len_t = len(dic["Modified_Dijkstra"])
		for each in dic.keys():
			if len_t != len(dic[each]):
				print(each)
				print(len(dic[each]))
				assert len_t == len(dic[each])

		shortest.append(total_actual)
		shortest_elev.append(total_actual_elev)
		algo_dist.append(total_cost_mod_dijkstra)
	dic["MAX"] = max_distance_val
	dic_e["MAX"] = max_elevation_val
	d2 = {"MAX": dic["MAX"], "Shortest_Path": dic["Shortest_Path"], "Least_Elevation": dic["Least_Elevation"],
		  "hybrid_max_elevation": dic["hybrid_max_elevation"]}

	d4e = {"MAX": dic_e["MAX"], "Shortest_Path": dic_e["Shortest_Path"], "Least_Elevation": dic_e["Least_Elevation"],
		  "hybrid_min_elevation": dic_e["hybrid_min_elevation"]}
	d5e = {"MAX": dic_e["MAX"], "Shortest_Path": dic_e["Shortest_Path"], "Least_Elevation": dic_e["Least_Elevation"],
		  "hybrid_max_elevation": dic_e["hybrid_max_elevation"]}

	if dump:
		pkl.dump(dic, open(dump_file, "wb"))

	if plot:
		plot_results_from_dict(dic)
		plot_results_from_dict(d4e, savename="Elev_hybrid_min_elevation_SP_LE",elev=True)
		plot_results_from_dict(d5e, savename="Elev_hybrid_max_elevation_SP_LE",elev=True)
	if plot_individuals:
		for each in dic:
			if each != "MAX":
				temp_dict = {}
				temp_dict["MAX"] = dic["MAX"]
				temp_dict[each] = dic[each]
				plot_results_from_dict(temp_dict, True, each)



if __name__ == '__main__':
	parser = argparse.ArgumentParser(description='Runs tests for all algorithms, expects api key to be passed when called')
	parser.add_argument('--key', help='an API key for the elevation API')
	args = parser.parse_args()
	key = args.key
	G_proj, G = get_map(place = "Boston", key= key, new_place=True)
	extratravel = int(input("Please input the extra percent travel: Make sure its an integer value\n"))
	num_dest = int(input("Please input the number of destinations you want to check: Make sure this number is an integer value\n"))
	origin_lat = 37.772
	origin_long = -122.434
	compare_algorithms(G, G_proj, origin_lat_long=[origin_lat, origin_long], bbox_lat_long=(origin_lat, origin_long), bbox_dist=500, num_dest=num_dest, extrapercent_travel=extratravel, plot=True)
