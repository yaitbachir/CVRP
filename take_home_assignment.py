from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
from math import sqrt
import csv
import argparse


def load_input_file(filename):
	global N,V,c,demands,capacities,nodes
	with open(filename) as fd:
		rd = csv.reader(fd, delimiter="\t", quotechar='"')
		header = next(rd)
		(N,V,c) = [int(i) for i in header]
		capacities = [c] * V
		for row in rd:
			demands.append(int(row[0]))
			nodes.append((int(row[1]),int(row[2])))


def euclidean_distance(position_1, position_2):
	"""Computes the Euclidean distance between two points"""
	return sqrt((position_1[0] - position_2[0])**2 + (position_1[1] - position_2[1])**2)

def create_distance_callback():
	"""Creates callback to return distance between points."""
	_distances_matrix = {}

	for from_node in range(N+1):
		_distances_matrix[from_node] = {}
		for to_node in range(N+1):
				_distances_matrix[from_node][to_node] = (euclidean_distance(nodes[from_node],nodes[to_node]))

	def distance_callback(from_node, to_node):
		"""Returns the euclidean distance between the two nodes"""
		return _distances_matrix[from_node][to_node]

	return distance_callback

def create_demand_callback():
		"""Creates callback to get demands at each location."""
		def demand_callback(from_node, to_node):
				return demands[from_node]
		return demand_callback

def add_capacity_constraints(routing, demand_callback):
		"""Adds capacity constraint"""
		capacity = "Capacity"
		routing.AddDimensionWithVehicleCapacity(
				demand_callback,
				0, # null capacity slack
				capacities, # vehicle maximum capacities
				True, # start cumul to zero
				capacity)

def print_solution(routing, assignment):
		"""Print routes on console."""
		total_dist = 0
		for vehicle_id in range(V):
				index = routing.Start(vehicle_id)
				plan_output = 'Route for vehicle {0} - Capacity {1}:\n'.format(vehicle_id,capacities[vehicle_id])
				route_dist = 0
				route_load = 0
				while not routing.IsEnd(index):
						node_index = routing.IndexToNode(index)
						next_node_index = routing.IndexToNode(assignment.Value(routing.NextVar(index)))
						route_dist += euclidean_distance(
								nodes[node_index],
								nodes[next_node_index])
						route_load += demands[node_index]
						plan_output += ' {0} Load({1}) -> '.format(node_index, route_load)
						index = assignment.Value(routing.NextVar(index))

				node_index = routing.IndexToNode(index)
				total_dist += route_dist
				plan_output += ' {0} Load({1})\n'.format(node_index, route_load)
				plan_output += 'Distance of the route: {0}\n'.format(route_dist)
				plan_output += 'Load of the route: {0}\n'.format(route_load)
				print(plan_output)
		print('Total Distance of all routes: {0}'.format(total_dist))

def write_output_file(filename, routing, assignment):
		"""Write Output File"""
		total_dist = 0
		output = []
		for vehicle_id in range(V):
			vec_output = []
			index = routing.Start(vehicle_id)
			vec_output.append(vehicle_id)
			route = ""
			while not routing.IsEnd(index):
					node_index = routing.IndexToNode(index)
					next_node_index = routing.IndexToNode(assignment.Value(routing.NextVar(index)))
					total_dist += euclidean_distance(nodes[node_index],nodes[next_node_index])
					route += '{0} '.format(node_index)
					index = assignment.Value(routing.NextVar(index))
			route += "0"
			vec_output.append(route)
			output.append(vec_output)
		optimal = 1 if (routing.status() == 1) else 0
		optimal = routing.status()
		output = [[total_dist, optimal]] + output
		with open(filename,'w') as outf:
			writer = csv.writer(outf, delimiter="\t")
			writer.writerows(output)

def display_chart(routing, assignment):
		"""Write Output File"""
		import matplotlib.pyplot as plt
		import matplotlib.cm as cm
		x_nodes = [node[0] for node in nodes]
		y_nodes = [node[1] for node in nodes]
		plt.figure(figsize=(20,10))
		plt.scatter(x_nodes,y_nodes,c='b',marker='o',zorder=5)
		for i,d in enumerate(demands):
			plt.text(x_nodes[i]-10,y_nodes[i]-40,"{0}\nd={1}".format(i,d),fontsize=8,color='b')
		plt.scatter(0,0,c='r',marker='s',zorder=10)
		plt.text(-10,-30,"depot",fontsize=10,color='r')
		colormap = cm.tab20
		for vehicle_id in range(V):
			index = routing.Start(vehicle_id)
			segment=0
			while not routing.IsEnd(index):
					node_index = routing.IndexToNode(index)
					next_node_index = routing.IndexToNode(assignment.Value(routing.NextVar(index)))
					plt.arrow(nodes[node_index][0],nodes[node_index][1],nodes[next_node_index][0]-nodes[node_index][0],nodes[next_node_index][1]-nodes[node_index][1],
										width=3, color=colormap(vehicle_id/V))
					if segment == 1:
						plt.text((nodes[next_node_index][0]+nodes[node_index][0])/2+20,(nodes[next_node_index][1]+nodes[node_index][1])/2+20, "V{0}".format(vehicle_id),fontsize=12, weight='bold', color=colormap(vehicle_id/V))
					index = assignment.Value(routing.NextVar(index))
					segment+=1
		plt.show()


########
# Main #
########
def main():
	# load input file
	load_input_file(args.input_file)

	# Constraint Validation
	# Are vehicle can load all the demand?
	if(sum(capacities) < sum(demands)) or N == 0:
		print("Problem impossible to solve!\nVerify the following:\n\
					- Format of input file\n\
					- Sum of vehicles capacities >= sum of demands of customers")
		return 0
	
	# Create Routing Model
	routing = pywrapcp.RoutingModel(N+1,V,0)
	# Define weight of each edge
	distance_callback = create_distance_callback()
	routing.SetArcCostEvaluatorOfAllVehicles(distance_callback)
	# Add Capacity constraint
	demand_callback = create_demand_callback()
	add_capacity_constraints(routing, demand_callback)

	# Setting first solution heuristic (cheapest addition).
	search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
	search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
	search_parameters.time_limit_ms = args.max_time * 1000 # in ms
	# Solve the problem.
	assignment = routing.SolveWithParameters(search_parameters)
	
	write_output_file(args.output_file,routing, assignment)
	if args.verbose:
		print_solution(routing, assignment)
	if args.chart:
		display_chart(routing, assignment)




if __name__ == '__main__':
	####################
	# Global variables #
	####################
	N = 0 # Number of locations
	V = 0 # Number of vehicules
	c = 0 # Capacity of all vehicule
	capacities = [] # Capacity for each vehicule (to augment problem with different capacity per vehioule)
	nodes = [(0,0)] # list of locations, with depot locations already served
	demands = [0] # depot demand, list of demands fopr each locations - initialise to 0 for depot solution


	#############
	# Arguments #
	#############
	parser = argparse.ArgumentParser()	

	parser.add_argument("input_file", help="Path to input file")
	parser.add_argument("output_file", help="Path to output file")
	parser.add_argument("-v", "--verbose", help="Write the solution details in the console",action="store_true")
	parser.add_argument("-c", "--chart", help="Display a Visualisation chart of the solution",action="store_true")
	parser.add_argument("-t", "--max_time", help="Max time in seconds for optimization [default 60s]",type=int,default=60)
	
	args = parser.parse_args()

	main()