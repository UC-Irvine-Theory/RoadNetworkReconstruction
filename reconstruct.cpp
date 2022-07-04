#include <set>
#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_set>
#include <tuple>
#include <string>
#include <utility>
#include <iterator>
#include <ctime>
#include <cmath>
#include <climits>
#include <algorithm>
#include <random>
#include <unordered_map>

std::ofstream outfileResults;
std::ofstream outfileCounts;

void SetToVec(std::unordered_set<int> &set, std::vector<int> &vec) {
	vec.reserve(set.size()); // See https://stackoverflow.com/questions/42519867/efficiently-moving-contents-of-stdunordered-set-to-stdvector
	for (auto it = set.begin(); it != set.end(); ) {
		vec.push_back(std::move(set.extract(it++).value()));
	}
} // Quickly writes the contents of a set into an empty vector of the same size.

class Graph
{
	std::vector<std::vector<int>> edges; // Each index in the outer vector is a node in the graph. Each vector at edges[node] is a vector of neighbors connected to 'node'.
	public:
	int step = 0; // Tracking how many partitions have occurred.
	size_t distanceQueryCount = 0;
	size_t kthHopQueryCount = 0;
	std::vector<int> centers; // A set of centers used for partitioning.
	std::vector<int> centersDistances; // For each node, the minimum distance from a center.
	std::vector<std::vector<int>> cells; // For each node, a vector of cells which it belongs to.
	std::vector<std::unordered_set<int>> cellsNodes; // For each cell, a set of nodes in that cell.
	std::set<std::pair<int, int>> queriedPairs; // Stores vertex pairs that have already been queried
	std::vector<int> cellSizes;
	std::vector<std::unordered_map<int, int>> cellNeighbors; // Stores a counter for each pair of clusters: a counter for a particular cluster pair corresponds to the number of vertices contributing to the neighborship of those two clusters. If the count is 0, the two clusters are not neighbors.
	std::unordered_map<int, int> nodeIds; // Stores mapping of node id's in input file to 0-indexed node id's which we use in our implementation.

	void SetNodes(int count) { // Initialize with a specified number of nodes.
		edges = std::vector<std::vector<int>>(count, std::vector<int>());
		centersDistances = std::vector<int>(count, INT_MAX);
		cells = std::vector<std::vector<int>>(count, std::vector<int>());
	}
	void AddEdge(int node1, int node2) { // Add an undirected/directed connection between 2 nodes.
		if (node1 == node2) {return;}
		if (std::find(edges[node1].begin(), edges[node1].end(), node2) == edges[node1].end()) {
			edges[node1].insert(lower_bound(edges[node1].begin(), edges[node1].end(), node2), node2);
			edges[node2].insert(lower_bound(edges[node2].begin(), edges[node2].end(), node1), node1);
		}
	}
	std::vector<int> ForceConnected() {
		int attempts = 0;
		int center;
		std::vector<int> subgraph;
		std::vector<int> connected;
		do {
			subgraph.clear();
			center = std::rand() % GetNodeCount();
			connected = GetDistances({center},0,{},true); // The reconstruction algorithms only work on connected graphs, and a very small fraction of the nodes in road network datasets are not connected to the whole.
			for (size_t node = 0; node < GetNodeCount(); node++) {
				if (connected[node] != INT_MAX) {subgraph.push_back(node);}
			}
			attempts++;
			if (attempts > 10) {break;}
		} while (subgraph.size() < GetNodeCount()/2);
		for (size_t node = 0; node < GetNodeCount(); node++) {
			if (connected[node] == INT_MAX) {edges[node] = {};}
		}
		return subgraph;
	}
	size_t GetNodeCount() {
		return edges.size();
	}

	std::vector<int> GetNodeNeighbors(int node) {
		return edges[node];
	}

	int GetMaximumDegree() {
		int maximum = 0;
		for (size_t node = 0; node < GetNodeCount(); node++) {
			int degree = GetNodeNeighbors(node).size();
			if (degree > maximum) {maximum=degree;}
		}
		return maximum;
	}

	void ClearCounters() {
		distanceQueryCount = 0;
		kthHopQueryCount = 0;
	}
	bool VerifyReconstruction(std::vector<std::vector<int>> reconstruction) {
		// Checks if the edges in the reconstruction match the edges in the input graph.
		for (size_t node = 0; node < GetNodeCount(); node++) {
			std::sort(reconstruction[node].begin(), reconstruction[node].end());
		}
		for (size_t i = 0; i<reconstruction.size(); i++) {
			for (size_t j = 0; j<reconstruction[i].size(); j++) {
				if (reconstruction[i][j] != edges[i][j]) {
					std::cout << reconstruction[i][j] << std::endl;
					std::cout << edges[i][j] << std::endl;
				}
			}
		}
		return (reconstruction == edges);
	}
	void GetDistances(std::vector<int> sources, std::vector<int> &distances, int limit = 0, std::unordered_set<int> subgraph = {}, bool free = false, int queryCount = 0, int hops = 0) {
		std::vector<int> previous;
		BreadthFirstSearch(sources, distances, previous, limit, subgraph, free, queryCount, hops);
	}
	std::vector<int> GetDistances(std::vector<int> sources, int limit = 0, std::unordered_set<int> subgraph = {}, bool free = false, int queryCount = 0, int hops = 0) {
		std::vector<int> distances(GetNodeCount(), INT_MAX); //By default, unreached nodes have a distance of INT_MAX.
		GetDistances(sources, distances, limit, subgraph, free, queryCount, hops);
		return distances;
	}
	void BreadthFirstSearch(std::vector<int> sources, std::vector<int> &distances, std::vector<int> &previous, int limit = 0, std::unordered_set<int> subgraph = {}, bool free = false, int queryCount = 0, int hops = 0) { //Breadth-first search flooding out from one or more source nodes. May optionally confine search to a subgraph. May optionally limit the depth of the search. Has both a version that assigns distances to an already allocated vector in-place, and a version that generates a new vector as a distance table.
		if (!queryCount) {queryCount = sources.size();}
		if (subgraph.size()) {
			for (size_t node = 0; node < GetNodeCount(); node++) {
				if (!subgraph.count(node)) {distances[node] = -1;} //If subgraph is specified, ignore nodes outside of that subgraph, assigning them a distance of -1.
			}
		}
		std::vector<int> frontier = sources;
		while (!frontier.empty()) {
			for (int node : frontier) { //Update distances table with currently visited nodes.
				distances[node] = hops;
				if(!free) {
					distanceQueryCount += queryCount; //One distance query from each of the source nodes.
					kthHopQueryCount += hops*queryCount; //Since it's a breadth-first search, once we find a shortest path from any one source to the destination, we can stop making kth-hop queries to that destination.
					if (hops == 0) {
						distanceQueryCount--; //It is unnecessary to make a query to establish that a node has a distance of 0 with itself.
						kthHopQueryCount--;
					}
				}
			}
			if (limit && hops == limit) {
				if (!free) {
					for (int distance : distances) { //Limiting the depth of the search is allowed for better running speed purposes, but we still have to count the queries that would have been necessary if we really only had a distance or kth_hop oracle.
						if (distance == INT_MAX) {
							distanceQueryCount += queryCount;
							kthHopQueryCount += hops*queryCount;
						}
					}
				}
				break;
			}
			std::unordered_set<int> newFrontier;  // Using an unordered set to prioritize fast insertion while filtering out duplicates.
			for (int node : frontier) {
				for (int neighbor : GetNodeNeighbors(node)) {
					if (distances[neighbor] == INT_MAX) {
						newFrontier.insert(neighbor);  // New frontier = neighbors of frontier nodes that haven't yet been visited.
						if (previous.size()) {previous[neighbor] = node;}
					}
				}
			}
			frontier.clear();
			SetToVec(newFrontier, frontier);  // Clear the frontier, then load the newFrontier into the frontier.
			hops++;
		}
	}

	void Partition(int newCenter) {
		// Adds a new center and returns the resulting partition of the graph. This function simulates the oracle by performing BFS to find distances.
		// We compute the number of queries necessary to find these distances during the search.
		step++;
		size_t previousQueryCount = distanceQueryCount;
		int hops = 0;
		centers.push_back(newCenter);
		cellSizes.push_back(0);
		cellNeighbors.push_back(std::unordered_map<int, int>());
		cellsNodes.push_back(std::unordered_set<int>());
		std::unordered_set<int> active_cells; // Only continue doing BFS on cells that contain vertices that are close to newCenter	
		for (int cell : cells[newCenter]) {
			active_cells.insert(cell);
		}
		std::unordered_set<int> visited;
		std::vector<int> frontier = {newCenter};
		std::unordered_set<int> ignore;
		while (!frontier.empty()) {
			for (int node : frontier) {  // Update distances table with currently visited nodes.
				visited.insert(node);
				if (!cells[node].empty()) {
					bool ignored = true;
					for (int cell : cells[node]) {
						if (active_cells.count(cell)) {ignored = false;}
					}
					if (ignored) {
						ignore.insert(node);
						continue;
					}
				}

				// Order query pairs by vertex ID
				std::pair<int, int> query;
				if (newCenter < node) {
					query = std::make_pair(newCenter, node);
				} else {
					query = std::make_pair(node, newCenter);
				}
				
				// Check to see if we already queried this pair. If not, update the query counts
				if (newCenter != node and !queriedPairs.count(query)) { 
					distanceQueryCount++;
					int nearest = std::ceil(log2(hops))+1;
					if (hops==1) kthHopQueryCount+=1; else kthHopQueryCount+= std::min<int>(2*(nearest-1), hops);
					queriedPairs.insert(query);
				}

				if (hops <= centersDistances[node]-1) {  // Inside the cell.
					for (int cell : cells[node]) {
						active_cells.insert(cell);
					}
					for (int cellId : cells[node]) { // Remove this node from all of its clusters.
						cellsNodes[cellId].erase(node);
						cellSizes[cellId]--;
					}
					// Decrease the degrees of these clusters if necessary.
					for (size_t i=0; i<cells[node].size(); i++){
						for (size_t j=i+1; j<cells[node].size(); j++){
							cellNeighbors[cells[node][i]][cells[node][j]]--;
							cellNeighbors[cells[node][j]][cells[node][i]]--;
							if (cellNeighbors[cells[node][i]][cells[node][j]] == 0 and cellNeighbors[cells[node][j]][cells[node][i]] == 0) {
								cellNeighbors[cells[node][i]].erase(cells[node][j]);
								cellNeighbors[cells[node][j]].erase(cells[node][i]);
							}
						}
					}
					// Add this node to newCenter's cluster.
					cells[node] = {(int)centers.size()-1};
					cellsNodes[(int)centers.size()-1].insert(node);
					centersDistances[node] = hops;
					cellSizes[(int)centers.size()-1]++;
				}
				else if (hops <= centersDistances[node]+1) {  // On the border of the cell.
					for (int cell : cells[node]) {
						active_cells.insert(cell);
						// Each cluster that this node belongs to is now a neighbor of newCenter's cluster
						cellNeighbors[(int)centers.size()-1][cell]++;
						cellNeighbors[cell][(int)centers.size()-1]++;
					}
					// Add this node to newCenter's cluster.
					cells[node].push_back((int)centers.size()-1);
					cellsNodes[(int)centers.size()-1].insert(node);
					centersDistances[node] = std::min(hops, centersDistances[node]);
					cellSizes[(int)centers.size()-1]++;
				}
			}
			std::unordered_set<int> newFrontier;  // Using an unordered set to prioritize fast insertion while filtering out duplicates.
			for (int node : frontier) {
				if (!ignore.count(node)) {
					for (int neighbor : GetNodeNeighbors(node)) {
						if (!visited.count(neighbor)) {
							newFrontier.insert(neighbor); // New frontier = neighbors of frontier nodes that haven't yet been visited.
						}
					}
				}
			}
			frontier.clear();
			SetToVec(newFrontier, frontier);  // Clear the frontier, then load the newFrontier into the frontier.
			hops++;
		}
		// New code to gather metrics about each step. This addition (in particular tracking the average cluster degree at each step) will probably increase the running time by a bit.
		int clusterCount = cellsNodes.size();
		float sizeTotal = 0;
		float degreeTotal = 0;
		float maxClusterSize = 0;
		float maxClusterDegree = 0;
		for (size_t i = 0; i<cellsNodes.size(); i++) {
			sizeTotal += cellSizes[i];
			if (cellsNodes[i].size() > maxClusterSize){maxClusterSize = cellsNodes[i].size();}
			degreeTotal += cellNeighbors[i].size();
			if (cellNeighbors[i].size() > maxClusterDegree){maxClusterDegree = cellNeighbors[i].size();}
		}
		outfileCounts << std::to_string(step)+", "+std::to_string(active_cells.size())+", "+std::to_string(distanceQueryCount-previousQueryCount)+", "+std::to_string(sizeTotal/clusterCount)+", "+std::to_string(degreeTotal/clusterCount)+", "+std::to_string(maxClusterSize)+", "+std::to_string(maxClusterDegree)+"\n";

	}

	std::vector<std::vector<int>> ExhaustiveQuery(std::unordered_set<int> subgraph) {
		// Find and return all the edges contained in subgraph and count the number of queries necessary to do this.
		std::vector<std::vector<int>> edges(GetNodeCount());
		for (int node : subgraph) {
			if (subgraph.size() < GetNodeCount()) {
				std::vector<int> neighbors = GetNodeNeighbors(node);
				std::vector<int> trimmed;
				for(int node : neighbors) {
					if (subgraph.count(node)) {
						trimmed.push_back(node);
					}
				}
				edges[node] = trimmed;
			} else {
				edges[node] = GetNodeNeighbors(node);
			}
			for (int node2 : subgraph){
				std::pair<int, int> query;
				if (node < node2) {
					query=std::make_pair(node, node2);
				} else {
					query=std::make_pair(node2, node);
				}
				if(node != node2 && !queriedPairs.count(query)){					
					// A single kth-hop or distance query for each node to every other node.
					kthHopQueryCount += 1;  
					distanceQueryCount += 1;
					queriedPairs.insert(query);
				}
			}
		}
		return edges;
	}
};


// Load a graph from a file according to the following format:
// - first line contains the number of nodes in the graph
// - the rest of the lines contain edge information in the form of two node IDs per line
Graph LoadGraph(std::string fileName) {  
	Graph graph;
	int currentNodeId = 0;
	std::string line;
	std::ifstream inFile;
	inFile.open(fileName);
	if (inFile.is_open()) {
		std::getline(inFile,line);
		graph.SetNodes(std::stoi(line));
		while (std::getline(inFile,line)) {
			if (std::count(line.begin(), line.end(), ' ') != 1) {continue;}  // The only other relevant type of line in the file describes edges, which uniquely contains only two numbers seperated by exactly one space.
			size_t split = line.find(' ');
			int node1 = std::stoi(line.substr(0, split));
			int node2 = std::stoi(line.substr(split+1));
			int nodeId1, nodeId2;

			// Give each node an id from 0 to num_nodes-1, since not all datasets have node IDs in this format.
			if (graph.nodeIds.count(node1)) {
				nodeId1 = graph.nodeIds[node1];
			} else {
				nodeId1 = currentNodeId++;
				graph.nodeIds[node1] = nodeId1;
			}

			if (graph.nodeIds.count(node2)) {
				nodeId2 = graph.nodeIds[node2];
			} else {
				nodeId2 = currentNodeId++;
				graph.nodeIds[node2] = nodeId2;
			}

			graph.AddEdge(nodeId1, nodeId2);
		}
		inFile.close();
	}
	return graph;
}

std::vector<std::unordered_set<int>> Cells(Graph &graph, std::vector<int> &subgraph, size_t cellSize) {
	size_t maxCell = INT_MAX;  // Size of the biggest cell right now.
	
	// Create list of available centers [0, 1, ..., subgraph.size()]
	std::vector<int> available_centers(subgraph.size()); 
	std::iota(available_centers.begin(), available_centers.end(), 0);

	// Keep partitioning the graph into cells until the maximum sized cell has size at most cellSize
	do {
		// Choose a random center that has not been selected before
		int center_idx = std::rand() % available_centers.size();
		int center = subgraph[available_centers[center_idx]];
		available_centers.erase(available_centers.begin()+center_idx);

		graph.Partition(center);

		// Find the maximum sized cell
		maxCell = 0;
		for (size_t k = 0; k < graph.cellsNodes.size(); k++) {
			if (graph.cellsNodes[k].size() > maxCell) {
				maxCell = graph.cellsNodes[k].size();
			}
		}
	} while (maxCell > cellSize and available_centers.size() > 0);
	return graph.cellsNodes;
}

void EdgeUnion(std::vector<std::vector<int>> &baseEdges, std::vector<std::vector<int>> newEdges) {
	// Returns the union of baseEdges and newEdges
	for (size_t node1 = 0; node1 < newEdges.size(); node1++) {
		for (size_t node2 : newEdges[node1]) {
			if (std::find(baseEdges[node1].begin(), baseEdges[node1].end(), node2) == baseEdges[node1].end()) {
				baseEdges[node1].push_back(node2);
				baseEdges[node2].push_back(node1);
			}
		}
	}
}


std::vector<std::vector<int>> Reconstruct(Graph &graph, std::vector<int> &subgraph) {
	std::vector<std::vector<int>> edges(graph.GetNodeCount(), std::vector<int>());
	std::vector<std::unordered_set<int>> cells = Cells(graph, subgraph, 100);
	for (auto cell : cells) {
		EdgeUnion(edges, graph.ExhaustiveQuery(cell));
	}
	return edges;
}

void ReconstructTest(Graph &graph) {
	graph.ClearCounters();
	std::vector<int> subgraph = graph.ForceConnected(); // Our algorithm reconstructs connected graphs only. If an unconnected graph is given as input, the largest connected component is reconstructed instead.
	outfileResults << "Starting a new reconstruction...\n";
	outfileResults << "\tNumber of Nodes: "+std::to_string(graph.GetNodeCount())+"\n";
	outfileResults << "\tNumber of Connected Nodes: "+std::to_string(subgraph.size())+"\n";
	outfileResults << "\tMaximum Degree: "+std::to_string(graph.GetMaximumDegree())+"\n";
	clock_t time = clock();
	auto reconstruction = Reconstruct(graph, subgraph);
	time = clock() - time;
	outfileResults << "\tReconstruction correctness: "+std::to_string(graph.VerifyReconstruction(reconstruction))+"\n";
	outfileResults << "\tReconstruction Time: "+std::to_string(((float)time)/CLOCKS_PER_SEC)+" seconds\n";
	outfileResults << "\tDistance Query Count: "+std::to_string(graph.distanceQueryCount)+"\n";
}

int main(int argc, char *argv[]) {
	std::string file_name = "";
	file_name += argv[1];
	outfileResults.open("out/"+file_name+".results");
	outfileCounts.open("out/"+file_name+".csv");
	outfileCounts << "Step, Cells Searched, Distance Queries Made, Average Cluster Size, Average Cluster Degree, Max Cluster Size, Max Cluster Degree\n";
	Graph graph = LoadGraph("data/"+file_name+".tmp");
	ReconstructTest(graph);
	outfileResults.close();
	outfileCounts.close();
	return 0;
}

