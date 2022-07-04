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
	size_t distanceQueryCount = 0;
	size_t kthHopQueryCount = 0;
	std::set<std::pair<int, int>> queriedPairs; // Stores vertex pairs that have already been queried
	std::unordered_map<int, int> nodeIds; // Stores mapping of node id's in input file to 0-indexed node id's which we use in our implementation.

	void SetNodes(int count) { // Initialize with a specified number of nodes.
		edges = std::vector<std::vector<int>>(count, std::vector<int>());
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

std::pair<int, int> constructQueryPair(int node1, int node2) {
	if (node1 < node2) {
		return std::make_pair(node1, node2);
	} else {
		return std::make_pair(node2, node1);
	}
}


std::vector<std::vector<int>> Reconstruct_ESA(Graph &graph, std::vector<int> &subgraph, int s) {
	std::vector<std::vector<int>> edges(graph.GetNodeCount(), std::vector<int>());
	std::vector<int> S;
	while (s--) {
		S.push_back(std::rand() % subgraph.size());
	}
	for (int node1: S) {
		for (int node2: subgraph) {
			std::pair<int, int> query = constructQueryPair(node1, node2);
			if(node1 != node2 && !graph.queriedPairs.count(query)){
				graph.distanceQueryCount += 1;
				graph.queriedPairs.insert(query);
			}

		}
	}
	std::set<std::pair<int,int>> E_hat;
	std::vector<std::vector<int>> dists(S.size(), std::vector<int>());
	for (int i = 0; i<S.size(); i++) {
		dists[i] = graph.GetDistances({S[i]},0,{},true);
	}

	for (int node1: subgraph) {
		for (int node2: subgraph) {
			bool add = true;
			for (int i = 0; i<S.size(); i++) {
				if (std::abs(dists[i][node1] - dists[i][node2]) > 1) {
					add = false;
					break;
				}
			}
			if (add) {
				E_hat.insert(constructQueryPair(node1, node2));
			}
		}
	}
	for (const auto& elem: E_hat) {
		int node1 = elem.first; int node2 = elem.second;
		std::vector<int> neighbors = graph.GetNodeNeighbors(node1);
		for (int node: neighbors) {
			if (node == node2) {
				edges[node1].push_back(node2);
				edges[node2].push_back(node1);
			}
		}
		std::pair<int, int> query = constructQueryPair(node1, node2);
		if(node1 != node2 && !graph.queriedPairs.count(query)){
			graph.distanceQueryCount += 1;
			graph.queriedPairs.insert(query);
		}
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
	// auto reconstruction = Reconstruct_ESA(graph, subgraph, std::sqrt(std::pow(std::log2(subgraph.size()), 2)*std::pow(subgraph.size(), 2.0/3.0)));
	auto reconstruction = Reconstruct_ESA(graph, subgraph, std::pow(subgraph.size(), 2.0/3.0));
	// auto reconstruction = Reconstruct_ESA(graph, subgraph, std::pow(std::log2(subgraph.size()), 2));

	time = clock() - time;
	outfileResults << "\tReconstruction correctness: "+std::to_string(graph.VerifyReconstruction(reconstruction))+"\n";
	outfileResults << "\tReconstruction Time: "+std::to_string(((float)time)/CLOCKS_PER_SEC)+" seconds\n";
	outfileResults << "\tDistance Query Count: "+std::to_string(graph.distanceQueryCount)+"\n";
	outfileResults << "\tKth-Hop Query Count: "+std::to_string(graph.kthHopQueryCount)+"\n";
}

int main(int argc, char *argv[]) {
	std::string file_name = "";
	file_name += argv[1];
	outfileResults.open("out_ESA/"+file_name+".results");
	Graph graph = LoadGraph("data/"+file_name+".tmp");
	ReconstructTest(graph);
	outfileResults.close();
	return 0;
}

