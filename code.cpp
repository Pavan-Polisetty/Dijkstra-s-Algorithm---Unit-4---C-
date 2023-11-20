#include <iostream>
#include <vector>
#include <queue>
#include <limits>

using namespace std;

// Structure to represent a node and its associated weight
struct Edge {
    int destination;
    int weight;
};

// Dijkstra's Algorithm
vector<int> dijkstra(const vector<vector<Edge>>& graph, int startNode, int numNodes) {
    vector<int> dist(numNodes, numeric_limits<int>::max()); // Initialize distances to infinity
    dist[startNode] = 0; // Set distance to the starting node as 0
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq; // Min heap to store nodes by their distance
    pq.push({0, startNode}); // Push the starting node with distance 0

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        for (const Edge& edge : graph[u]) {
            int v = edge.destination;
            int weight = edge.weight;

            // Relaxation step
            if (dist[v] > dist[u] + weight) {
                dist[v] = dist[u] + weight;
                pq.push({dist[v], v});
            }
        }
    }

    return dist;
}

int main() {
    // Input the number of nodes and edges
    int numNodes, numEdges;
    cout << "Enter the number of nodes and edges: ";
    cin >> numNodes >> numEdges;

    // Create an adjacency list to represent the graph
    vector<vector<Edge>> graph(numNodes);

    // Input edges and their weights
    cout << "Enter edges and their weights (source destination weight):" << endl;
    for (int i = 0; i < numEdges; ++i) {
        int source, destination, weight;
        cin >> source >> destination >> weight;
        graph[source].push_back({destination, weight});
        // For undirected graphs, uncomment the line below
        // graph[destination].push_back({source, weight});
    }

    // Input the starting and ending nodes
    int startNode, endNode;
    cout << "Enter starting node and ending node: ";
    cin >> startNode >> endNode;

    // Find the shortest distances using Dijkstra's algorithm
    vector<int> shortestDistances = dijkstra(graph, startNode, numNodes);

    // Output the shortest distance from the starting node to the ending node
    cout << "Shortest distance from node " << startNode << " to node " << endNode << ": " << shortestDistances[endNode] << endl;

    return 0;
}