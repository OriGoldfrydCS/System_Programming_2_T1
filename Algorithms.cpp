#include "Algorithms.hpp"
#include <queue>
#include <sstream>
#include <algorithm>
#include <climits>
#include <limits> 
#include <unordered_set>
#include <iostream>
#include <stack>  

using namespace std;

namespace ariel {
    
    /**
     * This function checks if all vertices in a given graph (g) are reachable from the vertex with index 0 by using BFS algorithm.
     *
     * @param graph The graph to check.
     * @return true if all vertices are reachable; otherwise, false.
     */
    bool Algorithms::isConnected(Graph& graph) {
        size_t numVertices = graph.getNumVertices();

        if (numVertices == 0) {
            return true;
        }

        vector<bool> visited(numVertices, false);
        size_t count = bfs(graph, 0, visited);

        return count == numVertices;
    }

    /**
     * This function checks if a given graph is strongly connected.
     *
     * @param graph The graph to check.
     * @return true if the graph is strongly connected; otherwise, false.
     */
    bool Algorithms::isStronglyConnected(Graph& graph) {
        size_t numVertices = graph.getNumVertices();

        if (numVertices == 0) {
            return true;
        }

        // Perform BFS from each vertex
        for (size_t startVertex = 0; startVertex < numVertices; startVertex++) {
            vector<bool> visited(numVertices, false);
            size_t count = bfs(graph, startVertex, visited);

            if (count != numVertices) {
                return false;
            }
        }

        return true;
    }

    string Algorithms::shortestPath(Graph& graph, size_t start, size_t end) {
        size_t numVertices = graph.getNumVertices();

        // Error checking
        if (start >= numVertices || end >= numVertices) {
            return "Invalid start or end vertex";
        }

        if(start == end){
            return "No path exists between a vertex and itself";
        }

        // Check if the graph is unweighted
        bool isUnweighted = true;
        for (size_t i = 0; i < numVertices; i++) {
            for (size_t j = 0; j < numVertices; j++) {
                if (graph.getAdjacencyMatrix()[i][j] != 0 && graph.getAdjacencyMatrix()[i][j] != 1) {
                    isUnweighted = false;
                    break;
                }
            }
            if (!isUnweighted) {
                break;
            }
        }

        // Choose the appropriate algorithm based on the graph type
        if (isUnweighted) {
            return bfsShortestPath(graph, start, end);
        } else {
            // Check if the graph has negative edges
            bool hasNegativeEdges = false;
            for (size_t i = 0; i < numVertices; i++) {
                for (size_t j = 0; j < numVertices; j++) {
                    if (graph.getAdjacencyMatrix()[i][j] < 0) {
                        hasNegativeEdges = true;
                        break;
                    }
                }
                if (hasNegativeEdges) {
                    break;
                }
            }

            if (hasNegativeEdges) {
                return bellmanFordShortestPath(graph, start, end);
            } else {
                return dijkstraShortestPath(graph, start, end);        
            }
        }
    }

    
    string Algorithms::isContainsCycle(Graph& graph) {
        size_t numVertices = graph.getNumVertices();
        vector<bool> visited(numVertices, false);
        vector<size_t> parent(numVertices, numeric_limits<size_t>::max());

        for (size_t v = 0; v < numVertices; v++) {
            if (!visited[v]) {
                string cycle = dfs_cycle(graph, v, visited, parent, v);
                if (!cycle.empty()) {
                    return cycle;
                }
            }
        }

        return "0";
    }
    
    string Algorithms::isBipartite(Graph& graph) {
       size_t numVertices = graph.getNumVertices();
        std::vector<int> color(numVertices, -1);  // -1 indicates uncolored

        for (size_t i = 0; i < numVertices; ++i) {
            if (color[i] == -1) {  // Start DFS only if the vertex is not colored
                if (!dfsCheck(graph, i, color, 0)) {
                    return "The graph is not bipartite";
                }
            }
        }

        // Collecting sets A and B for output, represented as BLUE and WHITE
        std::vector<size_t> setA, setB;
        for (size_t i = 0; i < numVertices; ++i) {
            if (color[i] == 0) setA.push_back(i);  // BLUE
            else if (color[i] == 1) setB.push_back(i);  // WHITE
        }

        std::stringstream result;
        result << "The graph is bipartite: A={";
        for (size_t i = 0; i < setA.size(); i++) {
            result << setA[i];
            if (i < setA.size() - 1) result << ",";
        }
        result << "}, B={";
        for (size_t i = 0; i < setB.size(); i++) {
            result << setB[i];
            if (i < setB.size() - 1) result << ",";
        }
        result << "}";
        return result.str();
    }


    bool Algorithms::dfsCheck(Graph& graph, size_t u, std::vector<int>& color, int col) {
        color[u] = col;  // Color the vertex

        for (size_t v = 0; v < graph.getNumVertices(); ++v) {
            if (graph.getAdjacencyMatrix()[u][v] == 0 && u != v) {  // Check for non-edge
                if (color[v] == -1) {   // If vertex v is not colored
                    color[v] = col;     // Color the same as u
                    if (!dfsCheck(graph, v, color, col)) {
                        return false;
                    }
                }
            } else if (graph.getAdjacencyMatrix()[u][v] != 0) {  // If there's an actual edge
                if (color[v] == -1) {
                    color[v] = 1 - col;  // Assign opposite color
                    if (!dfsCheck(graph, v, color, 1 - col)) {
                        return false;
                    }
                } else if (color[v] == col) {
                    return false;
                }
            }
        }
        return true;
    }
    


    string Algorithms::negativeCycle(Graph& graph) {
    size_t numVertices = graph.getNumVertices();
    vector<int> distance(numVertices, 0);
    vector<size_t> parent(numVertices, numeric_limits<size_t>::max());
    bool hasNegativeCycle = false;

        // Part 1: Check for negative self-loops
    for (size_t u = 0; u < numVertices; u++) {
        if (graph.getAdjacencyMatrix()[u][u] < 0) {
            hasNegativeCycle = true;
            return to_string(u) + "->" + to_string(u);
        }
    }

    // Part 2: Check for negative cycles between neighboring vertices with different edge weights
    for (size_t u = 0; u < numVertices; u++) {
        for (size_t v = 0; v < numVertices; v++) {
            if (u != v && graph.getAdjacencyMatrix()[u][v] != 0 && graph.getAdjacencyMatrix()[v][u] != 0 &&
                graph.getAdjacencyMatrix()[u][v] != graph.getAdjacencyMatrix()[v][u]) {
                int weight = graph.getAdjacencyMatrix()[u][v] + graph.getAdjacencyMatrix()[v][u];
                if (weight < 0) {
                    hasNegativeCycle = true;
                    return to_string(u) + "->" + to_string(v) + "->" + to_string(u);
                }
            }
        }
    }

    // Part 3: Relax edges V times to find negative cycles with distance greater than one edge
    for (size_t i = 0; i < numVertices; i++) {
        for (size_t u = 0; u < numVertices; u++) {
            for (size_t v = 0; v < numVertices; v++) {
                int weight = graph.getAdjacencyMatrix()[u][v];
                if (weight != 0 && distance[u] + weight < distance[v]) {
                    distance[v] = distance[u] + weight;
                    parent[v] = u;

                    // Check for negative cycle
                    if (i == numVertices - 1) {
                        hasNegativeCycle = true;
                    }
                }
            }
        }
    }

    // Part 4: If a negative cycle is found, trace back the cycle and format the cycle string
    if (hasNegativeCycle) {
        size_t startVertex = 0;

        // Find a vertex that is part of the negative cycle
        for (size_t v = 0; v < numVertices; v++) {
            if (parent[v] != numeric_limits<size_t>::max()) {
                startVertex = v;
                break;
            }
        }

        vector<size_t> cycle;
        unordered_set<size_t> visited;
        size_t current = startVertex;

        // Trace back to find the cycle
        while (visited.find(current) == visited.end()) {
            visited.insert(current);
            cycle.push_back(current);
            current = parent[current];
        }
        cycle.push_back(current);

        // Check if the cycle involves vertices with the same weight
        bool isValidCycle = false;
        for (size_t i = 0; i < cycle.size() - 1; i++) {
            size_t u = cycle[i];
            size_t v = cycle[i + 1];
            if (graph.getAdjacencyMatrix()[u][v] != graph.getAdjacencyMatrix()[v][u]) {
                isValidCycle = true;
                break;
            }
        }

        if (isValidCycle) {
            // Format the cycle string
            stringstream cycleString;
            cycleString << cycle[cycle.size() - 1];
            for (size_t i = cycle.size() - 2; i > 0; i--) {
                cycleString << "->" << cycle[i];
            }

            return cycleString.str();
        }
    }

    return "No negative cycle exists";
}


    /*********************************************/
    ///             PRIVATE SECTION             ///
    /*********************************************/

    /**
     * Performs BFS on the graph starting from the given vertex.
     *
     * @param graph The graph to perform BFS on.
     * @param startVertex The starting vertex for BFS.
     * @param visited A vector to track visited vertices.
     * @return The number of vertices visited during BFS.
     */
    size_t Algorithms::bfs(Graph& graph, size_t startVertex, vector<bool>& visited) {
        queue<size_t> queue;
        visited[startVertex] = true;
        queue.push(startVertex);
        size_t count = 1;

        while (!queue.empty()) {
            size_t currentVertex = queue.front();
            queue.pop();

            for (size_t i = 0; i < graph.getNumVertices(); i++) {
                if (graph.getAdjacencyMatrix()[currentVertex][i] != 0 && !visited[i]) {
                    visited[i] = true;
                    queue.push(i);
                    count++;
                }
            }
        }

        return count;
    }



    string Algorithms::bfsShortestPath(Graph& graph, size_t start, size_t end) {
        size_t numVertices = graph.getNumVertices();
        vector<bool> visited(numVertices, false);
        vector<size_t> parent(numVertices, numeric_limits<size_t>::max());

        queue<size_t> queue;
        visited[start] = true;
        queue.push(start);

        while (!queue.empty()) {
            size_t currentVertex = queue.front();
            queue.pop();

            if (currentVertex == end) {
                break;
            }

            for (size_t i = 0; i < numVertices; i++) {
                if (graph.getAdjacencyMatrix()[currentVertex][i] != 0 && !visited[i]) {
                    visited[i] = true;
                    queue.push(i);
                    parent[i] = currentVertex;
                }
            }
        }

        if (!visited[end]) {
            return "No path exists between " + to_string(start) + " and " + to_string(end);
        }

        // Reconstruct the shortest path
        string path;
        size_t current = end;
        while (current != start) {
            path = "->" + to_string(current) + path;
            current = parent[current];
        }
        path = to_string(start) + path;

        return path;
    }


    string Algorithms::bellmanFordShortestPath(Graph& graph, size_t start, size_t end) {
        size_t numVertices = graph.getNumVertices();
        vector<int> distance(numVertices, numeric_limits<int>::max());
        vector<size_t> parent(numVertices, numeric_limits<size_t>::max());

        distance[start] = 0;

        // Relax edges V-1 times
        for (int i = 0; i < numVertices - 1; i++) {
            for (size_t u = 0; u < numVertices; u++) {
                for (size_t v = 0; v < numVertices; v++) {
                    int weight = graph.getAdjacencyMatrix()[u][v];
                    if (weight != 0 && distance[u] != numeric_limits<int>::max() && distance[u] + weight < distance[v]) {
                        distance[v] = distance[u] + weight;
                        parent[v] = u;
                    }
                }
            }
        }

        // Check for negative cycle on the V-th iteration
        for (size_t u = 0; u < numVertices; u++) {
            for (size_t v = 0; v < numVertices; v++) {
                int weight = graph.getAdjacencyMatrix()[u][v];
                if (weight != 0 && distance[u] != numeric_limits<int>::max() && distance[u] + weight < distance[v]) {
                    return "Graph contains a negative cycle";
                }
            }
        }

        if (distance[end] == numeric_limits<int>::max()) {
            return "No path exists between " + to_string(start) + " and " + to_string(end);
        }

        // Reconstruct the shortest path
        string path;
        size_t current = end;
        while (current != start) {
            path = "->" + to_string(current) + path;
            current = parent[current];
        }
        path = to_string(start) + path;

        return path;
    }



    string Algorithms::dijkstraShortestPath(Graph& graph, size_t start, size_t end) {
        size_t numVertices = graph.getNumVertices();
        vector<int> distance(numVertices, numeric_limits<int>::max());
        vector<size_t> parent(numVertices, numeric_limits<size_t>::max());
        vector<bool> visited(numVertices, false);

        distance[start] = 0;

        for (size_t i = 0; i < numVertices; i++) {
            size_t minVertex = numeric_limits<size_t>::max();
            int minDistance = numeric_limits<int>::max();

            for (size_t v = 0; v < numVertices; v++) {
                if (!visited[v] && distance[v] < minDistance) {
                    minVertex = v;
                    minDistance = distance[v];
                }
            }

            if (minVertex == numeric_limits<size_t>::max()) {
                break;
            }

            visited[minVertex] = true;

            for (size_t v = 0; v < numVertices; v++) {
                int weight = graph.getAdjacencyMatrix()[minVertex][v];
                if (weight != 0 && !visited[v] && distance[minVertex] != numeric_limits<int>::max() &&
                    distance[minVertex] + weight < distance[v]) {
                    distance[v] = distance[minVertex] + weight;
                    parent[v] = minVertex;
                }
            }
        }

        if (distance[end] == numeric_limits<int>::max()) {
            return "No path exists between " + to_string(start) + " and " + to_string(end);
        }

        // Reconstruct the shortest path
        string path;
        size_t current = end;
        while (current != start) {
            path = "->" + to_string(current) + path;
            current = parent[current];
        }
        path = to_string(start) + path;

        return path;
    }

    string Algorithms::dfs_cycle(Graph& graph, size_t v, vector<bool>& visited, vector<size_t>& parent, size_t start) {
        visited[v] = true;

        for (size_t i = 0; i < graph.getNumVertices(); i++) {
            if (graph.getAdjacencyMatrix()[v][i] != 0) {
                if (!visited[i]) {
                    parent[i] = v;
                    string cycle = dfs_cycle(graph, i, visited, parent, start);
                    if (!cycle.empty()) {
                        return cycle;
                    }
                } else if (i != parent[v] && i == start) {
                    // Found a cycle
                    string cycle = to_string(i);
                    size_t current = v;
                    while (current != i) {
                        cycle = to_string(current) + "->" + cycle;
                        current = parent[current];
                    }
                    cycle = to_string(i) + "->" + cycle;
                    return cycle;
                }
            }
        }

        return "";
    }

//     bool Algorithms::bfs_bipartite(Graph& graph, size_t start, vector<int>& color) {
//     queue<size_t> queue;
//     queue.push(start);
//     color[start] = 0;

//     while (!queue.empty()) {
//         size_t vertex_v = queue.front();
//         queue.pop();

//         for (size_t vertex_w = 0; vertex_w < graph.getNumVertices(); ++vertex_w) {
//             if (graph.getAdjacencyMatrix()[vertex_v][vertex_w] != 0) {
//                 if (color[vertex_w] == -1) {
//                     color[vertex_w] = 1 - color[vertex_v];
//                     queue.push(vertex_w);
//                 } else if (color[vertex_w] == color[vertex_v]) {
//                     return false;
//                 }
//             }
//         }
//     }

//     return true;
// }

}