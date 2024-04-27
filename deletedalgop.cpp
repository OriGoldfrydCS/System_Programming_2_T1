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
     * 
     * This function checks if all vertices in a given graph (g) are reachable from the vertex with index 0 by using BFS algorithm.
     * 
     * @param graph The graph to check.
     * @return true if all vertices are reachable; otherwise, false.
     * 
     */
    bool Algorithms::isConnected(Graph& graph) {
        size_t numVertices = graph.getNumVertices();       // Get the number of vertices in g
        
        // If the graph is empty, it is connected trivially
        if (numVertices == 0){
            return true;
        }
        
        vector<bool> visited(numVertices, false);      // Track visited vertices
        queue<int> queue;                              // Queue for BFS
        
        // Start BFS from vertex 0
        visited[0] = true;
        queue.push(0);
        int count = 1;
        
        // Perform BFS
        while (!queue.empty()) {
            int currentVertex = queue.front();
            queue.pop();
            
            // Check all adjacent vertices
            for (int i = 0; i < numVertices; i++) {
                if (graph.getAdjacencyMatrix()[currentVertex][i] != 0 && !visited[i]) {
                    visited[i] = true;
                    queue.push(i);
                    count++;
                }
            }
        }
        
        // The graph is connected if all vertices were visited
        return count == numVertices;
    }
    

    /**
     * 
     * This function finds the shortest path from a start vertex to an end vertex by using Bellman-Ford algorithm.
     * 
     * @param graph The graph to search.
     * @param start The starting vertex.
     * @param end The destination vertex.
     * @return A string representing the path or "-1" if no path exists.
     * 
     */
    string Algorithms::shortestPath(Graph& graph, size_t start, size_t end) {
        int numVertices = graph.getNumVertices();

        // Error checking: start and end vertices should be within the valid range
        if (start < 0 || start >= numVertices || end < 0 || end >= numVertices) {
            string errorMessage = "Invalid ";
            if (start < 0 || start >= numVertices) {
                errorMessage += "start vertex: " + to_string(start);
            }
            if (start < 0 || start >= numVertices && end < 0 || end >= numVertices) {
                errorMessage += " and ";
            }
            if (end < 0 || end >= numVertices) {
                errorMessage += "end vertex: " + to_string(end);
            }
            return errorMessage;
        }

        // Check if the graph is unweighted
        bool isUnweighted = true;
        const auto& adjMatrix = graph.getAdjacencyMatrix();
        for (size_t i = 0; i < numVertices; i++) {
            for (size_t j = 0; j < numVertices; j++) {
                if (adjMatrix[i][j] < 0) {
                    isUnweighted = false;
                    break;
                }
            }
            if (!isUnweighted) {
                break;
            }
        }

        if (isUnweighted) {
            // If the graph is unweighted, use BFS
            return bfsShortestPath(graph, start, end);
        } else {
            // If the graph is weighted, check for negative edges
            bool hasNegativeEdges = false;
            for (size_t i = 0; i < numVertices; i++) {
                for (size_t j = 0; j < numVertices; j++) {
                    if (adjMatrix[i][j] < 0) {
                        hasNegativeEdges = true;
                        break;
                    }
                }
                if (hasNegativeEdges) {
                    break;
                }
            }

            if (hasNegativeEdges) {
                // If the graph has negative edges, use Bellman-Ford
                return bellmanFordShortestPath(graph, start, end);
            } else {
                // If the graph is weighted without negative edges, use Dijkstra
                return dijkstraShortestPath(graph, start, end);
            }
        }
    }

    string Algorithms::bfsShortestPath(Graph& graph, size_t start, size_t end) {
        int numVertices = graph.getNumVertices();
        vector<bool> visited(numVertices, false);
        vector<int> parent(numVertices, -1);
        queue<int> q;

        visited[start] = true;
        q.push(start);

        while (!q.empty()) {
            int curr = q.front();
            q.pop();

            if (curr == end) {
                // Path found, build the path string
                string path;
                while (curr != start) {
                    path = to_string(curr) + "->" + path;
                    curr = parent[curr];
                }
                path = to_string(start) + "->" + path;
                path.pop_back();
                path.pop_back();
                return path;
            }

            const auto& adjList = graph.getAdjacencyMatrix();
            for (size_t neighbor = 0; neighbor < numVertices; neighbor++) {
                if (adjList[curr][neighbor] != 0 && !visited[neighbor]) {
                    visited[neighbor] = true;
                    parent[neighbor] = curr;
                    q.push(neighbor);
                }
            }
        }

        // No path found
        return "No path exists between " + to_string(start) + " and " + to_string(end);
    }

    string Algorithms::bellmanFordShortestPath(Graph& graph, size_t start, size_t end) {
        int numVertices = graph.getNumVertices();
        vector<int> dist(numVertices, numeric_limits<int>::max());
        vector<int> parent(numVertices, -1);

        dist[start] = 0;

        for (size_t i = 0; i < numVertices - 1; i++) {
            for (size_t u = 0; u < numVertices; u++) {
                for (size_t v = 0; v < numVertices; v++) {
                    int weight = graph.getAdjacencyMatrix()[u][v];
                    if (weight != 0 && dist[u] != numeric_limits<int>::max() && dist[u] + weight < dist[v]) {
                        dist[v] = dist[u] + weight;
                        parent[v] = u;
                    }
                }
            }
        }

        // Check for negative cycles
        for (size_t u = 0; u < numVertices; u++) {
            for (size_t v = 0; v < numVertices; v++) {
                int weight = graph.getAdjacencyMatrix()[u][v];
                if (weight != 0 && dist[u] != numeric_limits<int>::max() && dist[u] + weight < dist[v]) {
                    return "Graph contains a negative cycle";
                }
            }
        }

        if (dist[end] == numeric_limits<int>::max()) {
            return "No path exists between " + to_string(start) + " and " + to_string(end);
        }

        string path;
        size_t curr = end;
        while (curr != start) {
            path = to_string(curr) + "->" + path;
            curr = parent[curr];
        }
        path = to_string(start) + "->" + path;
        path.pop_back();
        path.pop_back();
        return path;
    }

    string Algorithms::dijkstraShortestPath(Graph& graph, size_t start, size_t end) {
        int numVertices = graph.getNumVertices();
        vector<int> dist(numVertices, numeric_limits<int>::max());
        vector<bool> visited(numVertices, false);
        vector<int> parent(numVertices, -1);

        dist[start] = 0;

        for (size_t i = 0; i < numVertices; i++) {
            size_t u = -1;
            for (size_t j = 0; j < numVertices; j++) {
                if (!visited[j] && (u == -1 || dist[j] < dist[u])) {
                    u = j;
                }
            }

            if (u == end) {
                break;
            }

            visited[u] = true;

            for (size_t v = 0; v < numVertices; v++) {
                int weight = graph.getAdjacencyMatrix()[u][v];
                if (weight != 0 && dist[u] != numeric_limits<int>::max() && dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    parent[v] = u;
                }
            }
        }

        if (dist[end] == numeric_limits<int>::max()) {
            return "No path exists between " + to_string(start) + " and " + to_string(end);
        }

        string path;
        size_t curr = end;
        while (curr != start) {
            path = to_string(curr) + "->" + path;
            curr = parent[curr];
        }
        path = to_string(start) + "->" + path;
        path.pop_back();
        path.pop_back();
        return path;
    }


    /**
     * 
     * This function checks if the graph contains a cycle.
     * 
     * @param graph The graph.
     * @return true if a cycle is found; otherwise, false.
     * @note see also the auxiliary functions hasCycle, dfs_cycle and printCycle in the private section below.
     * 
     */
    bool Algorithms::isContainsCycle(Graph& graph) {
        if (hasCycle(graph)) {
            cout << "1" << endl;
            return true;
        } 
        
        cout << "0" << endl;
        return false;
    }

    

     /**
     * 
     * This function finds if the graph is bipartite.
     * 
     * @param graph The graph.
     * @return A string representing the bipartite sets or "0" if not bipartite.
     * @note see also the auxiliary function bfs_bipartite in the private section below.
     * 
     */
    string Algorithms::isBipartite(Graph& graph) {
        const auto& adj = graph.getAdjacencyMatrix();
        vector<int> color(static_cast<size_t>(graph.getNumVertices()), -1);

        for (int vertex_v = 0; vertex_v < graph.getNumVertices(); ++vertex_v) {
            if (color[static_cast<size_t>(vertex_v)] == -1) {
                if (!bfs_bipartite(graph, vertex_v, color)) {
                    return "0";
                }
            }
        }

        vector<int> setA;
        vector<int> setB;

        for (int vertex_v = 0; vertex_v < graph.getNumVertices(); ++vertex_v) {
            if (color[static_cast<size_t>(vertex_v)] == 0) {
                setA.push_back(vertex_v);
            } else {
                setB.push_back(vertex_v);
            }
        }

        stringstream result;
        result << "The graph is bipartite: A={";
        for (size_t i = 0; i < setA.size(); ++i) {
            result << setA[i];
            if (i < setA.size() - 1) {
                result << ", ";
            }
        }
        result << "}, B={";
        for (size_t i = 0; i < setB.size(); ++i) {
            result << setB[i];
            if (i < setB.size() - 1) {
                result << ", ";
            }
        }
        result << "}";
        return result.str();
    }

    
    /**
     * 
     * This function checks for negative weight cycles.
     * 
     * @param graph The graph.
     * @return true if the graph has a negative weight cycle; otherwise, false.
     * @note see also the auxiliary function detectNegativeCycle in the private section below.
     * 
     */
    bool Algorithms::negativeCycle(Graph& graph) {
        return detectNegativeCycle(graph);
    }



    /*********************************************/
    ///             PRIVATE SECTION             ///
    /*********************************************/

    /**
     * 
     * This function detects if the graph contains any cycle.
     * 
     * @param graph The graph to check.
     * @return true if the graph contains a cycle; otherwise, false.
     * @note this is an auxiliary function for bool Algorithms::isContainsCycle(Graph& g) [see above]
     * 
     **/
    bool Algorithms::hasCycle(Graph& graph) {
       size_t numVertices = static_cast<size_t>(graph.getNumVertices());
        vector<bool> visited(numVertices, false);
        vector<int> parent(numVertices, -1);    // Parent vector for tracking the path of vertices

        // Check each vertex for cycles
        for (size_t vertex_v = 0; vertex_v < numVertices; ++vertex_v) {
            if (!visited[vertex_v] && dfs_cycle(graph, static_cast<int>(vertex_v), visited, parent, -1)) {
                return true;
            }
        }
        return false;
    }



    /**
     * 
     * An auxiliary function for cycle detection using DFS algorithm.
     * 
     * @param graph The graph.
     * @param currect_v The current vertex.
     * @param visited Array to keep track of visited vertices.
     * @param parent Array to store the parent of each vertex.
     * @param prev The previous vertex.
     * @return true if a cycle is found; otherwise, false.
     * @note this is an auxiliary function for the hasCycle [see above].
     * 
     **/
    bool Algorithms::dfs_cycle(Graph& graph, size_t currect_v, vector<bool>& visited, vector<int>& parent, size_t prev) {
        visited[static_cast<size_t>(currect_v)] = true;
        const auto& adj = graph.getAdjacencyMatrix();

        // Explore each adjacent vertex
        for (size_t i = 0; i < adj[static_cast<size_t>(currect_v)].size(); ++i) {
            if (adj[static_cast<size_t>(currect_v)][i] != 0) {
                if (!visited[i]) {
                    parent[i] = currect_v;
                    if (dfs_cycle(graph, static_cast<int>(i), visited, parent, currect_v)) {
                        return true;
                    }
                } else if (static_cast<int>(i) != prev) {
                    printCycle(static_cast<int>(i), currect_v, parent);
                    return true;
                }
            }
        }

        return false;
    }



    /**
     * This function prints the cycle found in the graph (if found).
     * 
     * @param start The starting vertex of the cycle.
     * @param end The ending vertex of the cycle.
     * @param parent The array containing the parent of each vertex.
     * @note this is an auxiliary function for the dfs_cycle function [see above].
     * 
     **/
    void Algorithms::printCycle(size_t start, size_t end, vector<int>& parent) {
        vector<int> cycle;
        int current = end;
        cycle.push_back(current);

        // Trace back the cycle path
        while (current != start) {
            current = parent[static_cast<size_t>(current)];
            cycle.push_back(current);
        }

        reverse(cycle.begin(), cycle.end());

        cout << "Cycle found: ";
        for (size_t i = 0; i < cycle.size(); ++i) {
            if (i != 0) {
                cout << "->";
            }
            cout << cycle[i];
        }
        cout << "->" << cycle[0] << endl;  // Print the start vertex at the end to complete the cycle
    }



    /**
     * 
     * An auxiliary function for BFS used in bipartite checking.
     * 
     * @param graph The graph.
     * @param start The starting vertex for BFS.
     * @param color Array to store the color of each vertex.
     * @return true if successful; otherwise, false.
     * @note this is an auxiliary function for string Algorithms::isBipartite(Graph& g) [see above].
     */
    bool Algorithms::bfs_bipartite(Graph& graph, size_t start, vector<int>& color) {
        queue<int> queue;
        queue.push(start);
        color[static_cast<size_t>(start)] = 0;

        while (!queue.empty()) {
            int vertex_v = queue.front();
            queue.pop();

            for (int vertex_w = 0; vertex_w < graph.getNumVertices(); ++vertex_w) {
                if (graph.getAdjacencyMatrix()[static_cast<size_t>(vertex_v)][static_cast<size_t>(vertex_w)] != 0) {
                    if (color[static_cast<size_t>(vertex_w)] == -1) {
                        color[static_cast<size_t>(vertex_w)] = 1 - color[static_cast<size_t>(vertex_v)];
                        queue.push(vertex_w);
                    } else if (color[static_cast<size_t>(vertex_w)] == color[static_cast<size_t>(vertex_v)]) {
                        return false;
                    }
                }
            }
        }
        return true;
    }


    ///^^^^^^
     /**
     * This auxiliary function detects negative weight cycles in the graph using Bellman-Ford algorithm.
     * 
     * @param graph The graph.
     * @return true if a negative cycle is detected; otherwise, false.
     * @note this is an auxiliary function for bool Algorithms::negativeCycle(Graph& g) [see above]
     * 
     */
    bool Algorithms::detectNegativeCycle(Graph& graph) {
        const auto& adj = graph.getAdjacencyMatrix();
        std::size_t numVertices = static_cast<std::size_t>(graph.getNumVertices());

        if (numVertices == 0) {
            return false;
        }

        vector<int> dist(numVertices, INT_MAX);

        for (std::size_t src = 0; src < numVertices; ++src) {
            if (dist[src] == INT_MAX) {
                dist[src] = 0;
                for (std::size_t i = 0; i < numVertices - 1; i++) {
                    if (!relaxEdges(adj, dist, numVertices)){
                        break;
                    }
                }

                if (checkForNegativeCycle(adj, dist, numVertices)) {
                    return true;
                }
            }
        }

        return false;
    }

    bool Algorithms::relaxEdges(vector<vector<int>>& adj, vector<int>& dist, size_t numVertices) {
        bool relaxed = false;
        for (size_t vertex_u = 0; vertex_u < numVertices; vertex_u++) {
            for (size_t vertex_v = 0; vertex_v < numVertices; vertex_v++) {
                if (adj[vertex_u][vertex_v] != 0 && dist[vertex_u] != INT_MAX && dist[vertex_u] + adj[vertex_u][vertex_v] < dist[vertex_v]) {
                    dist[vertex_v] = dist[vertex_u] + adj[vertex_u][vertex_v];
                    relaxed = true;
                }
            }
        }
        return relaxed;
    }

    bool Algorithms::checkForNegativeCycle(vector<vector<int>>& adj, vector<int>& dist, size_t numVertices) {
        for (size_t vertex_u = 0; vertex_u < numVertices; vertex_u++) {
            for (size_t vertex_v = 0; vertex_v < numVertices; vertex_v++) {
                if (adj[vertex_u][vertex_v] != 0 && dist[vertex_u] != INT_MAX && dist[vertex_u] + adj[vertex_u][vertex_v] < dist[vertex_v]) {
                    return true;  // Negative cycle detected
                }
            }
        }
        return false;
    }



    /*********************************************/
    ///              TO BE DELETED              ///
    /*********************************************/

    // bool Algorithms::isBipartiteUtil(int v, vector<vector<int>>& adj, vector<int>& color, vector<bool>& visited) {
    //     std::size_t v_index = static_cast<std::size_t>(v);
    //     queue<int> q;
    //     q.push(v);
    //     color[v_index] = 0; // Start coloring with 0
    //     visited[v_index] = true;

    //     while (!q.empty()) {
    //         int u = q.front(); q.pop();
    //         std::size_t u_index = static_cast<std::size_t>(u);

    //         for (std::size_t i = 0; i < adj[u_index].size(); i++) {
    //             if (adj[u_index][i] != 0 && !visited[i]) {
    //                 color[i] = 1 - color[u_index]; // Alternate color
    //                 visited[i] = true;
    //                 q.push(static_cast<int>(i));
    //             } else if (adj[u_index][i] != 0 && color[i] == color[u_index]) {
    //                 return false;
    //             }
    //         }
    //     }
    //     return true;
    // }
}