#include "Algorithms.hpp"
#include <queue>
#include <sstream>
#include <algorithm>
#include <climits>
#include <unordered_set>
#include <iostream>
#include <stack>  

using namespace std;

namespace ariel {
    
    /**
     * @brief This method checks if all vertices in a given graph are reachable from the vertex with index 0 by using BFS algorithm.
     *
     * @param graph The graph to check.
     * @return true if all vertices are reachable; otherwise, false.
     */
    bool Algorithms::isConnected(Graph& graph) 
    {
        size_t numVertices = graph.getNumVertices();

        // The graph is trivially connected
        if (numVertices == 1) 
        {
            return true;
        }

        vector<bool> visited(numVertices, false);   // Initialize a vector with numVertices elements and sets each of them to false
        bfs(graph, 0, visited);                     // Perform BFS starting from vertex 0

        // If any vertex was not visited, the graph is not connected
        for (bool v : visited) 
        {
            if (!v) 
            {
                return false;
            }
        }

        return true;
    }


    /**
     * This function checks if a given graph is strongly connected by applying BFS from each vertex.
     *
     * @param graph The graph to check.
     * @return true if the graph is strongly connected; otherwise, false.
     */
    bool Algorithms::isStronglyConnected(Graph& graph) 
    {
        size_t numVertices = graph.getNumVertices();

        // The graph is trivially strongly connected
        if (numVertices == 1) 
        {
            return true;
        }

        // Perform BFS from each vertex
        for (size_t startVertex = 0; startVertex < numVertices; startVertex++) {
            vector<bool> visited(numVertices, false);       // Initiate a vector to keep tracking the visited vertices
            bfs(graph, startVertex, visited);

            // If any vertex was not visited, the graph is not strongly connected
            for (bool v : visited) 
            {
                if (!v) {
                    return false;
                }
            }
        }

        return true;
    }


    /**
     * @brief This method finds the shortest path between two vertices in a given graph.
     *
     * @note: The method uses a strategy to choose the appropriate algorithm based on the properties of the graph (weighted/unweighted, containts negative edges or not).
     * It uses BFS for unweighted graphs (time complexity: O(|V|*|E|)), Bellman-Ford for graphs with negative weights (time complexity: O(|V|*|E|)), 
     * or Dijkstra's for non-negative weighted graphs (time complexity: O(|V|*|V|)), and thus does not waste resources.
     *
     * @param graph The graph.
     * @param start The start vertex.
     * @param end The end vertex.
     * @return A string representing the path or a message if the path doesn't exist.
     */
    string Algorithms::shortestPath(Graph& graph, size_t start, size_t end) 
    {
        size_t numVertices = graph.getNumVertices();

        // Error checking no. 1
        if (start >= numVertices || end >= numVertices) 
        {
            return "Invalid start or end vertex";
        }

        // Error checking no. 2
        if (start == end) 
        {
            return "No path exists between a vertex and itself";
        }

        // Check if the graph is unweighted
        bool isUnweighted = isGraphUnweighted(graph);

        // Choose the algorithm based on the graph type
        if (isUnweighted) 
        {
            return bfsShortestPath(graph, start, end);
        } 
        else 
        {
            bool hasNegativeEdges = hasNegativeEdgesInGraph(graph);     // Check if the graph has negative edges to choose the relevant algorithm

            if (hasNegativeEdges) 
            {
                return bellmanFordShortestPath(graph, start, end);
            } 
            else 
            {
                return dijkstraShortestPath(graph, start, end);        
            }
        }
    }

     /**
     * @brief This method checks if there is a cycle in the graph.
     *
     * This method uses DFS to find a back edge, which indicates a cycle.
     *
     * @param graph The graph to check.
     * @return A string representing the cycle if one exists; otherwise, "0".
     */
    string Algorithms::isContainsCycle(Graph& graph) 
    {
        size_t numVertices = graph.getNumVertices();
        vector<bool> visited(numVertices, false);
        vector<size_t> parent(numVertices, INT_MAX);

        for (size_t v = 0; v < numVertices; v++) 
        {
            if (!visited[v]) 
            {
                string cycle = dfs_cycle(graph, v, visited, parent, v);
                if (!cycle.empty()) 
                {
                    return cycle;
                }
            }
        }

        return "0";
    }
    

    /**
     * @brief This function checks if a graph is bipartite.
     *
     * @note A graph is bipartite if it can be colored using two colors such that no two adjacent vertices with the same color.
     * This function uses DFS to color the graph.
     *
     * @param graph The graph to check.
     * @return A string indicating if the graph is bipartite; otherwise, an error message.
     */
    string Algorithms::isBipartite(Graph& graph) 
    {
        size_t numVertices = graph.getNumVertices();
        vector<int> color(numVertices, -1);             // Initiate a vector with a length of the number of vertices and uncolor (-1) all of them

        for (size_t i = 0; i < numVertices; ++i) 
        {
            // If a vertex is uncolored, start a new DFS
            if (color[i] == -1) 
            {  
                if (!dfsCheck(graph, i, color, 0)) 
                {
                    return "The graph is not bipartite";
                }
            }
        }

        // Saperate the vertices based on their colors to two sets
        vector<size_t> setA;
        vector<size_t> setB;

        for (size_t i = 0; i < numVertices; i++) 
        {
            if (color[i] == 0) 
            {
                setA.push_back(i);
            }
            else
            {
                setB.push_back(i);
            }
        }
        
        // Present the results as string
        string result = "The graph is bipartite: A=" + buildSet(setA) + ", B=" + buildSet(setB);
        return result;
    }


    /**
     * @brief This method checks for a negative cycle in the graph.
     *
     * This method checks for negative cycles using a variant of the Bellman-Ford algorithm, which is capable of detecting such cycles.
     *
     * @param graph The graph to check.
     * @return A string represents a negative cycle if exists; otherwise, "No negative cycle exists".
     */
    string Algorithms::negativeCycle(Graph& graph)
    {
        size_t numVertices = graph.getNumVertices();        // A variable to store the number of vertices in t he graph   
        vector<int> distance(numVertices, 0);               // Initialize distance vector with zeros
        vector<size_t> parent(numVertices, INT_MAX);        // Initialize parent vector with INT_MAX. This vector will use us to reconstruct paths and detect cycles
        bool hasNegativeCycle = false;                      // A flag to detect if there is a negative cycle

        // Part 1: Check for negative self-loops
        for (size_t u = 0; u < numVertices; u++) 
        {
            if (graph.getAdjacencyMatrix()[u][u] < 0) 
            {
                hasNegativeCycle = true;
                return to_string(u) + "->" + to_string(u);
            }
        }

        // Part 2: Check for negative cycles between neighboring vertices with different edge weights
        for (size_t u = 0; u < numVertices; u++) 
        {
            for (size_t v = 0; v < numVertices; v++) 
            {
                // Check if there are differing weights in the edges u->v and v->u that sum to a negative value
                if (u != v && graph.getAdjacencyMatrix()[u][v] != 0 && graph.getAdjacencyMatrix()[v][u] != 0 &&
                    graph.getAdjacencyMatrix()[u][v] != graph.getAdjacencyMatrix()[v][u]) 
                    {
                    int weight = graph.getAdjacencyMatrix()[u][v] + graph.getAdjacencyMatrix()[v][u];
                    if (weight < 0) 
                    {
                        hasNegativeCycle = true;
                        return to_string(u) + "->" + to_string(v) + "->" + to_string(u);
                    }
                }
            }
        }

        // Part 3: Check for negative cycles with with path involves more than one edge
        string result = detectNegativeCycle(graph);
        if (result != "No negative cycle exists") 
        {
            return result;
        }

        return "No negative cycle exists";
    }



    /*********************************************/
    ///             PRIVATE SECTION             ///
    /*********************************************/

    /**
     * @brief This auxiliary function preforms BFS on the graph starting from the given vertex.
     *
     * @param graph The graph to perform BFS on.
     * @param startVertex The starting vertex for BFS.
     * @param visited A vector to track visited vertices.
     */
    void Algorithms::bfs(Graph& graph, size_t startVertex, vector<bool>& visited) {
        queue<size_t> queue;
        visited[startVertex] = true;            // Mark the startVertex as visited
        queue.push(startVertex);                // Add the startVertex to the queue 


        // According to the BFS algorithm we will continue the process until there are no more vertices to explore
        while (!queue.empty()) 
        {
            size_t currentVertex = queue.front();                         // Store the front vertex and remove it from the queue
            queue.pop();

            for (size_t i = 0; i < graph.getNumVertices(); i++)           // Iterate over all vertices in the graph to find all neighbors vertices
            {
                // Check if there is an edge from currentVertex to vertex i, and it is still "true" (not visited yet)
                if (graph.getAdjacencyMatrix()[currentVertex][i] != 0 && !visited[i]) 
                {
                    visited[i] = true;
                    queue.push(i);
                }
            }
        }
    }


    /**
     * @brief This auxiliary function finds the shortest path using BFS for unweighted graphs.
     *
     * This function returns the shortest path by building the path itself.
     *
     * @param graph The graph.
     * @param start The start vertex.
     * @param end The end vertex.
     * @return A string representing the shortest path.
     */
    string Algorithms::bfsShortestPath(Graph& graph, size_t start, size_t end) 
    {
        size_t numVertices = graph.getNumVertices();
        vector<bool> visited(numVertices, false);
        vector<size_t> parent(numVertices, INT_MAX);

        queue<size_t> queue;
        visited[start] = true;
        queue.push(start);

        while (!queue.empty()) 
        {
            size_t currentVertex = queue.front();
            queue.pop();

            if (currentVertex == end) 
            {
                break;
            }

            for (size_t i = 0; i < numVertices; i++) 
            {
                if (graph.getAdjacencyMatrix()[currentVertex][i] != 0 && !visited[i]) 
                {
                    visited[i] = true;
                    queue.push(i);
                    parent[i] = currentVertex;
                }
            }
        }

        if (!visited[end]) 
        {
            return "No path exists between " + to_string(start) + " and " + to_string(end);
        }

        // Build the shortest path
        string path = reconstructPath(start, end, parent);

        return path;
    }



    /**
     * @brief This auxiliary function finds the shortest path in a graph with possible negative weights using Bellman-Ford algorithm.
     *
     * This algorithm also checks for negative cycles and if detected, returns an error message.
     *
     * @param graph The graph.
     * @param start The starting vertex.
     * @param end The destination vertex.
     * @return A string representing the path or an error message if a negative cycle is detected.
     */
    string Algorithms::bellmanFordShortestPath(Graph& graph, size_t start, size_t end) 
    {
        size_t numVertices = graph.getNumVertices();
        vector<int> distance(numVertices, INT_MAX);
        vector<size_t> parent(numVertices, INT_MAX);

        distance[start] = 0;

        // Relax edges V-1 times
        for (int i = 0; i < numVertices - 1; i++) 
        {
            for (size_t u = 0; u < numVertices; u++) 
            {
                for (size_t v = 0; v < numVertices; v++) 
                {
                    int weight = graph.getAdjacencyMatrix()[u][v];
                    
                    // Check if there is an edge u->v and if the current path through u is shorter
                    if (weight != 0 && distance[u] != INT_MAX && distance[u] + weight < distance[v]) 
                    {
                        // Checl if the graph is directed or undirected when v is not the parent of u
                        if (graph.isGraphDirected() || (!graph.isGraphDirected() && parent[u] != v)) 
                        {
                            distance[v] = distance[u] + weight;     // Update the distance to vertex v through u
                            parent[v] = u;                          // Record u as the parent of v
                        }
                    }
                }
            }
        }

        // Check for negative cycle on the V-th iteration
        for (size_t u = 0; u < numVertices; u++) 
        {
            for (size_t v = 0; v < numVertices; v++) 
            {
                int weight = graph.getAdjacencyMatrix()[u][v];
                
                // If an additional relaxation is possible, a negative cycle exists
                if (weight != 0 && distance[u] != INT_MAX && distance[u] + weight < distance[v]) 
                {
                    if (graph.isGraphDirected() || (!graph.isGraphDirected() && parent[u] != v)) 
                    {
                        return "Graph contains a negative cycle";
                    }
                }
            }
        }


        // Check if the distance from start to end remains infinity
        if (distance[end] == INT_MAX) 
        {
            return "No path exists between " + to_string(start) + " and " + to_string(end);
        }

        // Build the shortest path
        string path = reconstructPath(start, end, parent);

        return path;
    }


    string Algorithms::dijkstraShortestPath(Graph& graph, size_t start, size_t end) {
        size_t numVertices = graph.getNumVertices();
        vector<int> distance(numVertices, INT_MAX);
        vector<size_t> parent(numVertices, INT_MAX);
        vector<bool> visited(numVertices, false);

        distance[start] = 0;

        for (size_t i = 0; i < numVertices; i++) {
            size_t minVertex = findMinDistanceVertex(distance, visited);

            if (minVertex == INT_MAX) {
                break;
            }

            visited[minVertex] = true;

            for (size_t v = 0; v < numVertices; v++) {
                int weight = graph.getAdjacencyMatrix()[minVertex][v];
                if (weight != 0 && !visited[v] && distance[minVertex] != INT_MAX &&
                    distance[minVertex] + weight < distance[v]) {
                    distance[v] = distance[minVertex] + weight;
                    parent[v] = minVertex;
                }
            }
        }

        if (distance[end] == INT_MAX) {
            return "No path exists between " + to_string(start) + " and " + to_string(end);
        }

        // Reconstruct the shortest path
        string path = reconstructPath(start, end, parent);

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

    bool Algorithms::dfsCheck(Graph& graph, size_t u, vector<int>& color, int col) {
        if (color[u] != -1) {
            // If already colored, check for color conflict
            return color[u] == col;
        }

        // Color the current vertex
        color[u] = col;

        // Check all adjacent vertices
        for (size_t v = 0; v < graph.getNumVertices(); ++v) {
            if (graph.getAdjacencyMatrix()[u][v] != 0) {  // There's an edge from u to v
                // Try to color the adjacent vertex with the opposite color
                if (!dfsCheck(graph, v, color, 1 - col)) {
                    return false;
                }
            }
            if (graph.getAdjacencyMatrix()[v][u] != 0) {  // Check reverse direction for undirected graphs
                if (!dfsCheck(graph, v, color, 1 - col)) {
                    return false;
                }
            }
        }
        return true;
    }


    bool Algorithms::isGraphUnweighted(Graph& graph) {
        size_t numVertices = graph.getNumVertices();
        for (size_t i = 0; i < numVertices; i++) {
            for (size_t j = 0; j < numVertices; j++) {
                if (graph.getAdjacencyMatrix()[i][j] != 0 && graph.getAdjacencyMatrix()[i][j] != 1) {
                    return false;
                }
            }
        }
        return true;
    }

    bool Algorithms::hasNegativeEdgesInGraph(Graph& graph) {
        size_t numVertices = graph.getNumVertices();
        for (size_t i = 0; i < numVertices; i++) {
            for (size_t j = 0; j < numVertices; j++) {
                if (graph.getAdjacencyMatrix()[i][j] < 0) {
                    return true;
                }
            }
        }
        return false;
    }
    

    string Algorithms::reconstructPath(size_t start, size_t end, vector<size_t>& parent) {
        string path;
        size_t current = end;

        // If no path exists from start to end, return an empty string
        if (parent[current] == INT_MAX) {
            return "";
        }

        // Traverse backwards from end to start using the parent vector
        while (current != start) {
            path = "->" + to_string(current) + path;
            current = parent[current];
        }

        // Add the start vertex to the path
        path = to_string(start) + path;

        return path;
    }

    size_t Algorithms::findMinDistanceVertex(vector<int>& distance, vector<bool>& visited) {
        size_t minVertex = INT_MAX;
        int minDistance = INT_MAX;

        for (size_t v = 0; v < distance.size(); v++) {
            if (!visited[v] && distance[v] < minDistance) {
                minVertex = v;
                minDistance = distance[v];
            }
        }

        return minVertex;
    }


    string Algorithms::buildSet(const vector<size_t>& set) 
    {
        string result = "{";
        for (size_t i = 0; i < set.size(); i++) 
        {
            result += to_string(set[i]);
            if (i < set.size() - 1)             // We use this condition to determine whether to add "," to the string
            {
                result += ",";
            }
        }
        result += "}";
        return result;
    }


    string Algorithms::detectNegativeCycle(Graph& graph) {
        size_t numVertices = graph.getNumVertices();
        vector<int> distance(numVertices, 0);
        vector<size_t> parent(numVertices, INT_MAX);
        bool hasNegativeCycle = false;

        for (size_t start = 0; start < numVertices; ++start) {
            distance.assign(numVertices, INT_MAX);
            parent.assign(numVertices, INT_MAX);
            distance[start] = 0;

            for (int i = 0; i < numVertices - 1; i++) {
                for (size_t u = 0; u < numVertices; u++) {
                    for (size_t v = 0; v < numVertices; v++) {
                        int weight = graph.getAdjacencyMatrix()[u][v];
                        if (weight != 0 && distance[u] != INT_MAX && distance[u] + weight < distance[v]) {
                            if (graph.isGraphDirected() || (!graph.isGraphDirected() && parent[u] != v)) {
                                distance[v] = distance[u] + weight;
                                parent[v] = u;
                            }
                        }
                    }
                }
            }

            for (size_t u = 0; u < numVertices; u++) {
                for (size_t v = 0; v < numVertices; v++) {
                    int weight = graph.getAdjacencyMatrix()[u][v];
                    if (weight != 0 && distance[u] != INT_MAX && distance[u] + weight < distance[v]) {
                        if (graph.isGraphDirected() || (!graph.isGraphDirected() && parent[u] != v)) {
                            hasNegativeCycle = true;
                            size_t current = v;
                            vector<size_t> cycle;

                            while (current != INT_MAX) {
                                cycle.push_back(current);
                                current = parent[current];
                                if (find(cycle.begin(), cycle.end(), current) != cycle.end()) {
                                    cycle.push_back(current);
                                    break;
                                }
                            }

                            stringstream cycleString;
                            cycleString << cycle.back();
                            for (size_t i = cycle.size() - 2; i != static_cast<size_t>(-1); i--) {
                                cycleString << "->" << cycle[i];
                            }

                            return cycleString.str();
                        }
                    }
                }

                if (hasNegativeCycle) {
                    break;
                }
            }
        }

        return "No negative cycle exists";
    }

}