// ID: 200661775
// Email: origoldbsc@gmail.com

#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include "Graph.hpp"
#include <string>

using namespace std;

namespace ariel {

class Algorithms {

    public:

        /**
         * @brief This method checks if all vertices in a given graph are reachable from the vertex with index 0 by using BFS algorithm.
         *
         * @param graph The graph to check.
         * @return true if all vertices are reachable; otherwise, false.
         */
        static bool isConnected(Graph& graph);


        /**
         * This function checks if a given graph is strongly connected by applying BFS from each vertex.
         *
         * @param graph The graph to check.
         * @return true if the graph is strongly connected; otherwise, false.
         */
        static bool isStronglyConnected(Graph& graph);


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
        static string shortestPath(Graph& graph, size_t start, size_t end);
        
        
        /**
         * @brief This method checks if there is a cycle in the graph.
         *
         * This method uses DFS to find a back edge, which indicates a cycle.
         *
         * @param graph The graph to check.
         * @return A string representing the cycle if one exists; otherwise, "0".
         */
        static string isContainsCycle(Graph& graph);


        /**
         * @brief This function checks if a graph is bipartite.
         *
         * @note A graph is bipartite if it can be colored using two colors such that no two adjacent vertices with the same color.
         * This function uses DFS to color the graph.
         *
         * @param graph The graph to check.
         * @return A string indicating if the graph is bipartite; otherwise, an error message.
         */
        static string isBipartite(Graph& graph);


        /**
         * @brief This method checks for a negative cycle in the graph.
         *
         * This method checks for negative cycles using a variant of the Bellman-Ford algorithm, which is capable of detecting such cycles.
         *
         * @param graph The graph to check.
         * @return A string represents a negative cycle if exists; otherwise, "No negative cycle exists".
         */
        static string negativeCycle(Graph& graph);

    private:

        /**
         * @brief This auxiliary function performs BFS on the graph starting from the given vertex.
         * It can also be used to find the shortest path by tracking the parent of each vertex.
         *
         * @param graph The graph to perform BFS on.
         * @param startVertex The starting vertex for BFS.
         * @param visited A vector to track visited vertices.
         * @param parent A vector to track the parent of the vertices (extents the classic BFS for finding the shortest path).
         * @param parent A vector to track the parent of each vertex, used for rebuilding paths (extents the classic BFS for finding the shortest path).
         */
        static void bfs(Graph& graph, size_t startVertex, vector<bool>& visited, vector<size_t>& parent, size_t end);


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
        static string bfsShortestPath(Graph& graph, size_t start, size_t end);


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
        static string bellmanFordShortestPath(Graph& graph, size_t start, size_t end);


        /**
         * @brief This auxiliary function relaxes the edges in the graph during the shortest path process.
         *
         * @param graph The graph on which the edges are relaxed.
         * @param distance The vector storing the distances from the source vertex to each vertex.
         * @param parent The vector storing the parent of each vertex in the shortest path tree.
         */
        static bool relaxEdges(Graph& graph, vector<int>& distance, vector<size_t>& parent);


        /**
         * @brief This auxiliary function checks if an edge can be relaxed.
         *
         * @param graph The graph containing the edge.
         * @param vertex_u The source vertex of the edge.
         * @param vertex_v The destination vertex of the edge.
         * @param weight The weight of the edge.
         * @param distance The vector storing the distances from the source vertex to each vertex.
         * @param parent The vector storing the parent of each vertex in the shortest path tree.
         * @return true if the edge can be relaxed, false otherwise.
         */
        static bool canRelax(Graph& graph, size_t vertex_u, size_t vertex_v, int weight, vector<int>& distance, vector<size_t>& parent);


        /**
         * @brief This auxiliary function finds the shortest path between two vertices using Dijkstra's algorithm.
         * 
         * Dijkstra's algorithm is used for finding the shortest path from a single source vertex
         * to a single destination vertex in a graph with non-negative edge weights.
         *
         * @param graph The graph.
         * @param start The index of the start vertex.
         * @param end The index of the end vertex.
         * @return A string representing the shortest path or a message indicating no path exists.
         */
        static string dijkstraShortestPath(Graph& graph, size_t start, size_t end);
        

        /**
         * @brief This auxiliary function finds the vertex with the minimum distance that has not been visited yet.
         * 
         * This is a function used by Dijkstra's algorithm to select the next vertex to visit.
         *
         * @param distance A vector of distances from the start vertex.
         * @param visited A vector indicating whether each vertex has been visited.
         * @return The index of the vertex with the smallest distance that has not been visited.
         */
        static size_t findMinDistanceVertex(vector<int>& distance, vector<bool>& visited);


        /**
         * @brief This auxiliary function builds the path from the start vertex to the end vertex.
         *
         * @param start The start vertex of the path.
         * @param end The end vertex of the path.
         * @param parent The vector containing each vertex's parent in the path.
         * @return A string representing the path or an empty string if no path exists.
         */
        static string buildPath(size_t start, size_t end, vector<size_t>& parent);

        
        /**
         * @brief This auxiliary function uses DFS to detect a cycle in the graph starting from a given vertex.
     
        * @param graph The graph to be checked.
        * @param vertex The current vertex that explored in the DFS.
        * @param visited A vector that tracks if each vertex already visited to avoid revisiting.
        * @param recStack A vector that tracks "active" vertices in the current recursion to find cycles.
        * @param parent A vector that stores the parent of each vertex, for building the circle (if found).
        * @param isDirected Boolean that give an undication if the graph is directed; that affects finding of back edges.
        * @return A string representing the cycle, or an empty string if no cycle is found.
        */
        static string dfs_cycle(Graph& graph, size_t vertex, vector<bool>& visited, vector<bool>& recStack, vector<size_t>& parent, bool isDirected);
            

        /**
         * @brief This auxiliary function checks if a graph is bipartite using DFS by trying to color it with 2-colors.
         * 
         * A graph is bipartite if it can be colored using two colors such that no two adjacent vertices
         * share the same color.
         *
         * @param graph The graph to check.
         * @param currectVertex The vertex to start coloring.
         * @param colorVec A vector storing the color of each vertex, initialized to -1 (uncolored).
         * @param color The current color to use (0 or 1).
         * @return true if the graph can be colored bipartitely; otherwise, false.
         */
        static bool dfsCheck(Graph& graph, size_t currectVertex, vector<int>& colorVec, int color);
        
        
        /**
         * @brief This auxiliary function builds a string representation of a set of vertices.
         *
         * @param set A vector containing the indices of vertices in the set.
         * @return A string representation of the set.
         */
        static string buildSet(vector<size_t>& set);


        /**
         * @brief This auxiliary function checks the graph type (weighted/unweighted, negative edges).
         *
         * @param graph The graph to be checked.
         * @return A pair of boolean values the give indication if the graph is unweighted and if it has negative edges.
         */
        static pair<bool, bool> checkGraphType(Graph& graph);


        /**
         * @brief This auxiliary function finds a negative cycle (if exists) in a graph using a Bellman-Ford algorithm.
         * 
         * This function attempts to detect a negative cycle by relaxing edges repeatedly and try to find an indication for a cycle.
         *
         * @param graph The graph in which to detect negative cycles.
         * @return A string describing the cycle if found, or a message indicating no cycle exists.
         */
        static string findNegativeCircle(Graph& graph);


        /**
         * @brief This auxiliary function finds a vertex in a negative cycle.
         *
         * @param graph The graph.
         * @param distance The vector storing the distances from the source vertex to each vertex.
         * @param parent The vector storing the parent of each vertex in the shortest path tree.
         * @return The vertex in the negative cycle.
         */
        static size_t findNegativeCycleVertex(Graph& graph, vector<int>& distance, vector<size_t>& parent);


        /**
         * @brief This auxiliary function builds a negative cycle found in the graph.
         *
         * @param graph The graph containing the negative cycle.
         * @param distance The vector storing the distances from the source vertex to each vertex.
         * @param parent The vector storing the parent of each vertex in the shortest path tree.
         * @return A string representing the negative cycle.
         */
        static string buildNegativeCycle(Graph& graph, size_t vertex_v, vector<size_t>& parent);
    };

}

#endif