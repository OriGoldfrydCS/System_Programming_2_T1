// ID: 200661775
// Email: origoldbsc@gmail.com

#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include <string>
#include "Graph.hpp"

using namespace std;

namespace ariel {

class Algorithms {

    public:
        static bool isConnected(Graph& graph);
        static bool isStronglyConnected(Graph& graph);
        static string shortestPath(Graph& graph, size_t start, size_t end);
        static string isContainsCycle(Graph& graph);
        static string isBipartite(Graph& graph);
        static string negativeCycle(Graph& graph);

    private:
        static void bfs(Graph& graph, size_t startVertex, vector<bool>& visited);

        static string bfsShortestPath(Graph& graph, size_t start, size_t end);
        static string bellmanFordShortestPath(Graph& graph, size_t start, size_t end);
        static void relaxEdges(Graph& graph, vector<int>& distance, vector<size_t>& parent);
        static bool canRelax(Graph& graph, size_t vertex_u, size_t vertex_v, int weight, vector<int>& distance, vector<size_t>& parent);
        static bool hasNegativeCycle(Graph& graph, vector<int>& distance, vector<size_t>& parent);


        static string dijkstraShortestPath(Graph& graph, size_t start, size_t end);
        
        static string dfs_cycle(Graph& graph, size_t vertex, vector<bool>& visited, vector<size_t>& parent, size_t start);
        
        static bool dfsCheck(Graph& graph, size_t currectVertex, vector<int>& colorVec, int color);
        static string buildSet(vector<size_t>& set);

        static bool isGraphUnweighted(Graph& graph);
        static bool hasNegativeEdgesInGraph(Graph& graph);
        static string buildPath(size_t start, size_t end, vector<size_t>& parent);
        static size_t findMinDistanceVertex(vector<int>& distance, vector<bool>& visited);

        static string detectNegativeCycle(Graph& graph);

    };

}

#endif