#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include <string>
#include "Graph.hpp"

using namespace std;

namespace ariel {

class Algorithms {

    public:
        static bool isConnected(Graph& g);
        static bool isStronglyConnected(Graph& graph);
        static string shortestPath(Graph& graph, size_t start, size_t end);
        static string isContainsCycle(Graph& graph);
        static string isBipartite(Graph& graph);
        static string negativeCycle(Graph& graph);

    private:
        static void bfs(Graph& graph, size_t startVertex, vector<bool>& visited);

        static string bfsShortestPath(Graph& graph, size_t start, size_t end);
        static string bellmanFordShortestPath(Graph& graph, size_t start, size_t end);
        static string dijkstraShortestPath(Graph& graph, size_t start, size_t end);
        
        static string dfs_cycle(Graph& graph, size_t v, vector<bool>& visited, vector<size_t>& parent, size_t start);
        
        static bool dfsCheck(Graph& graph, size_t u, vector<int>& color, int col);
        static string buildSet(const vector<size_t>& set);

        static bool isGraphUnweighted(Graph& graph);
        static bool hasNegativeEdgesInGraph(Graph& graph);
        static string reconstructPath(size_t start, size_t end, vector<size_t>& parent);
        static size_t findMinDistanceVertex(vector<int>& distance, vector<bool>& visited);
        static string detectNegativeCycle(Graph& graph);
    };

}

#endif