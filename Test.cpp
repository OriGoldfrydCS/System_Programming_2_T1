#include "doctest.h"
#include "Algorithms.hpp"
#include "Graph.hpp"
#include <algorithm>  

using namespace std;
using namespace ariel;

bool isValidPath(const string& result, const initializer_list<string>& validPaths) {
    return find(validPaths.begin(), validPaths.end(), result) != validPaths.end();
}

Graph g;

TEST_CASE("Test isConnected: Empty graph") {
    vector<vector<int>> emptyGraph = {};
    g.loadGraph(emptyGraph);
    CHECK(Algorithms::isConnected(g) == true);
}

TEST_CASE("Test isConnected: Single vertex graph") {
    vector<vector<int>> singleVertexGraph = {{0}};
    g.loadGraph(singleVertexGraph);
    CHECK(Algorithms::isConnected(g) == true);
}

TEST_CASE("Test isConnected: Connected graph") {
    vector<vector<int>> connectedGraph = {
        {0, 1, 0, 0},
        {1, 0, 1, 1},
        {0, 1, 0, 0},
        {0, 1, 0, 0}};
    g.loadGraph(connectedGraph);
    CHECK(Algorithms::isConnected(g) == true);
}

TEST_CASE("Test isConnected: Disconnected graph") {
    vector<vector<int>> disconnectedGraph = {
        {0, 1, 0, 0},
        {1, 0, 0, 0},
        {0, 0, 0, 1},
        {0, 0, 1, 0}};
    g.loadGraph(disconnectedGraph);
    CHECK(Algorithms::isConnected(g) == false);
}

TEST_CASE("Test isConnected: Disconnected graph with multiple components") {
    vector<vector<int>> disconnectedMultipleComponents = {
        {0, 1, 0, 0, 0, 0},
        {1, 0, 0, 0, 0, 0},
        {0, 0, 0, 1, 0, 0},
        {0, 0, 1, 0, 0, 0},
        {0, 0, 0, 0, 0, 1},
        {0, 0, 0, 0, 1, 0}};
    g.loadGraph(disconnectedMultipleComponents);
    CHECK(Algorithms::isConnected(g) == false);
}

TEST_CASE("Test isConnected: Graph with self-loops") {
    vector<vector<int>> graphWithSelfLoops = {
        {1, 1, 0, 0},
        {1, 1, 1, 0},
        {0, 1, 0, 1},
        {0, 0, 1, 1}};
    g.loadGraph(graphWithSelfLoops);
    CHECK(Algorithms::isConnected(g) == true);
}

TEST_CASE("Test isConnected: Strongly connected graph") {
    vector<vector<int>> stronglyConnectedGraph = {
        {0, 1, 0, 1},
        {1, 0, 1, 0},
        {0, 1, 0, 1},
        {1, 0, 1, 0}};
    g.loadGraph(stronglyConnectedGraph);
    CHECK(Algorithms::isConnected(g) == true);
}

TEST_CASE("Test isConnected: Graph with a single edge") {
    vector<vector<int>> singleEdgeGraph = {
        {0, 1, 0},
        {0, 0, 0},
        {0, 0, 0}};
    g.loadGraph(singleEdgeGraph);
    CHECK(Algorithms::isConnected(g) == false);
}

TEST_CASE("Test isConnected: Complete graph") {
    vector<vector<int>> completeGraph = {
        {0, 1, 1, 1},
        {1, 0, 1, 1},
        {1, 1, 0, 1},
        {1, 1, 1, 0}};
    g.loadGraph(completeGraph);
    CHECK(Algorithms::isConnected(g) == true);
}

TEST_CASE("Test isConnected: Star graph") {
    vector<vector<int>> starGraph = {
        {0, 1, 1, 1, 1},
        {1, 0, 0, 0, 0},
        {1, 0, 0, 0, 0},
        {1, 0, 0, 0, 0},
        {1, 0, 0, 0, 0}};
    g.loadGraph(starGraph);
    CHECK(Algorithms::isConnected(g) == true);
}

TEST_CASE("Test isConnected: Graph with a single isolated vertex") {
    vector<vector<int>> singleIsolatedVertexGraph = {
        {0, 0, 0, 0},
        {0, 0, 1, 1},
        {0, 1, 0, 1},
        {0, 1, 1, 0}};
    g.loadGraph(singleIsolatedVertexGraph);
    CHECK(Algorithms::isConnected(g) == false);
}

TEST_CASE("Test isConnected: Graph with multiple isolated vertices") {
    vector<vector<int>> multipleIsolatedVerticesGraph = {
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 1, 1},
        {0, 0, 1, 0, 1},
        {0, 0, 1, 1, 0}};
    g.loadGraph(multipleIsolatedVerticesGraph);
    CHECK(Algorithms::isConnected(g) == false);
}

TEST_CASE("Test isConnected: Graph with all vertices isolated") {
    vector<vector<int>> allIsolatedVerticesGraph = {
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0}};
    g.loadGraph(allIsolatedVerticesGraph);
    CHECK(Algorithms::isConnected(g) == false);
}

TEST_CASE("Test isConnected: Graph with a vertex connected to itself") {
    vector<vector<int>> selfConnectedVertexGraph = {
        {1, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0}};
    g.loadGraph(selfConnectedVertexGraph);
    CHECK(Algorithms::isConnected(g) == false);
}

TEST_CASE("Test isConnected: Graph with all vertices connected to themselves") {
    vector<vector<int>> allSelfConnectedVerticesGraph = {
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}};
    g.loadGraph(allSelfConnectedVerticesGraph);
    CHECK(Algorithms::isConnected(g) == false);
}

TEST_CASE("Test shortestPath: Single vertex graph") {
    vector<vector<int>> singleVertexGraph = {{0}};
    g.loadGraph(singleVertexGraph);
    CHECK(Algorithms::shortestPath(g, 0, 0) == "0");
}

TEST_CASE("Test shortestPath: No path exists") {
    // Graph g;
    vector<vector<int>> noPathGraph = {
        {0, 1, 0, 0},
        {1, 0, 0, 0},
        {0, 0, 0, 1},
        {0, 0, 1, 0}};
    g.loadGraph(noPathGraph);
    CHECK(Algorithms::shortestPath(g, 0, 3) == "-1");
}

TEST_CASE("Test shortestPath: Multiple paths exist") {

    vector<vector<int>> multiplePathsGraph = {
        {0, 1, 1, 0},
        {1, 0, 1, 1},
        {1, 1, 0, 1},
        {0, 1, 1, 0}};
    g.loadGraph(multiplePathsGraph);
    string result = Algorithms::shortestPath(g, 0, 3);
    CHECK(isValidPath(result, {"0->1->3", "0->2->3"}));
}

TEST_CASE("Test shortestPath: Shortest path with weighted edges") {

    vector<vector<int>> weightedGraph = {
        {0, 2, 0, 6, 0},
        {2, 0, 3, 8, 5},
        {0, 3, 0, 0, 7},
        {6, 8, 0, 0, 9},
        {0, 5, 7, 9, 0}};
    g.loadGraph(weightedGraph);
    CHECK(Algorithms::shortestPath(g, 0, 3) == "0->3");
}

TEST_CASE("Test shortestPath: Graph with a longer path") {
    vector<vector<int>> longerPathGraph = {
        {0, 1, 0, 0, 0},
        {1, 0, 1, 0, 0},
        {0, 1, 0, 1, 0},
        {0, 0, 1, 0, 1},
        {0, 0, 0, 1, 0}};
    g.loadGraph(longerPathGraph);
    CHECK(Algorithms::shortestPath(g, 0, 4) == "0->1->2->3->4");
}

TEST_CASE("Test isContainsCycle: Empty graph") {
    // ariel::Graph g;
    vector<vector<int>> emptyGraph = {};
    g.loadGraph(emptyGraph);
    CHECK(Algorithms::isContainsCycle(g) == false);
}

TEST_CASE("Test isContainsCycle: Single vertex graph") {

    vector<vector<int>> singleVertexGraph = {{0}};
    g.loadGraph(singleVertexGraph);
    CHECK(Algorithms::isContainsCycle(g) == false);
}

TEST_CASE("Test isContainsCycle: Graph with no cycle") {

    vector<vector<int>> noCycleGraph = {
        {0, 1, 0},
        {1, 0, 1},
        {0, 1, 0}};
    g.loadGraph(noCycleGraph);
    CHECK(Algorithms::isContainsCycle(g) == false);
}

TEST_CASE("Test isContainsCycle: Undirected graph with a cycle") {

    vector<vector<int>> cycleGraph = {
        {0, 1, 1, 0, 0},
        {1, 0, 1, 0, 0},
        {1, 1, 0, 1, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0}};
    g.loadGraph(cycleGraph);
    CHECK(Algorithms::isContainsCycle(g) == true);
}

TEST_CASE("Test isContainsCycle: Directed graph with a cycle") {
    vector<vector<int>> directedCycleGraph = {
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1},
        {1, 0, 0, 0}};
    g.loadGraph(directedCycleGraph);
    CHECK(Algorithms::isContainsCycle(g) == true);
}

TEST_CASE("Test isContainsCycle: Graph with an odd cycle (not bipartite)") {
    vector<vector<int>> oddCycleGraph = {
        {0, 1, 0, 0, 1},
        {1, 0, 1, 0, 0},
        {0, 1, 0, 1, 0},
        {0, 0, 1, 0, 1},
        {1, 0, 0, 1, 0}};
    g.loadGraph(oddCycleGraph);
    CHECK(Algorithms::isBipartite(g) == "0");
}

TEST_CASE("Test isBipartite: Empty graph") {
    vector<vector<int>> emptyGraph = {};
    g.loadGraph(emptyGraph);
    CHECK(Algorithms::isBipartite(g) == "The graph is bipartite: A={}, B={}");
}

TEST_CASE("Test isBipartite: Single vertex graph") {
    vector<vector<int>> singleVertexGraph = {{0}};
    g.loadGraph(singleVertexGraph);
    CHECK(Algorithms::isBipartite(g) == "The graph is bipartite: A={0}, B={}");
}

TEST_CASE("Test isBipartite: Bipartite graph") {
    vector<vector<int>> bipartiteGraph = {
        {0, 1, 0},
        {1, 0, 1},
        {0, 1, 0}};
    g.loadGraph(bipartiteGraph);
    CHECK(Algorithms::isBipartite(g) == "The graph is bipartite: A={0, 2}, B={1}");
}

TEST_CASE("Test isBipartite: Non-bipartite graph") {
    vector<vector<int>> nonBipartiteGraph = {
        {0, 1, 1, 0, 0},
        {1, 0, 1, 0, 0},
        {1, 1, 0, 1, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0}};
    g.loadGraph(nonBipartiteGraph);
    CHECK(Algorithms::isBipartite(g) == "0");
}

TEST_CASE("Test isBipartite: Bipartite graph with connected components") {
    vector<vector<int>> bipartiteConnectedComponentsGraph = {
        {0, 1, 0, 0, 0, 0, 0},
        {1, 0, 1, 0, 0, 0, 0},
        {0, 1, 0, 1, 0, 0, 0},
        {0, 0, 1, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 1, 0, 1},
        {0, 0, 0, 0, 0, 1, 0}};
    g.loadGraph(bipartiteConnectedComponentsGraph);

    string expectedOutput = "The graph is bipartite: A={0, 2, 4, 6}, B={1, 3, 5}";
    CHECK(Algorithms::isBipartite(g) == expectedOutput);
}

TEST_CASE("Test isBipartite: Bipartite graph with an even cycle") {
    vector<vector<int>> evenCycleBipartiteGraph = {
        {0, 1, 0, 0, 0, 1},
        {1, 0, 1, 0, 0, 0},
        {0, 1, 0, 1, 0, 0},
        {0, 0, 1, 0, 1, 0},
        {0, 0, 0, 1, 0, 1},
        {1, 0, 0, 0, 1, 0}};
    g.loadGraph(evenCycleBipartiteGraph);
    CHECK(Algorithms::isBipartite(g) == "The graph is bipartite: A={0, 2, 4}, B={1, 3, 5}");
}

TEST_CASE("Test negativeCycle: Empty graph") {
    vector<vector<int>> emptyGraph = {};
    g.loadGraph(emptyGraph);
    CHECK(Algorithms::negativeCycle(g) == false);
}

TEST_CASE("Test negativeCycle: Graph with positive weights") {
    vector<vector<int>> positiveWeightGraph = {
        {0, 2, 0, 6, 0},
        {2, 0, 3, 8, 5},
        {0, 3, 0, 0, 7},
        {6, 8, 0, 0, 9},
        {0, 5, 7, 9, 0}};
    g.loadGraph(positiveWeightGraph);
    CHECK(Algorithms::negativeCycle(g) == false);
}


TEST_CASE("Test negativeCycle: Graph with negative weights but no cycle") {
    vector<vector<int>> negativeWeightNoCycleGraph = {
        {0, -2, 0, 0},
        {0, 0, -3, 0},
        {0, 0, 0, -1},
        {0, 0, 0, 0}};
    g.loadGraph(negativeWeightNoCycleGraph);
    CHECK(Algorithms::negativeCycle(g) == false);
}

TEST_CASE("Test negativeCycle: Graph with negative weights and a positive cycle") {
    vector<vector<int>> negativeWeightPositiveCycleGraph = {
        {0, -2, 0, 0},
        {0, 0, 3, 0},
        {0, 0, 0, 4},
        {5, 0, 0, 0}};
    g.loadGraph(negativeWeightPositiveCycleGraph);
    CHECK(Algorithms::negativeCycle(g) == false);
}

TEST_CASE("Test negativeCycle: Graph with negative weights and a zero-weight cycle") {
    vector<vector<int>> negativeWeightZeroWeightCycleGraph = {
        {0, -2, 0, 0},
        {0, 0, -3, 0},
        {0, 3, 0, 0},
        {0, 0, 0, 0}};
    g.loadGraph(negativeWeightZeroWeightCycleGraph);
    CHECK(Algorithms::negativeCycle(g) == false);
}

TEST_CASE("Test negativeCycle: Graph with negative weights and disconnected components") {
    vector<vector<int>> negativeWeightDisconnectedGraph = {
        {0, -2, 0, 0, 0},
        {-2, 0, 0, 0, 0},
        {0, 0, 0, -3, 0},
        {0, 0, -3, 0, 0},
        {0, 0, 0, 0, 0}};
    g.loadGraph(negativeWeightDisconnectedGraph);
    CHECK(Algorithms::negativeCycle(g) == true);        // In this case, both components have valid negative cycles because we can travel from 
                                                        // one vertex to another and back (0->1->0 and 2->3->2), all involving negative weights.
}

TEST_CASE("Test negativeCycle: Graph with negative weights and a self-loop") {
    vector<vector<int>> negativeWeightSelfLoopGraph = {
        {-1, 0, 0, 0},
        {0, 0, -2, 0},
        {0, 0, 0, -3},
        {0, 0, 0, 0}};
    g.loadGraph(negativeWeightSelfLoopGraph);
    CHECK(Algorithms::negativeCycle(g) == true);
}

TEST_CASE("Test negativeCycle: Graph with negative cycle") {
    vector<vector<int>> negativeCycleGraph = {
        {0, -2, 0, 6, 0},
        {-2, 0, -3, 8, 5},
        {0, -3, 0, -4, 7},
        {6, 8, -4, 0, 9},
        {0, 5, 7, 9, 0}};
    g.loadGraph(negativeCycleGraph);
    CHECK(Algorithms::negativeCycle(g) == true);
}

TEST_CASE("Test negativeCycle: Graph with a negative weight cycle") {
    vector<vector<int>> negativeWeightCycleGraph = {
        {0, -1, 0, 0},
        {0, 0, -1, 0},
        {0, 0, 0, -1},
        {-1, 0, 0, 0}};
    g.loadGraph(negativeWeightCycleGraph);
    CHECK(Algorithms::negativeCycle(g) == true);
}

TEST_CASE("Test negativeCycle: Graph with negative weights and no negative cycle") {
    vector<vector<int>> negativeWeightsNoNegativeCycleGraph = {
        {0, -1, 0, 0, 0},
        {0, 0, -1, 0, 0},
        {0, 0, 0, -1, 0},
        {0, 0, 0, 0, -1},
        {0, 0, 0, 0, 0}};
    g.loadGraph(negativeWeightsNoNegativeCycleGraph);
    CHECK(Algorithms::negativeCycle(g) == false);
}

TEST_CASE("Test negativeCycle: Graph with a negative weight edge but no negative cycle") {
    vector<vector<int>> negativeWeightNoNegativeCycleGraph = {
        {0, -1, 0, 0},
        {0, 0, -1, 0},
        {0, 0, 0, -1},
        {0, 0, 0, 0}};
    g.loadGraph(negativeWeightNoNegativeCycleGraph);
    CHECK(Algorithms::negativeCycle(g) == false);
}

TEST_CASE("Test invalid graph: non-square matrix") {
    vector<vector<int>> invalidGraph = {
        {0, 1, 2, 0},
        {1, 0, 3, 0},
        {2, 3, 0, 4},
        {0, 0, 4, 0},
        {0, 0, 0, 5}};
    CHECK_THROWS(g.loadGraph(invalidGraph));
}