#include "doctest.h"
#include "Algorithms.hpp"
#include "Graph.hpp"
#include <algorithm>  
#include <iostream>

using namespace std;
using namespace ariel;

Graph g;

// TEST_CASE("Test isConnected: Empty graph") {
//     vector<vector<int>> emptyGraph = {};
//     g.loadGraph(emptyGraph);
//     CHECK(Algorithms::isConnected(g) == true);
// }

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
    CHECK(Algorithms::shortestPath(g, 0, 0) == "No path exists between a vertex and itself");
}

TEST_CASE("Test shortestPath: No path exists") {
    vector<vector<int>> noPathGraph = {
        {0, 1, 0, 0},
        {1, 0, 0, 0},
        {0, 0, 0, 1},
        {0, 0, 1, 0}};
    g.loadGraph(noPathGraph);
    CHECK(Algorithms::shortestPath(g, 0, 3) == "No path exists between 0 and 3");
}

TEST_CASE("Test shortestPath: Multiple paths exist") {

    vector<vector<int>> multiplePathsGraph = {
        {0, 1, 1, 0},
        {1, 0, 1, 1},
        {1, 1, 0, 1},
        {0, 1, 1, 0}};
    g.loadGraph(multiplePathsGraph);
    CHECK(((Algorithms::shortestPath(g, 0, 3) == "0->1->3" ) || (Algorithms::shortestPath(g, 0, 3) == "0->2->3")));
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


// TEST_CASE("Test isContainsCycle: Empty graph") {
//     vector<vector<int>> emptyGraph = {};
//     g.loadGraph(emptyGraph);
//     CHECK(Algorithms::isContainsCycle(g) == "0");
// }

TEST_CASE("Test isContainsCycle: Single vertex graph") {
    vector<vector<int>> singleVertexGraph = {{0}};
    g.loadGraph(singleVertexGraph);
    CHECK(Algorithms::isContainsCycle(g) == "0");
}

TEST_CASE("Test isContainsCycle: Graph with no cycle") {
    vector<vector<int>> noCycleGraph = {
        {0, 1, 0},
        {1, 0, 1},
        {0, 1, 0}};
    g.loadGraph(noCycleGraph);
    CHECK(Algorithms::isContainsCycle(g) == "0");
}

TEST_CASE("Test isContainsCycle: Undirected graph with a cycle") {
    vector<vector<int>> cycleGraph = {
        {0, 1, 1, 0, 0},
        {1, 0, 1, 0, 0},
        {1, 1, 0, 1, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0}};
    g.loadGraph(cycleGraph);
    CHECK(((Algorithms::isContainsCycle(g) == "0->1->2->0") || (Algorithms::isContainsCycle(g) == "2->0->1->2")));
}

TEST_CASE("Test isContainsCycle: Directed graph with a cycle") {
    vector<vector<int>> directedCycleGraph = {
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1},
        {1, 0, 0, 0}};
    g.loadGraph(directedCycleGraph);
    CHECK(((Algorithms::isContainsCycle(g) == "0->1->2->3->0") || (Algorithms::isContainsCycle(g) == " 3->0->1->2->3")));
}


TEST_CASE("isContainsCycle - Disconnected graph with cycles") {
    vector<vector<int>> graph1 = {
        {0, 1, 0, 0, 0},
        {1, 0, 1, 0, 0},
        {1, 1, 0, 1, 0},
        {0, 0, 1, 0, 1},
        {0, 0, 0, 1, 0}
    };
    g.loadGraph(graph1);
    CHECK(((ariel::Algorithms::isContainsCycle(g) == "0->1->2->0") || (ariel::Algorithms::isContainsCycle(g) == "2->0->1->2")));
}

TEST_CASE("isContainsCycle - Directed graph with multiple cycles") {

    vector<vector<int>> graph2 = {
        {0, 1, 0, 0, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 1, 0},
        {1, 0, 0, 0, 1},
        {0, 0, 0, 0, 0}
    };
    g.loadGraph(graph2);
    CHECK(((ariel::Algorithms::isContainsCycle(g) == "0->1->2->3->0") || (ariel::Algorithms::isContainsCycle(g) == "3->0->1->2->3")));
}

TEST_CASE("isContainsCycle - Undirected graph with self-loop") {
    vector<vector<int>> graph3 = {
        {1, 1, 0, 0, 0},
        {1, 0, 1, 0, 0},
        {0, 1, 0, 1, 1},
        {0, 0, 1, 0, 1},
        {0, 0, 1, 1, 0}
    };
    g.loadGraph(graph3);
    CHECK(ariel::Algorithms::isContainsCycle(g) == "0->0");
}

TEST_CASE("isContainsCycle - Directed graph with multiple connected components") {

    vector<vector<int>> graph4 = {
        {0, 1, 0, 0, 0, 0},
        {0, 0, 1, 0, 0, 0},
        {1, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0, 1},
        {0, 0, 0, 1, 0, 0}
    };
    g.loadGraph(graph4);
    CHECK(((ariel::Algorithms::isContainsCycle(g) == "0->1->2->0") || (ariel::Algorithms::isContainsCycle(g) == "3->4->5->3")));
}

TEST_CASE("isContainsCycle - Large graph with long cycle") {

    vector<vector<int>> graph5 = {
        {0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 1, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0}
    };
    g.loadGraph(graph5);
    CHECK(((ariel::Algorithms::isContainsCycle(g) == "0->1->2->3->4->5->6->7->8->9->0") || (ariel::Algorithms::isContainsCycle(g) == "9->0->1->2->3->4->5->6->7->8->9")));
}

TEST_CASE("isContainsCycle - Graph with no cycles") {
    vector<vector<int>> graph6 = {
        {0, 1, 0, 0, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 1, 0},
        {0, 0, 0, 0, 1},
        {0, 0, 0, 0, 0}
    };
    g.loadGraph(graph6);
    CHECK(ariel::Algorithms::isContainsCycle(g) == "0");
}

// TEST_CASE("Test isContainsCycle: Empty graph") {
//     vector<vector<int>> emptyGraph = {};
//     g.loadGraph(emptyGraph);
//     CHECK(Algorithms::isContainsCycle(g) == "0");
// }

TEST_CASE("Test isContainsCycle: Single vertex graph") {
    vector<vector<int>> singleVertexGraph = {{0}};
    g.loadGraph(singleVertexGraph);
    CHECK(Algorithms::isContainsCycle(g) == "0");
}

TEST_CASE("Test isContainsCycle: Graph with no cycle") {

    vector<vector<int>> noCycleGraph = {
        {0, 1, 0},
        {1, 0, 1},
        {0, 1, 0}};
    g.loadGraph(noCycleGraph);
    CHECK(Algorithms::isContainsCycle(g) == "0");
}


TEST_CASE("Test isContainsCycle: Graph with an odd cycle (not bipartite)") {
    vector<vector<int>> oddCycleGraph = {
        {0, 1, 0, 0, 1},
        {1, 0, 1, 0, 0},
        {0, 1, 0, 1, 0},
        {0, 0, 1, 0, 1},
        {1, 0, 0, 1, 0}};
    g.loadGraph(oddCycleGraph);
    CHECK(Algorithms::isBipartite(g) == "The graph is not bipartite");
}


// TEST_CASE("Test isBipartite: Empty graph") {
//     vector<vector<int>> emptyGraph = {};
//     g.loadGraph(emptyGraph);
//     CHECK(Algorithms::isBipartite(g) == "The graph is bipartite: A={}, B={}");
// }

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
    CHECK(Algorithms::isBipartite(g) == "The graph is bipartite: A={0,2}, B={1}");
}

TEST_CASE("Test isBipartite: Non-bipartite graph") {
    vector<vector<int>> nonBipartiteGraph = {
        {0, 1, 1, 0, 0},
        {1, 0, 1, 0, 0},
        {1, 1, 0, 1, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0}};
    g.loadGraph(nonBipartiteGraph);
    CHECK(Algorithms::isBipartite(g) == "The graph is not bipartite");
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

    CHECK(((Algorithms::isBipartite(g) == "The graph is bipartite: A={0,2,4,6}, B={1,3,5}")
    || (Algorithms::isBipartite(g) == "The graph is bipartite: A={0,2,5}, B={1,3,4,6}")));
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
    CHECK(Algorithms::isBipartite(g) == "The graph is bipartite: A={0,2,4}, B={1,3,5}");
}

// TEST_CASE("Test negativeCycle: Empty graph") {
//     vector<vector<int>> emptyGraph = {};
//     g.loadGraph(emptyGraph);
//     CHECK(Algorithms::negativeCycle(g) == "No negative cycle exists");
// }

TEST_CASE("Test negativeCycle: Graph with positive weights") {
    vector<vector<int>> positiveWeightGraph = {
        {0, 2, 0, 6, 0},
        {2, 0, 3, 8, 5},
        {0, 3, 0, 0, 7},
        {6, 8, 0, 0, 9},
        {0, 5, 7, 9, 0}};
    g.loadGraph(positiveWeightGraph);
    CHECK(Algorithms::negativeCycle(g) == "No negative cycle exists");
}


TEST_CASE("Test negativeCycle: Graph with negative weights but no cycle") {
    vector<vector<int>> negativeWeightNoCycleGraph = {
        {0, -2, 0, 0},
        {0, 0, -3, 0},
        {0, 0, 0, -1},
        {0, 0, 0, 0}};
    g.loadGraph(negativeWeightNoCycleGraph);
    CHECK(Algorithms::negativeCycle(g) == "No negative cycle exists");
}

TEST_CASE("Test negativeCycle: Graph with negative weights and a positive cycle") {
    vector<vector<int>> negativeWeightPositiveCycleGraph = {
        {0, -2, 0, 0},
        {0, 0, 3, 0},
        {0, 0, 0, 4},
        {5, 0, 0, 0}};
    g.loadGraph(negativeWeightPositiveCycleGraph);
    CHECK(Algorithms::negativeCycle(g) == "No negative cycle exists");
}

TEST_CASE("Test negativeCycle: Graph with negative weights and a zero-weight cycle") {
    vector<vector<int>> negativeWeightZeroWeightCycleGraph = {
        {0, -2, 0, 0},
        {0, 0, -3, 0},
        {0, 3, 0, 0},
        {0, 0, 0, 0}};
    g.loadGraph(negativeWeightZeroWeightCycleGraph);
    CHECK(Algorithms::negativeCycle(g) == "No negative cycle exists");
}


TEST_CASE("Test negativeCycle: Graph with negative weights and no negative cycle") {
    vector<vector<int>> negativeWeightsNoNegativeCycleGraph = {
        {0, -1, 0, 0, 0},
        {0, 0, -1, 0, 0},
        {0, 0, 0, -1, 0},
        {0, 0, 0, 0, -1},
        {0, 0, 0, 0, 0}};
    g.loadGraph(negativeWeightsNoNegativeCycleGraph);
    CHECK(Algorithms::negativeCycle(g) == "No negative cycle exists");
}
        
TEST_CASE("Test negativeCycle: Graph with a negative weight edge but no negative cycle") {
    vector<vector<int>> negativeWeightNoNegativeCycleGraph = {
        {0, -1, 0, 0},
        {0, 0, -1, 0},
        {0, 0, 0, -1},
        {0, 0, 0, 0}};
    g.loadGraph(negativeWeightNoNegativeCycleGraph);
    CHECK(Algorithms::negativeCycle(g) == "No negative cycle exists");
}



// TEST_CASE("Test negativeCycle: Empty graph") {
//     vector<vector<int>> emptyGraph = {};
//     g.loadGraph(emptyGraph);
//     CHECK(Algorithms::negativeCycle(g) == "No negative cycle exists");
// }

TEST_CASE("Test negativeCycle: Graph with positive weights") {
    vector<vector<int>> positiveWeightGraph = {
        {0, 2, 0, 6, 0},
        {2, 0, 3, 8, 5},
        {0, 3, 0, 0, 7},
        {6, 8, 0, 0, 9},
        {0, 5, 7, 9, 0}};
    g.loadGraph(positiveWeightGraph);
    CHECK(Algorithms::negativeCycle(g) == "No negative cycle exists");
}

TEST_CASE("Test negativeCycle: Graph with negative weights but no cycle") {
    vector<vector<int>> negativeWeightNoCycleGraph = {
        {0, -2, 0, 0},
        {0, 0, -3, 0},
        {0, 0, 0, -1},
        {0, 0, 0, 0}};
    g.loadGraph(negativeWeightNoCycleGraph);
    CHECK(Algorithms::negativeCycle(g) == "No negative cycle exists");
}

TEST_CASE("Test negativeCycle: Graph with negative weights and a positive cycle") {
    vector<vector<int>> negativeWeightPositiveCycleGraph = {
        {0, -2, 0, 0},
        {0, 0, 3, 0},
        {0, 0, 0, 4},
        {5, 0, 0, 0}};
    g.loadGraph(negativeWeightPositiveCycleGraph);
    CHECK(Algorithms::negativeCycle(g) == "No negative cycle exists");
}

TEST_CASE("Test negativeCycle: Graph with negative weights and a zero-weight cycle") {
    vector<vector<int>> negativeWeightZeroWeightCycleGraph = {
        {0, -2, 0, 0},
        {0, 0, -3, 0},
        {0, 3, 0, 0},
        {0, 0, 0, 0}};
    g.loadGraph(negativeWeightZeroWeightCycleGraph);
    CHECK(Algorithms::negativeCycle(g) == "No negative cycle exists");
}

TEST_CASE("Test negativeCycle: Graph with negative weights and disconnected components") {
    vector<vector<int>> negativeWeightDisconnectedGraph = {
        {0, -2, 0, 0, 0},
        {-2, 0, 0, 0, 0},
        {0, 0, 0, -3, 0},
        {0, 0, -3, 0, 0},
        {0, 0, 0, 0, 0}};
    g.loadGraph(negativeWeightDisconnectedGraph);
    CHECK(Algorithms::negativeCycle(g) == "No negative cycle exists");        
}

TEST_CASE("Test negativeCycle: Graph with negative weights and a self-loop") {
    vector<vector<int>> negativeWeightSelfLoopGraph = {
        {-1, 0, 0, 0},
        {0, 0, -2, 0},
        {0, 0, 0, -3},
        {0, 0, 0, 0}};
    g.loadGraph(negativeWeightSelfLoopGraph);
    CHECK(((Algorithms::negativeCycle(g) == "0->0") || (Algorithms::negativeCycle(g) == "1->1") ||
    (Algorithms::negativeCycle(g) == "2->2")));
}

TEST_CASE("Test negativeCycle: Graph with negative cycle") {
    vector<vector<int>> negativeCycleGraph = {
        {0, -2, 0, 6, 0},
        {-2, 0, -3, 8, 5},
        {0, -3, 0, -4, 7},
        {6, 8, -4, 0, 9},
        {0, 5, 7, 9, 0}};
    g.loadGraph(negativeCycleGraph);
    CHECK(((Algorithms::negativeCycle(g) == "0->1->2->3->0") || (Algorithms::negativeCycle(g) == "1->2->3->0->1")));
}

TEST_CASE("Test negativeCycle: Graph with a negative weight cycle") {
    vector<vector<int>> negativeWeightCycleGraph = {
        {0, -1, 0, 0},
        {0, 0, -1, 0},
        {0, 0, 0, -1},
        {-1, 0, 0, 0}};
    g.loadGraph(negativeWeightCycleGraph);
    CHECK(((Algorithms::negativeCycle(g) == "0->1->2->3->0") || (Algorithms::negativeCycle(g) == "1->2->3->0->1")));
}

TEST_CASE("Test negativeCycle: Graph with negative weights and no negative cycle") {
    vector<vector<int>> negativeWeightsNoNegativeCycleGraph = {
        {0, -1, 0, 0, 0},
        {0, 0, -1, 0, 0},
        {0, 0, 0, -1, 0},
        {0, 0, 0, 0, -1},
        {0, 0, 0, 0, 0}};
    g.loadGraph(negativeWeightsNoNegativeCycleGraph);
    CHECK(Algorithms::negativeCycle(g) == "No negative cycle exists");
}

TEST_CASE("Test negativeCycle: Graph with a negative weight edge but no negative cycle") {
    vector<vector<int>> negativeWeightNoNegativeCycleGraph = {
        {0, -1, 0, 0},
        {0, 0, -1, 0},
        {0, 0, 0, -1},
        {0, 0, 0, 0}};
    g.loadGraph(negativeWeightNoNegativeCycleGraph);
    CHECK(Algorithms::negativeCycle(g) == "No negative cycle exists");
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

TEST_CASE("Test invalid graph: non-square matrix") {
    vector<vector<int>> invalidGraph = {
        {0, 1, 2, 0, 1},
        {1, 0, 3, 0, 2},
        {2, 3, 0, 4, 4},
        {0, 0, 4, 0, 3}};
    CHECK_THROWS(g.loadGraph(invalidGraph));
}


TEST_CASE("Test Sp: UNDI"){
    vector<vector<int>> G = {
        {0, 2, 4, 0, 0, 0},
        {2, 0, 1, 7, 0, 0},
        {4, 1, 0, 0, 3, 0},
        {0, 7, 0, 0, 2, 1},
        {0, 0, 3, 2, 0, 5},
        {0, 0, 0, 1, 5, 0}};
    g.loadGraph(G);
    CHECK(ariel::Algorithms::shortestPath(g, 0, 5) == "0->1->2->4->3->5");
    CHECK(ariel::Algorithms::shortestPath(g, 1, 4) == "1->2->4");
    CHECK(ariel::Algorithms::shortestPath(g, 2, 3) == "2->4->3");
    CHECK(ariel::Algorithms::shortestPath(g, 5, 0) == "5->3->4->2->1->0");
}

TEST_CASE("Test Sp: UNDI11111111"){
    vector<vector<int>> G = {
        {0, -2, 4, 0, 0},
        {-2, 0, 3, 2, 2},
        {4, 3, 0, 5, 0},
        {0, 2, 5, 0, 0},
        {0, 2, 0, 0, 0}};
    g.loadGraph(G);
    CHECK(ariel::Algorithms::shortestPath(g, 0, 3) == "0->1->3");
    CHECK(ariel::Algorithms::shortestPath(g, 1, 4) == "1->4");
    CHECK(ariel::Algorithms::shortestPath(g, 2, 4) == "2->0->1->4");
    CHECK(ariel::Algorithms::shortestPath(g, 3, 0) == "3->1->0");
}

/////////////////////////////////////////////////////////////////////////

TEST_CASE("Test isContainsCycle: Undirected graph with cycle") {
    vector<vector<int>> G = {
        {0, 1, 0, 0},
        {1, 0, 1, 0},
        {0, 1, 0, 1},
        {0, 0, 1, 0}
    };
    g.loadGraph(G);
    CHECK(ariel::Algorithms::isContainsCycle(g) == "0");
}

TEST_CASE("Test isContainsCycle: Undirected graph without cycle") {
    vector<vector<int>> G = {
        {0, 1, 0, 0},
        {1, 0, 1, 0},
        {0, 1, 0, 0},
        {0, 0, 0, 0}
    };
    g.loadGraph(G);
    CHECK(ariel::Algorithms::isContainsCycle(g) == "0");
}

TEST_CASE("Test isContainsCycle: Directed graph with cycle") {
    vector<vector<int>> G = {
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1},
        {1, 0, 0, 0}
    };
    g.loadGraph(G);
    CHECK(ariel::Algorithms::isContainsCycle(g) == "0->1->2->3->0");
}

TEST_CASE("Test isContainsCycle: Directed graph without cycle") {
    vector<vector<int>> G = {
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1},
        {0, 0, 0, 0}
    };
    g.loadGraph(G);
    CHECK(ariel::Algorithms::isContainsCycle(g) == "0");
}

// TEST_CASE("Test isContainsCycle: Empty graph") {
//     vector<vector<int>> G = {};
//     g.loadGraph(G);
//     CHECK(ariel::Algorithms::isContainsCycle(g) == "0");
// }

TEST_CASE("Test isContainsCycle: Single vertex graph") {
    vector<vector<int>> G = {
        {0}
    };
    g.loadGraph(G);
    CHECK(ariel::Algorithms::isContainsCycle(g) == "0");
}

TEST_CASE("Test isContainsCycle: Disconnected graph with cycle") {
    vector<vector<int>> G = {
        {0, 1, 0, 0, 0},
        {1, 0, 0, 0, 0},
        {0, 0, 0, 1, 0},
        {0, 0, 1, 0, 1},
        {0, 0, 0, 1, 0}
    };
    g.loadGraph(G);
    CHECK(ariel::Algorithms::isContainsCycle(g) == "0");
}

TEST_CASE("Test isContainsCycle: Undirected weighted graph with cycle") {
    vector<vector<int>> G = {
        {0, 2, 0, 0},
        {2, 0, -3, 0},
        {0, -3, 0, 5},
        {0, 0, 5, 0}
    };
    g.loadGraph(G);
    CHECK(ariel::Algorithms::isContainsCycle(g) == "0");
}

TEST_CASE("Test isContainsCycle: Directed weighted graph with cycle") {
    vector<vector<int>> G = {
        {0, 2, 0, 0},
        {0, 0, -3, 0},
        {0, 0, 0, 5},
        {-1, 0, 0, 0}
    };
    g.loadGraph(G);
    CHECK(ariel::Algorithms::isContainsCycle(g) == "0->1->2->3->0");
}



///////////////////////////////////////////////////////////////////////
//Imry//////////////////////

TEST_CASE("Test isConnected")
{
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 0},
        {1, 0, 1},
        {0, 1, 0}};
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::isStronglyConnected(g) == true);

    vector<vector<int>> graph1 = {
        {0, -1},
        {1, 0}};
    g.loadGraph(graph1);
    CHECK(ariel::Algorithms::isStronglyConnected(g) == true);



    vector<vector<int>> graph4 = {
        {0, 1},
        {0, 0}}; // only connected. not strongly connected
    g.loadGraph(graph4);
    CHECK(ariel::Algorithms::isStronglyConnected(g) == false);


    vector<vector<int>> graph2 = {
        {0, 1, 1, 0, 0},
        {1, 0, 1, 0, 0},
        {1, 1, 0, 1, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0}};
    g.loadGraph(graph2);
    CHECK(ariel::Algorithms::isStronglyConnected(g) == false);

    vector<vector<int>> graph3 = {
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0}};
    g.loadGraph(graph3);
    CHECK(ariel::Algorithms::isStronglyConnected(g) == false);
}

TEST_CASE("Test shortestPath for BFS and BF")
{
    ariel::Graph g;
    vector<vector<int>> graph = {// BFS on undirected
                                 {0, 1, 0},
                                 {1, 0, 1},
                                 {0, 1, 0}};
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::shortestPath(g, 0, 2) == "0->1->2");

    vector<vector<int>> graph6 = {// BFS on direced
                                  {0, 1, 0},
                                  {0, 0, 1},
                                  {0, 1, 0}};
    g.loadGraph(graph6);
    CHECK(ariel::Algorithms::shortestPath(g, 0, 2) == "0->1->2");

    vector<vector<int>> graph2 = {
        {0, 1, 1, 0, 0},
        {1, 0, 1, 0, 0},
        {1, 1, 0, 1, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0}};
    g.loadGraph(graph2);
    CHECK(ariel::Algorithms::shortestPath(g, 0, 4) == "No path exists between 0 and 4");
    // CHECK(ariel::Algorithms::shortestPath(g, -1, 4) == "Invalid start or end vertex");   //////////////////////
    CHECK(ariel::Algorithms::shortestPath(g, 0, 8) == "Invalid start or end vertex");
    CHECK(ariel::Algorithms::shortestPath(g, 8, 0) == "Invalid start or end vertex");
    CHECK(ariel::Algorithms::shortestPath(g, 0, 0) == "No path exists between a vertex and itself");
}

TEST_CASE("Test XXX"){

    vector<vector<int>> graph3 = {           //////////////////////////////////
        {0, 8, -1},
        {8, 0, -5},
        {-1, -5, 0}};
    g.loadGraph(graph3);
    CHECK(ariel::Algorithms::shortestPath(g, 0, 2) == "0->2");
    CHECK(ariel::Algorithms::shortestPath(g, 0, 1) == "0->2->1");
    CHECK(ariel::Algorithms::shortestPath(g, 1, 2) == "1->2");
    CHECK(ariel::Algorithms::shortestPath(g, 1, 0) == "1->2->0");

    vector<vector<int>> graph4 = {
        {0, 3, -1},
        {3, 0, -5},
        {-1, -5, 0}};
    g.loadGraph(graph4);
    CHECK(ariel::Algorithms::shortestPath(g, 0, 2) == "Graph contains a negative cycle");

    vector<vector<int>> graph5 = {
        {0, 8, -2},
        {8, 0, -5},
        {-1, -5, 0}};
    g.loadGraph(graph5);
    CHECK(ariel::Algorithms::shortestPath(g, 0, 2) == "Graph contains a negative cycle");

    vector<vector<int>> graph7 = {               //////////////////////////////////////////////
        {0, 0, 0, 0},
        {4, 0, -6, 0},
        {0, 0, 0, 5},
        {0, -2, 0, 0}};
    g.loadGraph(graph7);
    CHECK(ariel::Algorithms::shortestPath(g, 0, 3) == "No path exists between 0 and 3");
}
TEST_CASE("Test shortest path Dijkstra")
{
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 2, 0},
        {1, 0, 1},
        {0, 1, 0}};
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::shortestPath(g, 0, 2) == "0->1->2");

    vector<vector<int>> graph2 = {
        {0, 6, 10, 0, 0, 0, 0, 0, 0, 0},
        {6, 0, 12, 11, 14, 0, 0, 0, 0, 0},
        {10, 12, 0, 12, 0, 0, 8, 16, 0, 0},
        {0, 11, 12, 0, 0, 6, 3, 0, 0, 0},
        {0, 14, 0, 0, 0, 4, 0, 0, 6, 0},
        {0, 0, 0, 6, 4, 0, 0, 0, 12, 0},
        {0, 0, 8, 3, 0, 0, 0, 0, 16, 6},
        {0, 0, 16, 0, 0, 0, 0, 0, 0, 8},
        {0, 0, 0, 0, 6, 12, 16, 0, 0, 13},
        {0, 0, 0, 0, 0, 0, 6, 8, 13, 0}};
    g.loadGraph(graph2);
    CHECK(ariel::Algorithms::shortestPath(g, 0, 9) == "0->2->6->9");
    CHECK(ariel::Algorithms::shortestPath(g, 0, 8) == "0->1->4->8");
    CHECK(ariel::Algorithms::shortestPath(g, 3, 7) == "3->6->9->7");
    CHECK(ariel::Algorithms::shortestPath(g, 7, 5) == "7->9->6->3->5");

    vector<vector<int>> graph3 = {
        {0, 7, 5, 0, 0, 0},
        {7, 0, 0, 11, 0, 0},
        {5, 0, 0, 0, 0, 0},
        {0, 11, 1, 0, 1, 0},
        {0, 0, 0, 1, 0, 1},
        {0, 0, 0, 5, 0, 0}};
    g.loadGraph(graph3);
    CHECK(ariel::Algorithms::shortestPath(g, 0, 5) == "0->1->3->4->5");
    CHECK(ariel::Algorithms::shortestPath(g, 5, 0) == "5->3->2->0");
}

TEST_CASE("Test isContainsCycle")
{
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 0},
        {1, 0, 1},
        {0, 1, 0}};
    g.loadGraph(graph);
    vector<vector<int>> graph10 = {
        {0, 2, 0},
        {1, 0, 1},
        {0, 1, 0}};
    g.loadGraph(graph10);
    CHECK(((ariel::Algorithms::isContainsCycle(g) == "0->1->0") || (ariel::Algorithms::isContainsCycle(g) == "1->0->1")));

    vector<vector<int>> graph2 = {
        {0, 1, 1, 0, 0},
        {1, 0, 1, 0, 0},
        {1, 1, 0, 1, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0}};
    g.loadGraph(graph2);

    CHECK(((ariel::Algorithms::isContainsCycle(g) == "0->1->2->0") || (ariel::Algorithms::isContainsCycle(g) == "2->0->1->2")));

    vector<vector<int>> graph4 = {
        {0, 8, -2},
        {8, 0, -5},
        {-1, -5, 0}};
    g.loadGraph(graph4);   
    CHECK(((ariel::Algorithms::negativeCycle(g) == "0->2->0") || (ariel::Algorithms::negativeCycle(g) == "2->0->2"))); // there is more than 1 cycle

    vector<vector<int>> graph5 = {
        {0, 8, -1},
        {8, 0, -4},
        {-1, -5, 0}};
    g.loadGraph(graph5);

    CHECK(((ariel::Algorithms::negativeCycle(g) == "1->2->1") || (ariel::Algorithms::negativeCycle(g) == "2->1->2"))); // there is more than 1 cycle

    vector<vector<int>> graph6 = {
        {0, 3, 7},
        {0, 0, 0},
        {7, 1, 0}

    };
    g.loadGraph(graph6);
    // CHECK(ariel::Algorithms::isContainsCycle(g) == "0->2->0");  /// not correct
    CHECK(ariel::Algorithms::isContainsCycle(g) == "0");

    graph6[1][2] = 6;
    g.loadGraph(graph6);
    CHECK(((ariel::Algorithms::isContainsCycle(g) == "2->0->1") || (ariel::Algorithms::isContainsCycle(g) == "0->1->2->0"))); // directed is by graph, or by edge?

    vector<vector<int>> graph7 = {
        {0, 0, 0, 0},
        {4, 0, -6, 0},
        {0, 0, 0, 5},
        {0, -2, 0, 0}};
    g.loadGraph(graph7);
    CHECK(((ariel::Algorithms::isContainsCycle(g) == "1->2->3->1") || (ariel::Algorithms::isContainsCycle(g) == "3->1->2->3")));
}

TEST_CASE("Test isBipartite")
{
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 0},
        {1, 0, 1},
        {0, 1, 0}};
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::isBipartite(g) == "The graph is bipartite: A={0,2}, B={1}");

    vector<vector<int>> graph2 = {
        {0, 1, 1, 0, 0},
        {1, 0, 1, 0, 0},
        {1, 1, 0, 1, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0}};
    g.loadGraph(graph2);
    CHECK(ariel::Algorithms::isBipartite(g) == "The graph is not bipartite");

    vector<vector<int>> graph3 = {
        {0, 1, 0, 0, 0},
        {1, 0, 3, 0, 0},
        {0, 3, 0, 4, 0},
        {0, 0, 4, 0, 5},
        {0, 0, 0, 5, 0}};
    g.loadGraph(graph3);
    CHECK(ariel::Algorithms::isBipartite(g) == "The graph is bipartite: A={0,2,4}, B={1,3}");

    vector<vector<int>> graph4 = {
        {0, -1, 0, 0, 0},
        {1, 0, 2, 0, 0},
        {0, 3, 0, 4, 0},
        {0, 0, -9, 0, 7},
        {0, 0, 0, 5, 0}};
    g.loadGraph(graph4);
    CHECK(ariel::Algorithms::isBipartite(g) == "The graph is bipartite: A={0,2,4}, B={1,3}");

    vector<vector<int>> graph5 = {
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0}};
    g.loadGraph(graph5);
    CHECK(ariel::Algorithms::isBipartite(g) == "The graph is bipartite: A={0,1,2}, B={}");

    vector<vector<int>> graph6 = {
        {0, 0},
        {0, 0}};
    g.loadGraph(graph6);
    CHECK(ariel::Algorithms::isBipartite(g) == "The graph is bipartite: A={0,1}, B={}");         
    // CHECK(ariel::Algorithms::isBipartite(g) == "The graph is bipartite: A={0}, B={1}");      ///mistake


    vector<vector<int>> graph7 = {
        {0}};
    g.loadGraph(graph7);
    CHECK(ariel::Algorithms::isBipartite(g) == "The graph is bipartite: A={0}, B={}");

    vector<vector<int>> graph8 = {
        {0, 0, 0, 0},
        {4, 0, -6, 0},
        {0, 0, 0, 5},
        {0, -2, 0, 0}};
    g.loadGraph(graph8);
    CHECK(ariel::Algorithms::isBipartite(g) == "The graph is not bipartite");

    vector<vector<int>> graph9 = {
        {0, 0, 0},
        {0, 0, 0},
        {1, 1, 0}};
    g.loadGraph(graph9);
    CHECK(ariel::Algorithms::isBipartite(g) == "The graph is bipartite: A={0,1}, B={2}");

    vector<vector<int>> graph10 = {
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 0},
        {0, 0, 1, 0}};
    g.loadGraph(graph10);
    CHECK(ariel::Algorithms::isBipartite(g) == "The graph is bipartite: A={0,2}, B={1,3}");

    vector<vector<int>> graph11 = {
        {0, 1, 0, 0},
        {0, 0, 0, 1},
        {1, 0, 0, 1},
        {0, 0, 0, 0}};
    g.loadGraph(graph11);
    CHECK(ariel::Algorithms::isBipartite(g) == "The graph is bipartite: A={0,3}, B={1,2}");

    vector<vector<int>> graph12 = {
        {0, 3, 7},
        {0, 0, 0},
        {7, 1, 0}};
    g.loadGraph(graph12);
    CHECK(ariel::Algorithms::isBipartite(g) == "The graph is not bipartite");

} 
TEST_CASE("Test invalid graph")
    {
        ariel::Graph g;
        vector<vector<int>> graph = {
            {0, 1, 2, 0},
            {1, 0, 3, 0},
            {2, 3, 0, 4},
            {0, 0, 4, 0},
            {0, 0, 0, 5}};
        CHECK_THROWS(g.loadGraph(graph));

        vector<vector<int>> graph1 = {
            {}};
        CHECK_THROWS(g.loadGraph(graph));

        vector<vector<int>> graph2 = {// empty graph, 1 vertex and 0 edges
                                      {0}};
        CHECK_NOTHROW(g.loadGraph(graph2));
        // vector<vector<int>> graph3 = {
        //     {1}};
        // CHECK_THROWS(g.loadGraph(graph3));

        // vector<vector<int>> graph4 = {
        //     {1, 3, 4},
        //     {0, 0, 0},
        //     {0, 0, 0}};
        // CHECK_THROWS(g.loadGraph(graph4));
    }
    TEST_CASE("Test negative cycle")
    {
        ariel::Graph g;
        vector<vector<int>> graph = {
            {0, 0, 0, 0},
            {4, 0, -6, 0},
            {0, 0, 0, 5},
            {0, -2, 0, 0}};
        g.loadGraph(graph);
        CHECK(ariel::Algorithms::negativeCycle(g) == "1->2->3->1");

        vector<vector<int>> graph3 = {
            {0, 8, -1},
            {8, 0, -5},
            {-1, -5, 0}};
        g.loadGraph(graph3);
        CHECK(ariel::Algorithms::negativeCycle(g) == "No negative cycle exists");

        vector<vector<int>> graph4 = {
            {0, 8, -2},
            {8, 0, -5},
            {-1, -5, 0}};
        g.loadGraph(graph4);
        CHECK(((ariel::Algorithms::negativeCycle(g) == "0->2->0") || (ariel::Algorithms::negativeCycle(g) == "2->0->2")));

        vector<vector<int>> graph5 = {
            {0, 8, -1},
            {8, 0, -4},
            {-1, -5, 0}};
        g.loadGraph(graph5);
        CHECK(((ariel::Algorithms::negativeCycle(g) == "1->2->1") || (ariel::Algorithms::negativeCycle(g) == "2->1->2")));

        // vector<vector<int>> graph8 = {
        //     {0, 0, 0, 0},
        //     {4, 0, -6, 0},
        //     {0, 0, 0, 5},
        //     {0, -2, 0, 0}};
        // g.loadGraph(graph8);
        // CHECK(ariel::Algorithms::negativeCycle(g) == "1->2->3->1");
    }