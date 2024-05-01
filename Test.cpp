// ID: 200661775
// Email: origoldbsc@gmail.com

#include "doctest.h"
#include "Algorithms.hpp"
#include "Graph.hpp"
#include <algorithm>  
#include <iostream>

using namespace std;
using namespace ariel;

Graph g;         // Initiate an empty instance of graph to be used in the following tests

/*********************************************/
///             TEST FOR GRAPH              ///
/*********************************************/

TEST_CASE("Test empty graph") {
    CHECK(g.getNumVertices() == 0);
    CHECK(g.getNumEdges() == 0);
    CHECK(g.getAdjacencyMatrix().empty());
}

TEST_CASE("Test undirected graph") {
    vector<vector<int>> matrix = {
        {0, 1, 0, 1},
        {1, 0, 1, 0},
        {0, 1, 0, 1},
        {1, 0, 1, 0}};

    g.loadGraph(matrix);
    CHECK(g.getNumVertices() == 4);
    CHECK(g.getNumEdges() == 4);
    CHECK(g.getAdjacencyMatrix() == matrix);
    CHECK(!g.isGraphDirected());
}

TEST_CASE("Test undirected graph with self-loops") {
    vector<vector<int>> matrix = {
        {1, 1, 0, 1},
        {1, 1, 1, 0},
        {0, 1, 1, 1},
        {1, 0, 1, 1}};
    g.loadGraph(matrix);

    CHECK(g.getNumVertices() == 4);
    CHECK(g.getNumEdges() == 8);
    CHECK(g.getAdjacencyMatrix() == matrix);
    CHECK(!g.isGraphDirected());
}

TEST_CASE("Test disconnected undirected graph") {
    vector<vector<int>> matrix = {
        {0, 1, 0, 0},
        {1, 0, 0, 0},
        {0, 0, 0, 1},
        {0, 0, 1, 0}};

    g.loadGraph(matrix);
    CHECK(g.getNumVertices() == 4);
    CHECK(g.getNumEdges() == 2);
    CHECK(g.getAdjacencyMatrix() == matrix);
    CHECK(!g.isGraphDirected());
}

TEST_CASE("Test directed graph") {
    vector<vector<int>> matrix = {
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1},
        {1, 0, 0, 0}};

    g.loadGraph(matrix);
    CHECK(g.getNumVertices() == 4);
    CHECK(g.getNumEdges() == 4);
    CHECK(g.getAdjacencyMatrix() == matrix);
    CHECK(g.isGraphDirected());
}

TEST_CASE("Test disconnected directed graph") {
    vector<vector<int>> matrix = {
        {0, 1, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 1},
        {0, 0, 0, 0}};

    g.loadGraph(matrix);
    CHECK(g.getNumVertices() == 4);
    CHECK(g.getNumEdges() == 2);
    CHECK(g.getAdjacencyMatrix() == matrix);
    CHECK(g.isGraphDirected());
}

TEST_CASE("Test invalid graph") {
    vector<vector<int>> matrix = {
        {0, 1, 0},
        {1, 0, 1, 0},
        {0, 1, 0}};

    REQUIRE_THROWS_WITH(g.loadGraph(matrix), "Invalid graph: The graph is not a square matrix");
}

TEST_CASE("Test invalid graph: non-square matrix") {
    vector<vector<int>> matrix = {
        {0, 1, 2, 0},
        {1, 0, 3, 0},
        {2, 3, 0, 4},
        {0, 0, 4, 0},
        {0, 0, 0, 5}};
    REQUIRE_THROWS_WITH(g.loadGraph(matrix), "Invalid graph: The graph is not a square matrix");
}

TEST_CASE("Test invalid graph: non-square matrix") {
    vector<vector<int>> matrix = {
        {0, 1, 2, 0, 1},
        {1, 0, 3, 0, 2},
        {2, 3, 0, 4, 4},
        {0, 0, 4, 0, 3}};
    REQUIRE_THROWS_WITH(g.loadGraph(matrix), "Invalid graph: The graph is not a square matrix");
}

TEST_CASE("Test invalid graph with empty matrix") {
    vector<vector<int>> matrix;
    REQUIRE_THROWS_WITH(g.loadGraph(matrix), "Invalid graph: The graph matrix is empty");
}

TEST_CASE("Graph with 1 vertex and 0 edges is valid") {
vector<vector<int>> matrix = {{0}};
    CHECK_NOTHROW(g.loadGraph(matrix));
}

/*********************************************/
///           TEST FOR ALGORITHMS           ///
/*********************************************/

// isConnected tests

TEST_CASE("Test isConnected: Single vertex graph") {
    vector<vector<int>> matrix = {{0}};
    g.loadGraph(matrix);
    CHECK(Algorithms::isConnected(g) == true);

    matrix = {{5}};
    g.loadGraph(matrix);
    CHECK(Algorithms::isConnected(g) == true);
}

TEST_CASE("Test isConnected: Undirected graph") {
    vector<vector<int>> matrix = {
        {0, 3, 0, 0},
        {3, 0, -1, 4},
        {0, -1, 0, 0},
        {0, 4, 0, 0}};
    g.loadGraph(matrix);
    CHECK(Algorithms::isConnected(g) == true);
}

TEST_CASE("Test isConnected: Undirected Disconnected graph") {
    vector<vector<int>> matrix = {
        {0, 1, 0, 0},
        {1, 0, 0, 0},
        {0, 0, 0, 1},
        {0, 0, 1, 0}};
    g.loadGraph(matrix);
    CHECK(Algorithms::isConnected(g) == false);
}

TEST_CASE("Test isConnected: Disconnected graph with multiple components") {
    vector<vector<int>> matrix = {
        {0, 1, 0, 0, 0, 0},
        {1, 0, 0, 0, 0, 0},
        {0, 0, 0, 1, 0, 0},
        {0, 0, 1, 0, 0, 0},
        {0, 0, 0, 0, 0, 1},
        {0, 0, 0, 0, 1, 0}};
    g.loadGraph(matrix);
    CHECK(Algorithms::isConnected(g) == false);
}

TEST_CASE("Test isConnected: Graph with self-loops") {
    vector<vector<int>> matrix = {
        {1, 1, 0, 0},
        {1, 1, 1, 0},
        {0, 1, 0, 1},
        {0, 0, 1, 1}};
    g.loadGraph(matrix);
    CHECK(Algorithms::isConnected(g) == true);
}

TEST_CASE("Test isConnected: Graph with a single edge") {
    vector<vector<int>> matrix = {
        {0, 1, 0},
        {0, 0, 0},
        {0, 0, 0}};
    g.loadGraph(matrix);
    CHECK(Algorithms::isConnected(g) == false);
}

TEST_CASE("Test isConnected: Complete graph") {
    vector<vector<int>> matrix = {
        {0, 1, 1, 1},
        {1, 0, 1, 1},
        {1, 1, 0, 1},
        {1, 1, 1, 0}};
    g.loadGraph(matrix);
    CHECK(Algorithms::isConnected(g) == true);
}

TEST_CASE("Test isConnected: Star graph") {
    vector<vector<int>> matrix = {
        {0, 1, 1, 1, 1},
        {1, 0, 0, 0, 0},
        {1, 0, 0, 0, 0},
        {1, 0, 0, 0, 0},
        {1, 0, 0, 0, 0}};
    g.loadGraph(matrix);
    CHECK(Algorithms::isConnected(g) == true);
}

TEST_CASE("Test isConnected: Graph with a single isolated vertex") {
    vector<vector<int>> matrix = {
        {0, 0, 0, 0},
        {0, 0, 1, 1},
        {0, 1, 0, 1},
        {0, 1, 1, 0}};
    g.loadGraph(matrix);
    CHECK(Algorithms::isConnected(g) == false);
}

TEST_CASE("Test isConnected: Graph with all vertices isolated") {
    vector<vector<int>> matrix = {
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0}};
    g.loadGraph(matrix);
    CHECK(Algorithms::isConnected(g) == false);
}

TEST_CASE("Test isConnected: Graph with all vertices connected to themselves") {
    vector<vector<int>> matrix = {
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}};
    g.loadGraph(matrix);
    CHECK(Algorithms::isConnected(g) == false);
}



// isStronglyConnected tests

TEST_CASE("Test isStronglyConnected: Graph with negative edges"){
    vector<vector<int>> matrix = {
        {0, -1, 0},
        {-1, 0, -1},
        {0, -1, 0}};

    g.loadGraph(matrix);
    CHECK(Algorithms::isStronglyConnected(g) == true);
}

TEST_CASE("Test isStronglyConnected: Directed graph with one edge"){
    vector<vector<int>> matrix = {
        {0, 0},
        {6, 0}}; 

    g.loadGraph(matrix);
    CHECK(Algorithms::isStronglyConnected(g) == false);
}
    
TEST_CASE("Test isStronglyConnected: Directed graph with one isolated vertex"){
    vector<vector<int>> matrix = {
        {0, 1, 1, 0, 0},
        {1, 0, 1, 0, 0},
        {1, 1, 0, 1, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0}};
    g.loadGraph(matrix);
    CHECK(Algorithms::isStronglyConnected(g) == false);
}

TEST_CASE("Test isStronglyConnected: Directed graph"){
    vector<vector<int>> matrix = {
        {0, 2},
        {2, 0}};
    g.loadGraph(matrix);
    CHECK(Algorithms::isStronglyConnected(g) == true);
}


// shortestPath tests

TEST_CASE("Test shortestPath: Single vertex graph") {
    vector<vector<int>> matrix = {{0}};
    g.loadGraph(matrix);
    CHECK(Algorithms::shortestPath(g, 0, 0) == "No path exists between a vertex and itself");
}

TEST_CASE("Test shortestPath (BFS): No path exists") {
    vector<vector<int>> matrix = {
        {0, 1, 0, 0},
        {1, 0, 0, 0},
        {0, 0, 0, 1},
        {0, 0, 1, 0}};
    g.loadGraph(matrix);
    CHECK(Algorithms::shortestPath(g, 0, 3) == "No path exists between 0 and 3");
}

TEST_CASE("Test shortestPath (BFS): Multiple paths exist") {
    vector<vector<int>> matrix = {
        {0, 1, 1, 0},
        {1, 0, 1, 1},
        {1, 1, 0, 1},
        {0, 1, 1, 0}};
    g.loadGraph(matrix);
    CHECK(((Algorithms::shortestPath(g, 0, 3) == "0->1->3" ) || (Algorithms::shortestPath(g, 0, 3) == "0->2->3")));
}

TEST_CASE("Test shortestPath (Dijkstra): Shortest path with weighted edges") {

    vector<vector<int>> matrix = {
        {0, 2, 0, 6, 0},
        {2, 0, 3, 8, 5},
        {0, 3, 0, 0, 7},
        {6, 8, 0, 0, 9},
        {0, 5, 7, 9, 0}};
    g.loadGraph(matrix);
    CHECK(Algorithms::shortestPath(g, 0, 3) == "0->3");
}

TEST_CASE("Test shortestPath: Graph with a longer path") {
    vector<vector<int>> matrix = {
        {0, 1, 0, 0, 0},
        {1, 0, 1, 0, 0},
        {0, 1, 0, 1, 0},
        {0, 0, 1, 0, 1},
        {0, 0, 0, 1, 0}};
    g.loadGraph(matrix);
    CHECK(Algorithms::shortestPath(g, 0, 4) == "0->1->2->3->4");
}


TEST_CASE("Test shortestPath (Dijkstra): Graph with multiple paths") {
    vector<vector<int>> matrix = {
        {0, 3, 0, 7, 0},
        {3, 0, 4, 2, 0},
        {0, 4, 0, 5, 6},
        {7, 2, 5, 0, 1},
        {0, 0, 6, 1, 0}
    };
    
    g.loadGraph(matrix);
    
    CHECK(Algorithms::shortestPath(g, 0, 4) == "0->1->3->4");
    CHECK(Algorithms::shortestPath(g, 1, 2) == "1->2");
    CHECK(Algorithms::shortestPath(g, 2, 0) == "2->1->0");
    CHECK(Algorithms::shortestPath(g, 4, 1) == "4->3->1");
}

TEST_CASE("Test shortestPath (BF): Graph with multiple paths"){
    vector<vector<int>> matrix = {
        {0, -2, 4, 0, 0},
        {-2, 0, 3, 2, 2},
        {4, 3, 0, 5, 0},
        {0, 2, 5, 0, 0},
        {0, 2, 0, 0, 0}};
    g.loadGraph(matrix);
    CHECK(Algorithms::shortestPath(g, 0, 3) == "0->1->3");
    CHECK(Algorithms::shortestPath(g, 1, 4) == "1->4");
    CHECK(Algorithms::shortestPath(g, 2, 4) == "2->0->1->4");
    CHECK(Algorithms::shortestPath(g, 3, 0) == "3->1->0");
}

TEST_CASE("Test shortestPath (Dijkstra): Invalid vertex"){
    vector<vector<int>> matrix = {
            {0, 1, 1, 0},
            {1, 0, 0, 0},
            {1, 1, 0, 0},
            {0, 0, 1, 0}};
        g.loadGraph(matrix);
        CHECK(ariel::Algorithms::shortestPath(g, 0, 4) == "Invalid start or end vertex");
        CHECK(ariel::Algorithms::shortestPath(g, 4, 0) == "Invalid start or end vertex");
        CHECK(ariel::Algorithms::shortestPath(g, 0, 3) == "No path exists between 0 and 3");
        CHECK(ariel::Algorithms::shortestPath(g, 1, 1) == "No path exists between a vertex and itself");
}

// isContainsCycle tests

TEST_CASE("Test isContainsCycle: Single vertex graph") {
    vector<vector<int>> matrix = {{0}};
    g.loadGraph(matrix);
    CHECK(Algorithms::isContainsCycle(g) == "0");
}

TEST_CASE("Test isContainsCycle: Undirected graph with no cycle") {
    vector<vector<int>> matrix = {
        {0, 1, 0},
        {1, 0, 1},
        {0, 1, 0}};
    g.loadGraph(matrix);
    CHECK(Algorithms::isContainsCycle(g) == "0");
}

TEST_CASE("Test isContainsCycle: Undirected graph with a cycle") {
    vector<vector<int>> matrix = {
        {0, 1, 1, 0, 0},
        {1, 0, 1, 0, 0},
        {1, 1, 0, 1, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0}};
    g.loadGraph(matrix);
    CHECK(((Algorithms::isContainsCycle(g) == "0->1->2->0") || (Algorithms::isContainsCycle(g) == "2->0->1->2")));
}

TEST_CASE("Test isContainsCycle: Directed graph with a cycle") {
    vector<vector<int>> matrix = {
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1},
        {1, 0, 0, 0}};
    g.loadGraph(matrix);
    CHECK(((Algorithms::isContainsCycle(g) == "0->1->2->3->0") || (Algorithms::isContainsCycle(g) == " 3->0->1->2->3")));
}


TEST_CASE("Test isContainsCycle: Graph with self-loop") {
    vector<vector<int>> matrix = {
        {2, 1, 0, 0, 0},
        {1, 0, 1, 0, 0},
        {0, 1, 0, 0, 1},
        {0, 0, 1, 0, 1},
        {0, 0, 1, 0, 0}
    };
    g.loadGraph(matrix);
    CHECK(Algorithms::isContainsCycle(g) == "0->0");
}


TEST_CASE("Test isContainsCycl: Large graph with long cycle") {

    vector<vector<int>> matrix = {
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
    g.loadGraph(matrix);
    CHECK(((ariel::Algorithms::isContainsCycle(g) == "0->1->2->3->4->5->6->7->8->9->0") || (ariel::Algorithms::isContainsCycle(g) == "9->0->1->2->3->4->5->6->7->8->9")));
}


// isBipartite tests

TEST_CASE("Test isBipartite: Graph with an odd cycle (not bipartite)") {
    vector<vector<int>> matrix = {
        {0, 1, 0, 0, 1},
        {1, 0, 1, 0, 0},
        {0, 1, 0, 1, 0},
        {0, 0, 1, 0, 1},
        {1, 0, 0, 1, 0}};
    g.loadGraph(matrix);
    CHECK(Algorithms::isBipartite(g) == "The graph is not bipartite");
}


TEST_CASE("Test isBipartite: Single vertex graph") {
    vector<vector<int>> matrix = {{0}};
    g.loadGraph(matrix);
    CHECK(Algorithms::isBipartite(g) == "The graph is bipartite: A={0}, B={}");
}

TEST_CASE("Test isBipartite: Graph without edges") {
 vector<vector<int>> matrix = {
        {0, 0},
        {0, 0}};
    g.loadGraph(matrix);
    CHECK(Algorithms::isBipartite(g) == "The graph is bipartite: A={0,1}, B={}");         
}

TEST_CASE("Test isBipartite: Bipartite graph") {
    vector<vector<int>> matrix = {
        {0, 1, 0},
        {1, 0, 1},
        {0, 1, 0}};
    g.loadGraph(matrix);
    CHECK(Algorithms::isBipartite(g) == "The graph is bipartite: A={0,2}, B={1}");
}

TEST_CASE("Test isBipartite: Non-bipartite graph") {
    vector<vector<int>> matrix = {
        {0, 1, 1, 0, 0},
        {1, 0, 1, 0, 0},
        {1, 1, 0, 1, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0}};
    g.loadGraph(matrix);
    CHECK(Algorithms::isBipartite(g) == "The graph is not bipartite");
}

TEST_CASE("Test isBipartite: Bipartite graph with connected components") {
    vector<vector<int>> matrix = {
        {0, 1, 0, 0, 0, 0, 0},
        {1, 0, 1, 0, 0, 0, 0},
        {0, 1, 0, 1, 0, 0, 0},
        {0, 0, 1, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 1, 0, 1},
        {0, 0, 0, 0, 0, 1, 0}};
    g.loadGraph(matrix);

    CHECK(((Algorithms::isBipartite(g) == "The graph is bipartite: A={0,2,4,6}, B={1,3,5}")
    || (Algorithms::isBipartite(g) == "The graph is bipartite: A={0,2,5}, B={1,3,4,6}")));
}

TEST_CASE("Test isBipartite: Bipartite graph with an even cycle") {
    vector<vector<int>> matrix = {
        {0, 1, 0, 0, 0, 1},
        {1, 0, 1, 0, 0, 0},
        {0, 1, 0, 1, 0, 0},
        {0, 0, 1, 0, 1, 0},
        {0, 0, 0, 1, 0, 1},
        {1, 0, 0, 0, 1, 0}};
    g.loadGraph(matrix);
    CHECK(Algorithms::isBipartite(g) == "The graph is bipartite: A={0,2,4}, B={1,3,5}");
}


// negativeCircle tests

TEST_CASE("Test negativeCycle: Graph with negative weights and a positive cycle") {
    vector<vector<int>> matrix = {
        {0, 2, 0, 6, 0},
        {2, 0, 3, 8, 5},
        {0, 3, 0, 0, 7},
        {6, 8, 0, 0, 9},
        {0, 5, 7, 9, 0}};
    g.loadGraph(matrix);
    CHECK(Algorithms::negativeCycle(g) == "No negative cycle exists");
}


TEST_CASE("Test negativeCycle: Graph with negative weights but no cycle") {
    vector<vector<int>> matrix = {
        {0, -2, 0, 0},
        {0, 0, -3, 0},
        {0, 0, 0, -1},
        {0, 0, 0, 0}};
    g.loadGraph(matrix);
    CHECK(Algorithms::negativeCycle(g) == "No negative cycle exists");
}

TEST_CASE("Test negativeCycle: Graph with negative weights and a zero-weight cycle") {
    vector<vector<int>> matrix = {
        {0, -2, 0, 0},
        {0, 0, -3, 0},
        {0, 3, 0, 0},
        {0, 0, 0, 0}};
    g.loadGraph(matrix);
    CHECK(Algorithms::negativeCycle(g) == "No negative cycle exists");
}
    
TEST_CASE("Test negativeCycle: Graph with positive weights") {
    vector<vector<int>> matrix = {
        {0, 2, 0, 6, 0},
        {2, 0, 3, 8, 5},
        {0, 3, 0, 0, 7},
        {6, 8, 0, 0, 9},
        {0, 5, 7, 9, 0}};
    g.loadGraph(matrix);
    CHECK(Algorithms::negativeCycle(g) == "No negative cycle exists");
}

TEST_CASE("Test negativeCycle: Graph with negative weights and disconnected components") {
    vector<vector<int>> matrix = {
        {0, -2, 0, 0, 0},
        {-2, 0, 0, 0, 0},
        {0, 0, 0, -3, 0},
        {0, 0, -3, 0, 0},
        {0, 0, 0, 0, 0}};
    g.loadGraph(matrix);
    CHECK(Algorithms::negativeCycle(g) == "No negative cycle exists");        
}

TEST_CASE("Test negativeCycle: Graph with negative weights and a self-loop") {
    vector<vector<int>> matrix = {
        {-1, 0, 0, 0},
        {0, 0, -2, 0},
        {0, 0, 0, -3},
        {0, 0, 0, 0}};
    g.loadGraph(matrix);
    CHECK(Algorithms::negativeCycle(g) == "0->0");
}

TEST_CASE("Test negativeCycle: Graph with negative cycle") {
    vector<vector<int>> matrix = {
        {0, -2, 0, 6, 0},
        {-2, 0, -3, 8, 5},
        {0, -3, 0, -4, 7},
        {6, 8, -4, 0, 9},
        {0, 5, 7, 9, 0}};
    g.loadGraph(matrix);
    CHECK(((Algorithms::negativeCycle(g) == "0->1->2->3->0") || (Algorithms::negativeCycle(g) == "1->2->3->0->1")));
}

TEST_CASE("Test negativeCycle: Graph with a negative weight cycle") {
    vector<vector<int>> matrix = {
        {0, -1, 0, 0},
        {0, 0, -1, 0},
        {0, 0, 0, -1},
        {-1, 0, 0, 0}};
    g.loadGraph(matrix);
    CHECK(((Algorithms::negativeCycle(g) == "0->1->2->3->0") || (Algorithms::negativeCycle(g) == "1->2->3->0->1")));
}

TEST_CASE("Test negativeCycle: Graph with negative weights and no negative cycle") {
    vector<vector<int>> matrix = {
        {0, -1, 0, 0, 0},
        {0, 0, -1, 0, 0},
        {0, 0, 0, -1, 0},
        {0, 0, 0, 0, -1},
        {0, 0, 0, 0, 0}};
    g.loadGraph(matrix);
    CHECK(Algorithms::negativeCycle(g) == "No negative cycle exists");
}

TEST_CASE("Test negativeCycle: Graph with a negative weight edge but no negative cycle") {
    vector<vector<int>> matrix = {
        {0, -1, 0, 0},
        {0, 0, -1, 0},
        {0, 0, 0, -1},
        {0, 0, 0, 0}};
    g.loadGraph(matrix);
    CHECK(Algorithms::negativeCycle(g) == "No negative cycle exists");
}


TEST_CASE("Test negativeCycle: Directed weighted graph with cycle") {
vector<vector<int>> matrix = {
        {0, -4, 3},
        {-3, 0, -5},
        {3, -5, 0}};
    g.loadGraph(matrix);   
    CHECK(((Algorithms::negativeCycle(g) == "0->1->0") || (Algorithms::negativeCycle(g) == "1->0->1"))); 
}

TEST_CASE("Test negativeCircle: Compelx graph with two negative cycles") {
    vector<vector<int>> matrix = {
        {0, -2, 0, 0, 0},
        {-2, 0, -1, 0, 0},
        {-4, -9, 0, -7, 0},
        {0, 0, -1, 0, -1},
        {0, 0, 0, -1, 0}
    };
    g.loadGraph(matrix);
    CHECK(((Algorithms::negativeCycle(g) == "1->2->1") || (Algorithms::negativeCycle(g) == "2->1->2") 
    || (Algorithms::negativeCycle(g) == "4->5->4") || (Algorithms::negativeCycle(g) == "5->4->5") ));
}    