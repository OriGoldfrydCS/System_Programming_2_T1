// ID: 200661775
// Email: origoldbsc@gmail.com

#include "Graph.hpp"
#include <iostream>
#include <stdexcept>

using namespace std;

namespace ariel {

    /**
     * @brief A default constructor for the Graph class.
     */
    Graph::Graph() : _numVertices(0), _numEdges(0), _isDirected(false) {}


    /**
     * @brief This method loads a graph from an adjacency matrix.
     * @param matrix The adjacency matrix representing the graph.
     * @throws If the matrix is empty or not square throw invalid_argument exception
     */
    void Graph::loadGraph(vector<vector<int>>& matrix) 
    {
        // Matrix with no vertices is invalid graph
        if (matrix.empty()) 
        {
            throw invalid_argument("Invalid graph: The graph matrix is empty");
        }

        _numVertices = matrix.size();            // A variable to store how many vertices have in the  graph
        _numEdges = 0;                           // A variable to store how many edges have in the  graph

        // Check if the matrix is square
        for (size_t i = 0; i < matrix.size(); i++)
        {
        if (matrix[i].size() != _numVertices)
            {
                throw invalid_argument("Invalid graph: The graph is not a square matrix");
            }
        }
        
        _adjacencyMatrix = matrix;

        // Check if the graph is directed or not
        _isDirected = checkDirected();

        // Count the number of edges based on graph type
        _numEdges = countEdges();
    }
        

    /**
     * @brief This method prints the number of vertices and edges in the graph.
     */
    // NOLINTNEXTLINE
    void Graph::printGraph() const
    {
        cout << "Graph with " << _numVertices << " vertices and " << _numEdges << " edges" << endl;
    }


    /**
     * @brief This method returns the number of vertices in the graph.
     * @return The number of vertices.
     */
    size_t Graph::getNumVertices() const
    {
        return _numVertices;
    }


    /**
     * @brief This method returns the number of edges in the graph.
     * @return The number of edges.
     */
    size_t Graph::getNumEdges() const
    {
        return _numEdges;
    }


    /**
     * @brief This method returns if the graph is directed or  not.
     * @return True if the graph is directed, false otherwise.
     */
    bool Graph::isGraphDirected() const
    {
        return _isDirected;
    }


    /**
     * @brief This method returns the adjacency matrix of the graph. 
     * A 2D array represented by vetcor of vectors.
     * @return A reference to the adjacency matrix.
     */
    vector<vector<int>>& Graph::getAdjacencyMatrix() 
    {
        return _adjacencyMatrix;
    }


    /*********************************************/
    ///             PRIVATE SECTION             ///
    /*********************************************/

    /**
    * @brief This auxiliary function determines if a graph is directed or not.
    * @return true if the graph is directed, otherwise false.
    */
    bool Graph::checkDirected()
    {
        for (size_t vertex_v = 0; vertex_v < _numVertices; vertex_v++)
        {
            for (size_t vertex_u = 0; vertex_u < _numVertices; vertex_u++)
            {
                if (_adjacencyMatrix[vertex_v][vertex_u] != _adjacencyMatrix[vertex_u][vertex_v])
                {
                    return true;
                }
            }
        }
        return false;
    }


    /**
    * @brief This auxiliary function countd the number of edges in a graph.
    * @return number of edges.
    */
    size_t Graph::countEdges()
    {
        size_t count = 0;
        if (_isDirected)
        {
            // Count edges for directed graph
            for (size_t i = 0; i < _numVertices; i++)
            {
                for (size_t j = 0; j < _numVertices; j++)
                {
                    if (_adjacencyMatrix[i][j] != 0)
                    {
                        count++;
                    }
                }
            }
        }
        else
        {
            // Count edges for undirected graph
            for (size_t i = 0; i < _numVertices; i++)
            {
                for (size_t j = i; j < _numVertices; j++)
                {
                    if (_adjacencyMatrix[i][j] != 0)
                    {
                        count++;
                    }
                }
            }
        }
        return count;
    }
}