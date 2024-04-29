// ID: 200661775
// Email: origoldbsc@gmail.com

#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <vector>

using namespace std; 

namespace ariel {

    /**
     * @This class represents a graph by using an adjacency matrix.
     */
    class Graph 
    {

    // Private attributes and functions   
    private:
        vector<vector<int>> adjacencyMatrix;   // A variable that stores the adjacency matrix which representing the graph. 
        size_t numVertices;                    // A variable that stores the number of vertices in the graph.
        size_t numEdges;                       // A variable that stores the  number of edges in the graph.
        bool isDirected;                       // A flag that indicates if the graph is directed or undirected.


        /**
         * @brief This auxiliary function determines if a graph is directed or not.
         * @return true if the graph is directed, otherwise false.
         */
        bool checkDirected();


        /**
         * @brief This auxiliary function countd the number of edges in a graph.
         * @return number of edges.
         */
        size_t countEdges();


    // Public methods  
    public:

        /**
         * @brief A default constructor for the Graph class.
         */
        Graph();


        /**
         * @brief This method loads a graph from an adjacency matrix.
         * @param matrix The adjacency matrix representing the graph.
         * @throws If the matrix is empty or not square throw invalid_argument exception
         */
        void loadGraph(vector<vector<int>>& matrix);


        /**
         * @brief This method prints the number of vertices and edges in the graph.
         */
        void printGraph();


        /**
         * @brief This method returns the number of vertices in the graph.
         * @return The number of vertices.
         */
        size_t getNumVertices();


        /**
         * @brief This method returns the number of edges in the graph.
         * @return The number of edges.
         */
        size_t getNumEdges();


        /**
         * @brief This method returns if an instance of a graph is directed or not.
         */
        
        bool isGraphDirected();


        /**
         * @brief This method returns the adjacency matrix of the graph. 
         * A 2D array represented by vetcor of vectors.
         * @return A reference to the adjacency matrix.
         */
        vector<vector<int>>& getAdjacencyMatrix();
    };
}

#endif