/*
COP3503 Programming Assignment - Dijkstra's Algorithm
Author: Devon Villalona
How to Run the Program:
1. Compile the Java program using the following command:
   javac DijkstraAlgorithm.java

2. Make sure the input file "cop3503-dijkstra-input.txt" is in the same directory as the compiled class file.
 
3. Run the compiled Java program using the command:
   java DijkstraAlgorithm
*/

import java.io.*;
import java.util.*;

public class DijkstraAlgorithm {
    public static void main(String[] args) {
        // Input and output file paths
        String inputFile = "cop3503-dijkstra-input.txt";
        String outputFile = "cop3503-dijkstra-output-Villalona-Devon.txt";

        try (BufferedReader reader = new BufferedReader(new FileReader(inputFile));
             PrintWriter writer = new PrintWriter(new FileWriter(outputFile))) {

            // Read the number of vertices, source vertex, and number of edges from the input file
            int numVertices = Integer.parseInt(reader.readLine().trim());
            int sourceVertex = Integer.parseInt(reader.readLine().trim());
            int numEdges = Integer.parseInt(reader.readLine().trim());

            // Initialize the graph as an adjacency list
            List<List<Edge>> graph = new ArrayList<>(numVertices + 1);
            for (int i = 0; i <= numVertices; i++) {
                graph.add(new ArrayList<>());
            }

            // Read edges from the input file and add them to the graph
            for (int i = 0; i < numEdges; i++) {
                String[] edgeInfo = reader.readLine().split(" ");
                int from = Integer.parseInt(edgeInfo[0]);
                int to = Integer.parseInt(edgeInfo[1]);
                int weight = Integer.parseInt(edgeInfo[2]);
                graph.get(from).add(new Edge(to, weight));
                graph.get(to).add(new Edge(from, weight));
            }

            // Run Dijkstra's algorithm to find the shortest paths from the source vertex
            dijkstra(graph, numVertices, sourceVertex, writer);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    static void dijkstra(List<List<Edge>> graph, int numVertices, int source, PrintWriter writer) {
        // Array to store the shortest distances from the source to each vertex
        int[] dist = new int[numVertices + 1];
        // Array to store the parent of each vertex in the shortest path tree
        int[] parent = new int[numVertices + 1];
        // Boolean array to mark visited vertices
        boolean[] visited = new boolean[numVertices + 1];

        // Initialize distances to infinity and parents to -1
        Arrays.fill(dist, Integer.MAX_VALUE);
        Arrays.fill(parent, -1);
        // Distance to the source itself is 0
        dist[source] = 0;

        // Priority queue to select the vertex with the minimum distance
        PriorityQueue<Vertex> pq = new PriorityQueue<>(Comparator.comparingInt(v -> v.distance));
        pq.add(new Vertex(source, 0));

        while (!pq.isEmpty()) {
            // Extract the vertex with the minimum distance
            Vertex current = pq.poll();
            int u = current.id;

            // Skip if the vertex has already been visited
            if (visited[u]) continue;
            visited[u] = true;

            // Relax all adjacent vertices of the current vertex
            for (Edge edge : graph.get(u)) {
                int v = edge.to;
                int weight = edge.weight;

                // If a shorter path to v is found, update the distance and parent
                if (!visited[v] && dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    parent[v] = u;
                    pq.add(new Vertex(v, dist[v]));
                }
            }
        }

        // Writing the result to output file
        writer.println(numVertices);
        for (int i = 1; i <= numVertices; i++) {
            // Print the vertex, its distance from the source, and its parent in the shortest path tree
            writer.printf("%d %d %d%n", i, (i == source) ? -1 : (dist[i] == Integer.MAX_VALUE ? -1 : dist[i]), parent[i]);
        }
    }

    // Class to represent an edge in the graph
    static class Edge {
        int to, weight;

        Edge(int to, int weight) {
            this.to = to;
            this.weight = weight;
        }
    }

    // Class to represent a vertex and its distance from the source
    static class Vertex {
        int id, distance;

        Vertex(int id, int distance) {
            this.id = id;
            this.distance = distance;
        }
    }
}
