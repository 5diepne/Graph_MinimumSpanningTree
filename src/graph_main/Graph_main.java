/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Main.java to edit this template
 */
package graph_main;

import java.awt.BorderLayout;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Scanner;

/**
 *
 * @author DiepTCNN
 */
class Node {

    int vertex;
    int weight;

    public Node() {
    }

    public Node(int vertex, int weight) {
        this.vertex = vertex;
        this.weight = weight;
    }

    @Override
    public String toString() {
        return "(" + vertex + ", " + weight + ")";
    }

}

class Edge {

    int source;
    int destination;
    int cost;

    public Edge(int source, int destination, int cost) {
        this.source = source;
        this.destination = destination;
        this.cost = cost;
    }

    @Override
    public String toString() {
        return source + " -> " + destination + " (Cost: " + cost + ")";
    }
}

class Graph {

    int n; // n là số đỉnh của graph
    ArrayList<Node>[] adj_list;
    ArrayList<Edge>[] adjList;

    public Graph(String filename) throws FileNotFoundException {
        Scanner sc = new Scanner(new File(filename));
        n = Integer.parseInt(sc.nextLine()); //6
        adj_list = new ArrayList[n + 1]; //1-based index
    //    adjList = new ArrayList[n + 1];
        for (int i = 1; i <= n; i++) {
            adj_list[i] = new ArrayList<>();
            String line = sc.nextLine();
            String[] sub_str = line.split(" ");
            for (int j = 0; j < sub_str.length; j += 2) {
                adj_list[i].add(new Node(Integer.parseInt(sub_str[j]),
                        Integer.parseInt(sub_str[j + 1])));
            }

        }
        sc.close();
    }

    void Display() {
        System.out.println("Number of Vertex = " + n);
        for (int i = 1; i <= n; i++) {
            System.out.print(i + " -->");
            for (Node node : adj_list[i]) {
                System.out.print(node + " ");
            }
            System.out.println("");
        }
    }

    void Dijkstra(int Start, int End) {
        PriorityQueue<Node> DSCho = new PriorityQueue<>( //DS chờ danh sách chờ
                (a, b) -> (a.weight - b.weight)); // trọng số a-b bé
        DSCho.add(new Node(Start, 0));
        int[] label = new int[n + 1];
        for (int i = 1; i <= n; i++) {
            label[i] = Integer.MAX_VALUE;
        }
        label[Start] = 0;
        int[] Pred = new int[n + 1];
        while (!DSCho.isEmpty()) {
            Node chon = DSCho.poll();
            System.out.println("." + chon.vertex);
            System.out.println("Chi phi cua duong di ngan nhat -> " + label[chon.vertex]);
            for (int back = chon.vertex; back != Start; back = Pred[back]) {
                System.out.println(back + " <- ");
            }
            System.out.println(Start);
            if (chon.vertex == End) {
                break;
            }
            for (Node LC : adj_list[chon.vertex]) { //LC là lân cận
                if (label[LC.vertex] > label[chon.vertex] + LC.weight) {
                    label[LC.vertex] = label[chon.vertex] + LC.weight;
                    DSCho.removeIf(n -> (n.vertex == LC.vertex));
                    DSCho.add(new Node(LC.vertex, label[LC.vertex]));
                    Pred[LC.vertex] = chon.vertex;
                }
            }
        }
    }

    public void lazyPrimMST() {
        boolean[] visited = new boolean[n + 1];
        PriorityQueue<Edge> minHeap = new PriorityQueue<>((e1, e2) -> e1.cost - e2.cost);
        int startVertex = 1; // Start from vertex 1 (or any other starting vertex)
        visited[startVertex] = true;
        for (Edge edge : adjList[startVertex]) {
            minHeap.offer(edge);
        }

        long totalWeight = 0;
        ArrayList<Edge> mstEdges = new ArrayList<>(); // Store the edges in the MST

        while (!minHeap.isEmpty()) {
            Edge minEdge = minHeap.poll();
            int from = minEdge.source;
            int to = minEdge.destination;

            if (visited[from] && visited[to]) {
                continue; // Skip if both vertices are already visited
            }

            // Print the edge being considered
            System.out.println("Considering edge: " + from + " -> " + to + " (Cost: " + minEdge.cost + ")");

            visited[from] = true;
            visited[to] = true;
            totalWeight += minEdge.cost;
            mstEdges.add(minEdge); // Add the edge to the MST

            // Print the current MST
            System.out.println("Current MST Edges:");
            for (Edge edge : mstEdges) {
                System.out.println(edge.source + " -> " + edge.destination + " (Cost: " + edge.cost + ")");
            }

            for (Edge next : adjList[to]) {
                if (!visited[next.destination]) {
                    minHeap.offer(next);
                }
            }

        }

        System.out.println("Total MST Weight: " + totalWeight);
        System.out.println("Final MST Edges:");
        for (Edge edge : mstEdges) {
            System.out.println(edge.source + " -> " + edge.destination + " (Cost: " + edge.cost + ")");
        }
    }

    public void eagerPrimMST() {
        boolean[] inTree = new boolean[n + 1];
        int[] distance = new int[n + 1];
        int[] parent = new int[n + 1];

        for (int i = 1; i <= n; i++) {
            distance[i] = Integer.MAX_VALUE;
        }

        distance[1] = 0;

        PriorityQueue<Integer> pq = new PriorityQueue<>(n, Comparator.comparingInt(v -> distance[v]));

        for (int i = 1; i <= n; i++) {
            pq.add(i);
        }

        while (!pq.isEmpty()) {
            int u = pq.poll();
            inTree[u] = true;

            for (Edge edge : adjList[u]) {
                int v = edge.destination;
                int weight = edge.cost;

                if (!inTree[v] && weight < distance[v]) {
                    distance[v] = weight;
                    parent[v] = u;
                    pq.remove(v);  // Remove and re-add to update priority queue
                    pq.add(v);

                    // Visualization: Print the selected edge
                    System.out.println("Considering edge: " + u + " -> " + v + " (Cost: " + weight + ")");
                    System.out.println("Current MST Edges:");
                    for (int vertex = 2; vertex <= n; vertex++) {
                        if (parent[vertex] != 0) {
                            System.out.println(parent[vertex] + " -> " + vertex + " (Cost: " + distance[vertex] + ")");
                        }
                    }
                }
            }
        }

        int totalWeight = 0;
        List<Edge> mstEdges = new ArrayList<>();
        for (int v = 2; v <= n; v++) {
            totalWeight += distance[v];
            mstEdges.add(new Edge(parent[v], v, distance[v]));
        }
        System.out.println("Total MST Weight: " + totalWeight);
        System.out.println("Final MST Edges:");
        for (Edge edge : mstEdges) {
            System.out.println(edge.source + " -> " + edge.destination + " (Cost: " + edge.cost + ")");
        }
    }
}

    public class Graph_main {

        public static void main(String[] args) throws FileNotFoundException {
            Graph G = new Graph("src\\dijkstra.txt");
            G.Display();
            G.Dijkstra(1, 5);
        //    G.lazyPrimMST();
        //    G.eagerPrimMST();
        }

    }
