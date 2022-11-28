package graphdemo;

/**
 * A test bed for a weighted digraph abstract data type implementation
 * and implementations of elementary classical graph algorithms that use the
 * ADT
 * @see GraphAPI.java, Graph.java, City.java
 * @author Duncan, YOUR NAME
 * <pre>
 * usage: GraphDemo <graphFileName>
 * <graphFileName> - a text file containing the description of the graph
 * in DIMACS file format
 * Date: TYPE LAST DATE MODIFIED
 * course: csc 3102
 * programming project 3
 * Instructor: Dr. Duncan
 * </pre>
 */

import java.io.*;
import java.util.*;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Function;
import java.util.Comparator;
import java.util.PriorityQueue;
import java.util.concurrent.atomic.AtomicBoolean;

public class GraphDemo {
    public static final Double INFINITY = Double.POSITIVE_INFINITY;
    public static final Integer NIL = -1;

    public static void main(String[] args) throws GraphException {
        if (args.length != 1) {
            System.out.println("Usage: GraphDemo <filename>");
            System.exit(1);
        }
        City c1, c2;
        Scanner console;
        int menuReturnValue, i, j;
        Function<City, PrintStream> f = aCity -> System.out.printf("%-2d  %-30s%n", aCity.getKey(),
                aCity.getLabel().trim());
        Graph<City> g = readGraph(args[0]);
        long s = g.size();
        menuReturnValue = -1;
        while (menuReturnValue != 0) {
            menuReturnValue = menu();
            switch (menuReturnValue) {
                case 1: // post-order depth-first-search traversal of g'
                    System.out.println();
                    System.out.println("Breadth-First-Search Traversal of the Graph In " + args[0]);
                    System.out.println("==========================================================================");
                    // invoke the bfsTraverse function
                    // Output should be aligned in two-column format as illustrated below:
                    // 1 Charlottetown
                    // 4 Halifax
                    // 2 Edmonton
                    g.bfsTraverse(f);

                    System.out.println("==========================================================================");
                    System.out.println();
                    System.out.println("Depth-First-Search Traversal of the Graph In " + args[0]);
                    System.out.println("==========================================================================");
                    // invoke the dfsTraverse function
                    // Output should be aligned in two-column format as illustrated below:
                    // 1 Charlottetown
                    // 4 Halifax
                    // 2 Edmonton
                    g.dfsTraverse(f);

                    System.out.println("==========================================================================");
                    System.out.println();
                    System.out.println();
                    break;
                case 2: // Connected Components of G
                    System.out.println();
                    System.out.println("Connected Components in " + args[0]);
                    System.out.println();
                    if (g.size() > 0) {
                        // Add code here to print the list of cities in each component of the graph.
                        // For example:
                        // *** Component # 1 ***
                        // Baton Rouge
                        // Gonzales
                        // Metaire
                        // *** Component # 2 ***
                        // Lafayette
                        // Independence
                        // *** Component # 3 ***
                        // Baker
                        // Eunice
                        // Franklin
                        // --------------------------
                        // Number of Components: 3

                        int[] components = new int[(int) g.size() + 1];
                        int numComponents = getComponents(g, components);
                        int n = 1;
                        while (n <= numComponents) {
                            System.out.printf("*** Component # %d ***\n", n);
                            for (int num = 1; num < components.length; num++) {
                                if (components[num] == n) {
                                    System.out.println(g.retrieveVertex(new City(num)).getLabel().trim());
                                }
                            }
                            n++;
                        }
                        System.out.println("------------------------");
                        System.out.println("Number of Components: " + numComponents);

                        // End code
                    } else
                        System.out.println("The graph has no connected component.");
                    System.out.println();
                    System.out.println();
                    System.out.println();
                    break;
                case 3:// Shortest-path algorithm
                    console = new Scanner(System.in);
                    System.out.printf("Enter the source vertex: ");
                    i = console.nextInt();
                    System.out.printf("Enter the destination vertex: ");
                    j = console.nextInt();
                    if (g.isPath(new City(i), new City(j)) && g.isPath(new City(j), new City(i))) {
                        int dest;
                        double[][] cost = new double[(int) g.size()][(int) g.size()];
                        int[][] path = new int[(int) g.size()][(int) g.size()];
                        int initial = i;
                        floyd(g, cost, path);
                        System.out.printf("Shortest round trip from %s to %s:%n",
                                g.retrieveVertex(new City(i)).getLabel().trim(),
                                g.retrieveVertex(new City(j)).getLabel().trim());
                        System.out.println(
                                "=========================================================================================");
                        // Add code here to print each leg of the trip from the source to the
                        // destination
                        // using the format below, where the columns are left-aligned and the distances
                        // are displayed to the nearest hundredths.
                        // For example:
                        // Baton Rouge -> New Orleans:
                        // Baton Rouge -> Gonzales 10.20 mi
                        // Gonzales -> Metaire 32.00 mi
                        // Metaire -> New Orleans 7.25 mi
                        // Distance: 49.75 mi
                        //
                        // New Orleans -> Baton Rouge
                        // New Orleans -> Metaire 8.00 mi
                        // Metaire -> Gonzales 33.00 mi
                        // Gonzalen -> Baton Rouge 10.00 mi
                        // Distance: 51.00 mi
                        //

                        System.out.printf("%s -> %s:\n", g.retrieveVertex(new City(i)).getLabel().trim(),
                                g.retrieveVertex(new City(j)).getLabel().trim());
                        int first = i;
                        int last = path[i - 1][j - 1];
                        double totalDist = 0;
                        while (first != last) {
                            City fCity1 = new City(first);
                            City fCity2 = new City(last);
                            System.out.printf("%s -> %s %.2f mi\n", g.retrieveVertex(fCity1).getLabel().trim(),
                                    g.retrieveVertex(fCity2).getLabel().trim(), cost[first - 1][last - 1]);
                            totalDist += cost[first - 1][last - 1];
                            first = last;
                            last = path[first - 1][j - 1];
                        }
                        System.out.printf("Distance: %.2f mi\n\n", totalDist);

                        System.out.printf("%s -> %s:\n", g.retrieveVertex(new City(j)).getLabel().trim(),
                                g.retrieveVertex(new City(i)).getLabel().trim());
                        first = j;
                        last = path[j - 1][i - 1];
                        totalDist = 0;
                        while (first != last) {
                            City fCity1 = new City(first);
                            City fCity2 = new City(last);
                            System.out.printf("%s -> %s %.2f mi\n", g.retrieveVertex(fCity1).getLabel().trim(),
                                    g.retrieveVertex(fCity2).getLabel().trim(), cost[first - 1][last - 1]);
                            totalDist += cost[first - 1][last - 1];
                            first = last;
                            last = path[first - 1][i - 1];
                        }
                        System.out.printf("Distance: %.2f mi\n", totalDist);

                        // End code
                        System.out.println(
                                "=========================================================================================");
                        System.out.printf("Round Trip Distance: %.2f miles.%n%n",
                                cost[i - 1][j - 1] + cost[j - 1][i - 1]);
                    } else
                        System.out.printf("There is no path.%n%n");
                    break;
                case 4: // Check whether g is bipartite
                    System.out.println();
                    AtomicBoolean bip = new AtomicBoolean(true);
                    int[] part = isBipartite(g, bip);
                    if (bip.get()) {
                        if (g.size() == 0)
                            System.out.println("An empty graph is vacuously bipartite.");
                        else {
                            System.out.printf("The Graph is Bipartite.%n%n");
                            System.out.println("First Partition: ");
                            for (i = 1; i <= g.size(); i++) {
                                if (part[i] == 1)
                                    System.out.printf("%d. %s%n ", i, g.retrieveVertex(new City(i)).getLabel().trim());
                            }
                            System.out.println();
                            System.out.println("Second Partition: ");
                            int k = 0;
                            for (i = 1; i <= g.size(); i++) {
                                if (part[i] == 0) {
                                    System.out.printf("%d. %s%n ", i, g.retrieveVertex(new City(i)).getLabel().trim());
                                    k++;
                                }
                            }
                            if (k == 0)
                                System.out.println("EMPTY");
                        }
                    } else
                        System.out.println("The graph is not bipartite.");
                    System.out.println();
                    break;
                case 5: // primMST;
                    int edgesInMST = 0;
                    System.out.printf("Enter the root of the MST: ");
                    console = new Scanner(System.in);
                    j = console.nextInt();
                    int[] mst = new int[(int) g.size()];
                    double totalWt = primMST(g, j, mst);
                    String cityNameA, cityNameB;
                    System.out.println();
                    for (i = 1; i <= g.size(); i++) {
                        if (mst[i - 1] < 1)
                            cityNameA = "NONE";
                        else {
                            edgesInMST++;
                            cityNameA = g.retrieveVertex(new City(mst[i - 1])).getLabel().trim();
                        }
                        cityNameB = g.retrieveVertex(new City(i)).getLabel().trim();
                        System.out.printf("%d-%s parent[%d] <- %d (%s)%n", i, cityNameB, i, mst[i - 1], cityNameA);
                    }
                    System.out.printf("The weight of the minimum spanning tree/forest is %.2f miles.%n", totalWt);
                    System.out.printf("Spanning Tree Edge Density is %.2f%%.%n%n", 100.0 * edgesInMST / g.countEdges());
                    break;
                default:
                    ;
            } // end switch
        } // end while
    }// end main

    /**
     * This method reads a text file formatted as described in the project
     * description.
     * 
     * @param filename the name of the DIMACS formatted graph file.
     * @return an instance of a graph.
     */
    private static Graph<City> readGraph(String filename) {
        try {
            Graph<City> newGraph = new Graph();
            try (FileReader reader = new FileReader(filename)) {
                char temp;
                City c1, c2, aCity;
                String tmp;
                int k, m, v1, v2, j, size = 0, nEdges = 0;
                Integer key, v1Key, v2Key;
                Double weight;
                Scanner in = new Scanner(reader);
                while (in.hasNext()) {
                    tmp = in.next();
                    temp = tmp.charAt(0);
                    if (temp == 'p') {
                        size = in.nextInt();
                        nEdges = in.nextInt();
                    } else if (temp == 'c') {
                        in.nextLine();
                    } else if (temp == 'n') {
                        key = in.nextInt();
                        tmp = in.nextLine();
                        aCity = new City(key, tmp);
                        newGraph.insertVertex(aCity);
                    } else if (temp == 'e') {
                        v1Key = in.nextInt();
                        v2Key = in.nextInt();
                        weight = in.nextDouble();
                        c1 = new City(v1Key);
                        c2 = new City(v2Key);
                        newGraph.insertEdge(c1, c2, weight);
                    }
                }
            }
            return newGraph;
        } catch (IOException exception) {
            System.out.println("Error processing file: " + exception);
        }
        return null;
    }

    /**
     * Display the menu interface for the application.
     * 
     * @return the menu option selected.
     */
    private static int menu() {
        Scanner console = new Scanner(System.in);
        // int option;
        String option;
        do {
            System.out.println("  BASIC WEIGHTED GRAPH APPLICATION   ");
            System.out.println("=======================================================");
            System.out.println("[1] BFS/DFS Traversal of G");
            System.out.println("[2] Connected Components of G");
            System.out.println("[3] Floyd's Shortest Round Trip in G");
            System.out.println("[4] Check whether G is Bipartite");
            System.out.println("[5] Prim's Minimum Spanning Tree/Forest in G");
            System.out.println("[0] Quit");
            System.out.println("=====================================");
            System.out.printf("Select an option: ");
            option = console.nextLine().trim();
            try {
                int choice = Integer.parseInt(option);
                if (choice < 0 || choice > 7) {
                    System.out.println("Invalid option...Try again");
                    System.out.println();
                } else
                    return choice;
            } catch (NumberFormatException e) {
                System.out.println("Invalid option...Try again");
            }
        } while (true);
    }

    /**
     * Determines whether an undirected graph is bipartite
     * 
     * @param g         an undirected graph
     * @param bipartite is true if g is bipartite, otherwise false
     * @return an array of size |G|+1. The first entry is |G| and the remaining
     *         entries are in {0,1} when g is bipartite where [i] = 0 if vertex i
     *         is in the first partition and 1 if it is in the second partition;
     *         if g is not bipartite NULL returned.
     */
    private static int[] isBipartite(Graph<City> g, AtomicBoolean bipartite) throws GraphException {
        // Implement this method

        bipartite.set(true);
        int[] part = new int[(int) (g.size() + 1)];
        for (int i = 0; i < part.length; i++) {
            part[i] = NIL;
        }
        for (int i = 1; i < g.size() - 1; i++) {
            if (part[i] == NIL) {
                part[i] = 0;
            }
            for (int j = i + 1; j < g.size(); j++) {
                if (g.isEdge(new City(i), new City(j))) {
                    if (part[i] == part[j]) {
                        bipartite.set(false);
                        return null;
                    }
                    if (part[j] == NIL) {
                        part[j] = (part[i] + 1) % 2;
                    }
                }
            }
        }
        return part;
    }

    /**
     * This method computes the cost and path matrices using the
     * Floyd all-pairs shortest path algorithm.
     * 
     * @param g    an instance of a weighted directed graph.
     * @param dist a matrix containing distances between pairs of vertices.
     * @param path a matrix of intermediate vertices along the path between a pair
     *             of vertices. 0 indicates that the two vertices are adjacent.
     * @return none.
     */
    private static void floyd(Graph<City> g, double dist[][], int path[][]) throws GraphException {
        // implement this method.
        // S = dist
        // P = path
        for (int i = 1; i <= g.size(); i++) {
            for (int j = 1; j <= g.size(); j++) {
                if (i == j) {
                    path[i - 1][j - 1] = j;
                    dist[i - 1][j - 1] = 0;
                } else if (g.isEdge(new City(i), new City(j))) {
                    path[i - 1][j - 1] = j;
                    dist[i - 1][j - 1] = g.retrieveEdge(new City(i), new City(j));
                } else {
                    path[i - 1][j - 1] = NIL;
                    dist[i - 1][j - 1] = INFINITY;
                }
            }
        }
        for (int i = 1; i <= g.size(); i++) {
            path[i - 1][i - 1] = i;
            dist[i - 1][i - 1] = 0;
        }
        for (int k = 1; k <= g.size(); k++) {
            for (int i = 1; i <= g.size(); i++) {
                for (int j = 1; j <= g.size(); j++) {
                    if (dist[i - 1][j - 1] > dist[i - 1][k - 1] + dist[k - 1][j - 1] && dist[i - 1][k - 1] != INFINITY
                            && dist[k - 1][j - 1] != INFINITY) {
                        path[i - 1][j - 1] = path[i - 1][k - 1];
                        dist[i - 1][j - 1] = dist[i - 1][k - 1] + dist[k - 1][j - 1];
                    }
                }
            }
        }
    }

    /**
     * This method generates a minimum spanning tree rooted at a given
     * vertex, root. If no such MST exists, then it generates a minimum
     * spanning forest.
     * 
     * @param g      a weighted undirected graph
     * @param root   root of the minimum spanning tree, when one exists.
     * @param parent the parent implementation of the minimum spanning tree/forest
     * @return the weight of such a tree or forest.
     * @throws GraphException when this graph is empty
     * 
     *                        <pre>
     * {@code
     * If a minimum spanning tree rooted at r is in the graph,
     * the parent implementation of a minimum spanning tree or forest is
     * determined. If no such tree exist, the parent implementation 
     * of an MSF is generated. If the tree is empty, an exception 
     * is generated.
     * }
     * </pre>
     */
    private static double primMST(Graph<City> g, int root, int[] parent) throws GraphException {
        // implement this method
        int i;
        if (g.isEmpty())
            throw new GraphException("Empty graph in call to primMST()");
        if (!g.isVertex(new City(root)))
            throw new GraphException("Non-existent root in call to primMST()");
        int numVertices = (int) g.size();
        double[] dist = new double[numVertices];
        boolean[] processed = new boolean[numVertices + 1];
        for (i = 0; i < numVertices; i++) {
            dist[i] = INFINITY;
            parent[i] = -1;
            processed[i] = false;
        }
        dist[root - 1] = 0;
        double totalWeight = 0;
        class Node {
            public int parent, id;
            public double key;

            public Node() {

            }

            public Node(int p, int v, double k) {
                parent = p;
                id = v;
                key = k;
            }
        }

        Comparator<Node> cmp = (v1, v2) -> {
            double d = v1.key - v2.key;
            if (d < 0)
                return -1;
            if (d > 0)
                return 1;
            d = v1.id - v2.id;
            if (d < 0)
                return -1;
            if (d > 0)
                return 1;
            return 0;
        };
        // Define an instance of the PriorityQueue class that uses the comparator;
        // Then implement the priority-queue-based Prim's MST algorithm

        PriorityQueue<Node> pq = new PriorityQueue<>(cmp);
        int vCount = 0;
        int[] msTree = new int[numVertices];
        for (i = 0; i < msTree.length; i++) {
            msTree[i] = NIL;
        }

        // put one node object for each vertex of the graph in the pq
        for (i = 0; i < numVertices; i++) {
            pq.add(new Node(NIL, i, INFINITY));
        }
        // add a node object for root to pq so root is added to the pq first
        pq.add(new Node(NIL, root, 0));

        // when all the vertices are added to the spanning tree/forest the
        // algorithm terminates
        while (vCount < numVertices) {
            Node u = pq.poll();
            while (processed[u.id] == true) {
                u = pq.poll();
            }
            vCount++;
            msTree[u.id - 1] = u.parent;

            // if this vertex is a root, its distance has to be 0 since it is the first
            // vertex added to the tree
            if (msTree[u.id - 1] == -1) {
                u.key = 0;
            }
            dist[u.id] = u.key;
            // update the weight of the spanning tree forest by the weight of the edge
            // used to connect u to the tree
            totalWeight += u.key;

            // update all the neighbors of u that are not already in the tree
            for (int v = 1; v < numVertices; v++) {
                City c1 = new City(u.id);
                City c2 = new City(v);
                if (g.isEdge(c1, c2) && !processed[v]) {
                    if (g.retrieveEdge(c1, c2) < dist[v]) {
                        dist[v] = g.retrieveEdge(c1, c2);
                        msTree[v - 1] = u.id;
                        pq.add(new Node(u.id, v, dist[v]));
                    }
                } else if (g.isEdge(c2, c1) && !processed[v]) {
                    if (g.retrieveEdge(c2, c1) < dist[v]) {
                        dist[v] = g.retrieveEdge(c2, c1);
                        msTree[v - 1] = u.id;
                        pq.add(new Node(u.id, v, dist[v]));
                    }
                }
            }
        }
        for (i = 0; i < numVertices; i++) {
            parent[i] = msTree[i];
        }
        return totalWeight;
    }

    /**
     * Generates the connected components of this graph
     * 
     * @param g          a graph of city objects
     * @param components an associative array of city
     *                   key to component number: if components[i] = k, then
     *                   city i+1 is in component k.
     * @return the number of components in this graph
     * @throws GraphException
     */
    private static int getComponents(Graph<City> g, int components[]) throws GraphException {
        // Implement this method

        /* initialize all elements in the array to 0 */
        for (int i = 0; i < components.length - 1; i++) {
            components[i] = 0;
        }
        /* start counting with component 1 */
        int count = 0;
        /* look through all the vertices in the graph */
        for (int i = 1; i < components.length; i++) {
            City startCity = new City(i);
            /* set the vertex index to the current component # as long as it is 0 */
            if (components[i] == 0) {
                count++;
                components[i] = count;
            }
            /* loop through the other vertices in the array */
            for (int j = i + 1; j < components.length; j++) {
                City endCity = new City(j);
                /* if path between (i, j) + the index is not set, set to current component # */
                if ((g.isPath(startCity, endCity) || g.isPath(endCity, startCity)) && components[j] == 0) {
                    components[j] = count;
                }
                /* if path between (i, j) + the index is set, readjust to current component # */
                else if ((g.isPath(startCity, endCity) || g.isPath(endCity, startCity)) && components[j] != 0) {
                    components[i] = components[j];
                    /* makes sure component # stays on track */
                    count = components[i];
                }
            }
        }
        return count;

    }
}
