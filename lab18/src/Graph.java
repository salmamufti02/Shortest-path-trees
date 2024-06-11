import java.util.*;

public class Graph implements Iterable<Integer> {

    private LinkedList<Edge>[] adjLists;
    private int vertexCount;
    int[] distances = new int[vertexCount];

    /* Initializes a graph with NUMVERTICES vertices and no Edges. */
    public Graph(int numVertices) {
        adjLists = (LinkedList<Edge>[]) new LinkedList[numVertices];
        for (int k = 0; k < numVertices; k++) {
            adjLists[k] = new LinkedList<Edge>();
        }
        vertexCount = numVertices;
        distances = new int[vertexCount];
    }

    /* Adds a directed Edge (V1, V2) to the graph. That is, adds an edge
       in ONE directions, from v1 to v2. */
    public void addEdge(int v1, int v2) {
        addEdge(v1, v2, 0);
    }

    /* Adds an undirected Edge (V1, V2) to the graph. That is, adds an edge
       in BOTH directions, from v1 to v2 and from v2 to v1. */
    public void addUndirectedEdge(int v1, int v2) {
        addUndirectedEdge(v1, v2, 0);
    }

    /* Adds a directed Edge (V1, V2) to the graph with weight WEIGHT. If the
       Edge already exists, replaces the current Edge with a new Edge with
       weight WEIGHT. */
    public void addEdge(int v1, int v2, int weight) {
        Edge edge = new Edge(v1, v2, weight);
        adjLists[v1].add(edge);
    }
    public Edge getEdge(int u, int v) {
        for (Edge edge : adjLists[u]) {
            if (edge.to == v) {
                return edge;
            }
        }
        return null;
    }


    /* Adds an undirected Edge (V1, V2) to the graph with weight WEIGHT. If the
       Edge already exists, replaces the current Edge with a new Edge with
       weight WEIGHT. */
    public void addUndirectedEdge(int v1, int v2, int weight) {
        addEdge(v1, v2, weight);
        addEdge(v2, v1, weight);
    }

    /* Returns true if there exists an Edge from vertex FROM to vertex TO.
       Returns false otherwise. */
    public boolean isAdjacent(int from, int to) {
        for (Edge edge : adjLists[from]) {
            if (edge.to == to) {
                return true;
            }
        }
        return false;
    }

    /* Returns a list of all the vertices u such that the Edge (V, u)
       exists in the graph. */
    public List<Integer> neighbors(int v) {
        List<Integer> neighbors = new ArrayList<>();
        for (Edge edge : adjLists[v]) {
            neighbors.add(edge.to);
        }
        return neighbors;
    }

    /* Returns the number of incoming Edges for vertex V. */
    public int inDegree(int v) { //number of edges for vertex v
        int inDegree = 0;
        for (int i = 0; i < vertexCount; i++) {
            for (Edge edge : adjLists[i]) {
                if (edge.to == v) {
                    inDegree++;
                }
            }

        }
        return inDegree;
    }

    /* Returns an Iterator that outputs the vertices of the graph in topological
       sorted order. */
    public Iterator<Integer> iterator() {
        return new TopologicalIterator();
    }

    /**
     * A class that iterates through the vertices of this graph,
     * starting with a given vertex. Does not necessarily iterate
     * through all vertices in the graph: if the iteration starts
     * at a vertex v, and there is no path from v to a vertex w,
     * then the iteration will not include w.
     */
    private class DFSIterator implements Iterator<Integer> {

        private Stack<Integer> fringe;
        private HashSet<Integer> visited;

        public DFSIterator(Integer start) {
            fringe = new Stack<>();
            visited = new HashSet<>();
            fringe.push(start);
        }

        public boolean hasNext() {
            if (!fringe.isEmpty()) {
                int i = fringe.pop();
                while (visited.contains(i)) {
                    if (fringe.isEmpty()) {
                        return false;
                    }
                    i = fringe.pop();
                }
                fringe.push(i);
                return true;
            }
            return false;
        }

        public Integer next() {
            int curr = fringe.pop();
            visited.add(curr);
            for (int neighbor : neighbors(curr)) {
                if (!visited.contains(neighbor)) {
                    fringe.push(neighbor);
                }
            }
            return curr;
        }

        //ignore this method
        public void remove() {
            throw new UnsupportedOperationException(
                    "vertex removal not implemented");
        }

    }

    /* Returns the collected result of performing a depth-first search on this
       graph's vertices starting from V. */
    public List<Integer> dfs(int v) {
        ArrayList<Integer> result = new ArrayList<Integer>();
        Iterator<Integer> iter = new DFSIterator(v);

        while (iter.hasNext()) {
            result.add(iter.next());
        }
        return result;
    }

    /* Returns true iff there exists a path from START to STOP. Assumes both
       START and STOP are in this graph. If START == STOP, returns true. */
    public boolean pathExists(int start, int stop) {
        Iterator<Integer> iter = new DFSIterator(start);
        while (iter.hasNext()) {
            int currentVertex = iter.next();
            if (currentVertex == stop) {
                return true;
            }
        }
        return false;
    }


    /* Returns the path from START to STOP. If no path exists, returns an empty
       List. If START == STOP, returns a List with START. */
    public List<Integer> path(int start, int stop) {
        List<Integer> result = new ArrayList<>();
        Iterator<Integer> iter = new DFSIterator(start);
        HashSet<Integer> visited = new HashSet<>();

        while (iter.hasNext()) {
            int currentVertex = iter.next();
            visited.add(currentVertex);
            if (currentVertex == stop) {
                break;
            }
        }
        if (!visited.contains(stop)) {
            return result;
        }
        int currentVertex = stop;
        result.add(currentVertex);
        while (currentVertex != start) {
            for (int neighbor : neighbors(currentVertex)) {
                if (visited.contains(neighbor) && isAdjacent(neighbor, currentVertex)) {
                    result.add(neighbor);
                    currentVertex = neighbor;
                    break;
                }
            }
        }
        Collections.reverse(result);
        return result;
    }

    public List<Integer> topologicalSort() {
        ArrayList<Integer> result = new ArrayList<Integer>();
        Iterator<Integer> iter = new TopologicalIterator();
        while (iter.hasNext()) {
            result.add(iter.next());
        }
        return result;
    }

    private class TopologicalIterator implements Iterator<Integer> {

        private Stack<Integer> fringe;

        private int[] currentInDegree;


        TopologicalIterator() {
            fringe = new Stack<>();
            currentInDegree = new int[vertexCount];
            for (int v = 0; v <vertexCount; v ++) {
                if  (currentInDegree[v] == 0) {
                    fringe.push(v);
                }
            }
        }

        public boolean hasNext() {
            return !fringe.isEmpty();
        }

        public Integer next() {
            if (!hasNext()) {
                throw new NoSuchElementException();
            }
            return fringe.pop();
        }

        public void remove() {
            throw new UnsupportedOperationException();
        }

    }

    private class Edge {

        private int from;
        private int to;
        private int weight;

        Edge(int from, int to, int weight) {
            this.from = from;
            this.to = to;
            this.weight = weight;
        }

        public String toString() {
            return "(" + from + ", " + to + ", weight = " + weight + ")";
        }

    }

    private void generateG1() {
        addEdge(0, 1);
        addEdge(0, 2);
        addEdge(0, 4);
        addEdge(1, 2);
        addEdge(2, 0);
        addEdge(2, 3);
        addEdge(4, 3);
    }

    private void generateG2() {
        addEdge(0, 1);
        addEdge(0, 2);
        addEdge(0, 4);
        addEdge(1, 2);
        addEdge(2, 3);
        addEdge(4, 3);
    }

    private void generateG3() {
        addUndirectedEdge(0, 2);
        addUndirectedEdge(0, 3);
        addUndirectedEdge(1, 4);
        addUndirectedEdge(1, 5);
        addUndirectedEdge(2, 3);
        addUndirectedEdge(2, 6);
        addUndirectedEdge(4, 5);
    }

    private void generateG4() {
        addEdge(0, 1);
        addEdge(1, 2);
        addEdge(2, 0);
        addEdge(2, 3);
        addEdge(4, 2);
    }

    private void printDFS(int start) {
        System.out.println("DFS traversal starting at " + start);
        List<Integer> result = dfs(start);
        Iterator<Integer> iter = result.iterator();
        while (iter.hasNext()) {
            System.out.println(iter.next() + " ");
        }
        System.out.println();
        System.out.println();
    }

    private void printPath(int start, int end) {
        System.out.println("Path from " + start + " to " + end);
        List<Integer> result = path(start, end);
        if (result.size() == 0) {
            System.out.println("No path from " + start + " to " + end);
            return;
        }
        Iterator<Integer> iter = result.iterator();
        while (iter.hasNext()) {
            System.out.println(iter.next() + " ");
        }
        System.out.println();
        System.out.println();
    }

    private void printTopologicalSort() {
        System.out.println("Topological sort");
        List<Integer> result = topologicalSort();
        Iterator<Integer> iter = result.iterator();
        while (iter.hasNext()) {
            System.out.println(iter.next() + " ");
        }
    }
    public List <Integer> shortestPath (int start, int stop) {
        PriorityQueue<int[]> fringe = new PriorityQueue<>(Comparator.comparingInt(arr ->arr[1]));
        int[] distances = new int[vertexCount];
        Arrays.fill(distances, Integer.MAX_VALUE);
        distances[start] = 0;

        int[] predecessors = new int[vertexCount];
        Arrays.fill(predecessors, -1);

        fringe.offer(new int[]{start, 0});

        while (!fringe.isEmpty()) {
            int[] current = fringe.poll();
            int currentVertex = current[0];
            int currentDistance = current[1];

            if (currentDistance > distances[currentVertex]) {
                continue;
            }
            for (Edge edge : adjLists[currentVertex]) {
                int neighbor = edge.to;
                int newDistance = currentDistance + edge.weight;
                    if (newDistance < distances[neighbor]) {

                        distances[neighbor] = newDistance;
                        predecessors[neighbor] = currentVertex;
                        fringe.offer(new int[]{neighbor, newDistance});
                    }}}

            List<Integer> path = new ArrayList<>();
            int vertex = stop;
            while (vertex != -1) {
                path.add(0, vertex);
                vertex = predecessors[vertex];
            }
            if (path.get(0) != start) {
                return new ArrayList<>();
            }
            return path;

        }

    public static void main(String[] args) {
        Graph g1 = new Graph(5);
        g1.generateG1();
        g1.printDFS(0);
        g1.printDFS(2);
        g1.printDFS(3);
        g1.printDFS(4);

        g1.printPath(0, 3);
        g1.printPath(0, 4);
        g1.printPath(1, 3);
        g1.printPath(1, 4);
        g1.printPath(4, 0);

        Graph g2 = new Graph(5);
        g2.generateG2();
        g2.printTopologicalSort();
    }
}