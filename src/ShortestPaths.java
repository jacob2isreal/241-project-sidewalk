import java.util.Map;
import java.util.HashMap;
import java.util.LinkedList;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.PriorityQueue; 

/** Provides an implementation of Dijkstra's single-source shortest paths
 * algorithm.
 * Sample usage:
 *   Graph g = // create your graph
 *   ShortestPaths sp = new ShortestPaths();
 *   Node a = g.getNode("A");
 *   sp.compute(a);
 *   Node b = g.getNode("B");
 *   LinkedList<Node> abPath = sp.getShortestPath(b);
 *   double abPathLength = sp.getShortestPathLength(b);
 *   */
public class ShortestPaths {
    // stores auxiliary data ass    ociated with each node for the shortest
    // paths computation:
    private HashMap<Node,PathData> paths;

    /** Compute the shortest path to all nodes from origin using Dijkstra's
     * algorithm. Fill in the paths field, which associates each Node with its
     * PathData record, storing total distance from the source, and the
     * back pointer to the previous node on the shortest path.
     * Precondition: origin is a node in the Graph.*/
    public void compute(Node origin) {
        HashMap<Node, Boolean> visited = new HashMap<>();
        paths = new HashMap<Node,PathData>();
        paths.put(origin, new PathData(0.0, null));
        PriorityQueue<Node> queue = new PriorityQueue<>(); 
        queue.add(origin);

        while(!queue.isEmpty()){
            Node n = queue.remove();
            PathData data = paths.get(n);
            if(!visited.containsKey(n))

            if (visited.containsKey(n)) {
                continue;
            }

            visited.put(n, true);

            PathData nData = paths.get(n);
            double nDist = data.distance;

            for(HashMap.Entry<Node, Double> edge : n.getNeighbors().entrySet()){
                Node n1 = edge.getKey();
                double weight = edge.getValue();
                double newDist = nDist + weight; 
                PathData n1Data = paths.get(n1);
                double n1Dist = (n1Data == null) ? Double.POSITIVE_INFINITY : n1Data.distance;

                if(newDist < n1Dist){
                    paths.put(n1, new PathData(newDist, n));
                    queue.add(n1);
                }

            }

        }
    }

    /** Returns the length of the shortest path from the origin to destination.
     * If no path exists, return Double.POSITIVE_INFINITY.
     * Precondition: destination is a node in the graph, and compute(origin)
     * has been called. */
    public double shortestPathLength(Node destination) {
        PathData data = paths.get(destination);
        if(data != null){
            return data.distance;
        } else {
            return Double.POSITIVE_INFINITY;
        }
    }

    /** Returns a LinkedList of the nodes along the shortest path from origin
     * to destination. This path includes the origin and destination. If origin
     * and destination are the same node, it is included only once.
     * If no path to it exists, return null.
     * Precondition: destination is a node in the graph, and compute(origin)
     * has been called. */
    public LinkedList<Node> shortestPath(Node destination) {
        LinkedList<Node> path = new LinkedList<>();
        Node n = destination;
        while(n != null){
            PathData data = paths.get(n);
            
            if(data == null){
                return null;
            }
            path.addFirst(n);
            n = data.previous;
        }
        if(path.isEmpty()){
            return null;
        }
        return path;
    }


    /** Inner class representing data used by Dijkstra's algorithm in the
     * process of computing shortest paths from a given source node. */
    class PathData {
        double distance; // distance of the shortest path from source
        Node previous; // previous node in the path from the source

        /** constructor: initialize distance and previous node */
        public PathData(double dist, Node prev) {
            distance = dist;
            previous = prev;
        }
    }

    /** Static helper method to open and parse a file containing graph
     * information. Can parse either a basic file or a CSV file with
     * sidewalk data. See GraphParser, BasicParser, and DBParser for more.*/
    protected static Graph parseGraph(String fileType, String fileName) throws
        FileNotFoundException {
        // create an appropriate parser for the given file type
        GraphParser parser;
        if (fileType.equals("basic")) {
            parser = new BasicParser();
        } else if (fileType.equals("db")) {
            parser = new DBParser();
        } else {
            throw new IllegalArgumentException(
                    "Unsupported file type: " + fileType);
        }

        // open the given file
        parser.open(new File(fileName));

        // parse the file and return the graph
        return parser.parse();
    }

    public static void main(String[] args) {
      // read command line args
      String fileType = args[0];
      String fileName = args[1];
      String SidewalkOrigCode = args[2];

      String SidewalkDestCode = null;
      if (args.length == 4) {
        SidewalkDestCode = args[3];
      }

      // parse a graph with the given type and filename
      Graph graph;
      try {
          graph = parseGraph(fileType, fileName);
      } catch (FileNotFoundException e) {
          System.out.println("Could not open file " + fileName);
          return;
      }
      graph.report();


      // TODO 4: create a ShortestPaths object, use it to compute shortest
      // paths data from the origin node given by origCode.
      ShortestPaths sp = new ShortestPaths(); 
      if (origin != null) {
          sp.compute(origin);
      }
      // TODO 5:
      // If destCode was not given, print each reachable node followed by the
      // length of the shortest path to it from the origin.
      // TODO 6:
      // If destCode was given, print the nodes in the path from
      // origCode to destCode, followed by the total path length
      // If no path exists, print a message saying so.
      if (SidewalkDestCode == null) {
          for (Node node : graph.getNodes()) {
              double dist = sp.shortestPathLength(node);
              if (dist != Double.POSITIVE_INFINITY) {
                  System.out.println(node + ": " + dist);
              }
          }
      } else { 
        Node d = graph.getNode(SidewalkDestCode);
        if(d == null){
            System.out.println("Destination not found: " + SidewalkDestCode);
            return;
        }
        LinkedList<Node> path = sp.shortestPath(d);
        if(path == null){
            System.out.println("No path found from " + SidewalkOrigCode + " to " + SidewalkDestCode);
        } else {
            for (Node n : path){
                System.out.println(n);
            }
            System.out.println("Path length: " + sp.shortestPathLength(d));
        }
      }
    }
}
