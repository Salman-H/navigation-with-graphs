/**
 * @author Salman Hashmi
 * @author UCSD MOOC development team
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;

import java.util.List;
import java.util.ListIterator;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;

import geography.GeographicPoint;
import util.GraphLoader;


/**
 * @author Salman Hashmi
 * @author UCSD MOOC development team
 * 
 *         A class which represents a graph of geographic locations/coordinates of
 *         road intersections which are the nodes in the graph defined by the MapNode class
 *
 */
public class MapGraph {
	// TODO: Add your member variables here in WEEK 2
	private HashMap<GeographicPoint, MapNode> graphNodesHashMap;
	private int numNodes;
	private int numEdges;

	/**
	 * Create a new empty MapGraph
	 */
	public MapGraph() {
		// TODO: Implement in this constructor in WEEK 2
		graphNodesHashMap = new HashMap<GeographicPoint, MapNode>();
		numNodes = 0;
		numEdges = 0;
	}

	/**
	 * Get the number of vertices (road intersections) in the graph
	 * 
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices() {
		// TODO: Implement this method in WEEK 2
		return numNodes;
	}

	/**
	 * Return the intersections, which are the vertices in this graph.
	 * 
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices() {
		// TODO: Implement this method in WEEK 2
		return graphNodesHashMap.keySet();
	}

	/**
	 * Get the number of road segments in the graph
	 * 
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges() {
		// TODO: Implement this method in WEEK 2
		return numEdges;
	}

	/**
	 * Add a node corresponding to an intersection at a Geographic Point If the
	 * location is already in the graph or null, this method does not change the
	 * graph.
	 * 
	 * @param location
	 *            The location of the intersection
	 * @return true if a node was added, false if it was not (the node was
	 *         already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location) {
		// TODO: Implement this method in WEEK 2
		if (graphNodesHashMap.containsKey(location) || location == null) return false;
		// otherwise add the node
		graphNodesHashMap.put(location, new MapNode(location));
		// and increment node count
		numNodes++;
		return true;
	}

	/**
	 * Adds a directed edge to the graph from pt1 to pt2. Precondition: Both
	 * GeographicPoints have already been added to the graph
	 * 
	 * @param from
	 *            The starting point of the edge
	 * @param to
	 *            The ending point of the edge
	 * @param roadName
	 *            The name of the road
	 * @param roadType
	 *            The type of the road
	 * @param length
	 *            The length of the road, in km
	 * @throws IllegalArgumentException
	 *             If the points have not already been added as nodes to the
	 *             graph, if any of the arguments is null, or if the length is
	 *             less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double length)
			throws IllegalArgumentException {

		// TODO: Implement this method in WEEK 2
		if ( from == null || to == null || roadName == null || roadType == null
				|| !graphNodesHashMap.containsKey(from) || !graphNodesHashMap.containsKey(to) 
				|| length < 0) {
			throw new IllegalArgumentException();
		}
		// create new edge
		MapEdge newEdge = new MapEdge(from, to, roadName, roadType, length);
		
		// associate the edge with the MapNode with location == from
		MapNode mapNode = graphNodesHashMap.get(from);
		mapNode.addEdgeToNode(newEdge);
		numEdges++;
	}
	
	/**
	 * Return a String representation of the graph
	 * 
	 * @return A string representation of the graph
	 */
	public String toString() {
		
		String s = "\nGraph with " + numNodes + " vertices and " + numEdges + " edges.\n";
		s += "\n\t" + graphNodesHashMap.values();
		return s;
	}

	/**
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *         path from start to goal (including both start and goal).
	 */
	
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return bfs(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *         path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		// TODO: Implement this method in WEEK 2

		// Hook for visualization. See writeup.
		// nodeSearched.accept(next.getLocation());
		
		System.out.println("");
		System.out.println(" *** bfs ***");
		System.out.println(" start: " + start);
		System.out.println(" goal: " + goal);
		
		if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return null;
		}
		
		// the List to be returned outlining the desired path is constructed from this HashMap
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		
		// call the bfsSearch helper method to perform the actual search
		boolean found = bfsSearch(start, goal, parentMap, nodeSearched);
		
		// if no path found, return null
		if (!found) {
			System.out.println("No path exists");
			return null;
		}
		
		// reconstruct the path
		return constructPath(start, goal, parentMap);
	}
	
	/**
	 * Constructs the found path from the HashMap parentMap 
	 * 
	 * @param start:		The starting location
	 * @param goal:			The goal location
	 * @param parentMap:	The HashMap that keeps track of the path followed during the search
	 * @return:				The List of nodes as Geographic Point objects that represent the found path
	 */
	private List<GeographicPoint> constructPath(GeographicPoint start, GeographicPoint goal, HashMap<GeographicPoint, GeographicPoint> parentMap) {
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		GeographicPoint curr = goal;
		while (curr != start) {
			path.addFirst(curr);
			curr = parentMap.get(curr);
		}
		path.addFirst(start);
		return path;
	}
	
	/**
	 * Perform the actual BFS search and fill the HashMap parentMap along the way
	 * 
	 * @param start:		The starting location
	 * @param goal:			The goal location
	 * @param parentMap:	The HashMap that keeps track of the path followed during the search
	 * @return:				True if a path exists from start to goal, false otherwise 
	 */
	private boolean bfsSearch(GeographicPoint start, GeographicPoint goal, HashMap<GeographicPoint, GeographicPoint> parentMap, Consumer<GeographicPoint> nodeSearched) {
		
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		Queue<GeographicPoint> toExplore = new LinkedList<GeographicPoint>();
		
		toExplore.add(start);
		boolean found = false;
		
		// Do the search
		while (!toExplore.isEmpty()) {
			
			GeographicPoint curr = toExplore.remove();
			
			// if the distance between GeographicPoints curr and goal is zero,
			// this means they are the same point
			if (curr.distance(goal) == 0) {
				found = true;
				break;
			}
			
			// curr is a GeographicPoint object. To find its neighbors, we first
			// get the corresponding MapNode using our graphNodesHashMap. Then, we
			// use the appropriate method in the MapNode class to the get the neighbors
			MapNode currMapNode = graphNodesHashMap.get(curr);
			
			List<GeographicPoint> neighbors = currMapNode.getMapNodeNeighborsAsPoints();
			
			ListIterator<GeographicPoint> it = neighbors.listIterator(neighbors.size());
			while (it.hasPrevious()) {
				GeographicPoint next = it.previous();
				// ******************************
				// Report node to consumer as it is explored
				nodeSearched.accept(next);
				// ******************************
				if (!visited.contains(next)) {
					visited.add(next);
					parentMap.put(next, curr);
					toExplore.add(next);
				}
			}
		}
		return found;
	}
	
	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return dijkstra(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		// TODO: Implement this method in WEEK 3

		// Hook for visualization. See writeup.
		// nodeSearched.accept(next.getLocation());

		return null;
	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return aStarSearch(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		// TODO: Implement this method in WEEK 3

		// Hook for visualization. See writeup.
		// nodeSearched.accept(next.getLocation());

		return null;
	}

	public static void main(String[] args) {
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");
		System.out.println(theMap);
		
		// test bfs
		//System.out.println("");
		//System.out.println(" ****** Test BFS ******** ");
		//GeographicPoint startPoint = new GeographicPoint(1.0, 1.0);
		//GeographicPoint endPoint = new GeographicPoint(8.0, -1.0);
		//System.out.println(theMap.bfs(startPoint, endPoint));

		// Edge Distance Testing
		System.out.println("");
		System.out.println(" ****** Test Edge Distance ******** ");
		GeographicPoint point = new GeographicPoint(4.0, 1.0);
		GeographicPoint point2 = new GeographicPoint(7.0, 3.0);
		MapNode node = theMap.graphNodesHashMap.get(point);
		System.out.println(node);
		System.out.println(node.hasEdge("main"));
		//System.out.println(node.getEdgeList());
		System.out.println(node.getEdgeTo(point2));
		
		// PriorityQueue Testing
		Queue<GeographicPoint> pq1 = new PriorityQueue<GeographicPoint>();
		
		// You can use this method for testing.

		/*
		 * Use this code in Week 3 End of Week Quiz MapGraph theMap = new
		 * MapGraph(); System.out.print("DONE. \nLoading the map...");
		 * GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		 * System.out.println("DONE.");
		 * 
		 * GeographicPoint start = new GeographicPoint(32.8648772,
		 * -117.2254046); GeographicPoint end = new GeographicPoint(32.8660691,
		 * -117.217393);
		 * 
		 * 
		 * List<GeographicPoint> route = theMap.dijkstra(start,end);
		 * List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		 * 
		 */

	}

}
