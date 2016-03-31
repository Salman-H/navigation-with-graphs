/**
 * @author UCSD MOOC development team
 * @author Salman Hashmi
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;

import java.util.List;
import java.util.ListIterator;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;

import geography.GeographicPoint;
import util.GraphLoader;


/**
 * @author UCSD MOOC development team
 * @author Salman Hashmi
 * 
 *         A class which represents a graph of geographic locations Nodes in the
 *         graph are intersections between
 *
 */
public class MapGraph {
	// TODO: Add your member variables here in WEEK 2
	private HashMap<GeographicPoint, MapNode> graphNodesHashMap;
	private int numNodes;
	private int numEdges;
	
	GeographicPoint zero = new GeographicPoint(1.0, 1.0);
	GeographicPoint one = new GeographicPoint(4.0, 1.0);
	GeographicPoint two = new GeographicPoint(4.0, 2.0);
	GeographicPoint three = new GeographicPoint(5.0, 1.0);
	GeographicPoint four = new GeographicPoint(6.5, 0.0);
	GeographicPoint five = new GeographicPoint(8.0, -1.0);
	GeographicPoint six = new GeographicPoint(4.0, 0.0);
	GeographicPoint seven = new GeographicPoint(7.0, 3.0);
	GeographicPoint eight = new GeographicPoint(4.0, -1.0);
	
	
	HashMap<GeographicPoint, Integer> tempDict = new HashMap<GeographicPoint, Integer>();

	/**
	 * Create a new empty MapGraph
	 */
	public MapGraph() {
		// TODO: Implement in this constructor in WEEK 2
		graphNodesHashMap = new HashMap<GeographicPoint, MapNode>();
		numNodes = 0;
		numEdges = 0;
		
		tempDict.put(zero, 0);
		tempDict.put(one, 1);
		tempDict.put(two, 2);
		tempDict.put(three, 3);
		tempDict.put(four, 4);
		tempDict.put(five, 5);
		tempDict.put(six, 6);
		tempDict.put(seven, 7);
		tempDict.put(eight, 8);
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
		// otherwise..
		graphNodesHashMap.put(location, new MapNode(location));
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
			//return new LinkedList<GeographicPoint>();
			return null;
		}
		
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		boolean found = bfsSearch(start, goal, parentMap);

		if (!found) {
			System.out.println("No path exists");
			//return new LinkedList<GeographicPoint>();
			return null;
		}
		
		// reconstruct the path
		return constructPath(start, goal, parentMap);
	}
	
	// reconstruct the path
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
	
	// do the actual bfs serach and fill the HashMap parentMap along the way
	private boolean bfsSearch(GeographicPoint start, GeographicPoint goal, HashMap<GeographicPoint, GeographicPoint> parentMap) {
		
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
			MapNode currMapNode = graphNodesHashMap.get(curr);
			List<GeographicPoint> neighbors = currMapNode.getMapNodeNeighborsAsPoints();
			
			ListIterator<GeographicPoint> it = neighbors.listIterator(neighbors.size());
			while (it.hasPrevious()) {
				GeographicPoint next = it.previous();
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
		System.out.println("");
		System.out.println(" ****** Test BFS ******** ");
		GeographicPoint startPoint = new GeographicPoint(1.0, 1.0);
		GeographicPoint endPoint = new GeographicPoint(8.0, -1.0);
		//System.out.println(theMap.bfs(startPoint, endPoint));

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
