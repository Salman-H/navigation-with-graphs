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
		// ********** testing ****************
		System.out.println("");
		System.out.println("Inside constructPath");
		
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		GeographicPoint curr = goal;
		
		// ********** testing ****************
		System.out.println("");
		System.out.println("parentMap inside constructPath:");
		System.out.println(parentMap);
		
		// test counter for limited loop run
		//int counter = 0;
		
		// condition: curr != start <- original
		while (curr.distance(start) != 0) {
			
			// ********** testing ****************
			//if (counter == 6) break;
			//System.out.println("");
			//System.out.println("while (curr != start)..");
			
			//System.out.println("curr == " + curr + "added to start of path");
			path.addFirst(curr);
			//System.out.println("path: " + path);
			
			//System.out.println("curr = parentMap.get( " + curr + " )");
			curr = parentMap.get(curr);
			//System.out.println("new curr: " + curr);
			
			//System.out.println("next iteration needed?");
			//System.out.println("curr == start?");
			//System.out.println(curr + " == " + start + "?");
			//System.out.println(curr == start);
			
			// test counter
			//counter++;
		}
		path.addFirst(start);
		
		// ********** testing ****************
		System.out.println("");
		System.out.println("constructed path:");
		//System.out.println(path);
		
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
			
			/*
			  for (GeographicPoint neighborLocation: neighbors) {
			  		if (!visited.contains(neighborLocation)) {
			  			visited.add(neighborLocation);
			  			parentMap.put(curr, neighborLocation);
			  			toExplore.add(neighborLocation);
			  		}
			  }*/
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
	 * Find the shortest path from start to goal using Dijkstra's algorithm
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
		
		System.out.println("");
		System.out.println(" *** Dijkstra ***");
		System.out.println(" start: " + start);
		System.out.println(" goal: " + goal);
		
		if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return null;
		}
		
		// the List to be returned outlining the desired path is constructed from this HashMap
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		
		// call the dijkstraSearch helper method to perform the actual search
		
		boolean found = graphSearchAlgorithm("dijkstra", start, goal, parentMap, nodeSearched);
		
		// if no path found, return null
		if (!found) {
			System.out.println("No path exists");
			return null;
		}
		
		// reconstruct the path
		return constructPath(start, goal, parentMap);

		//return null;
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
		
		System.out.println("");
		System.out.println(" *** A* ***");
		System.out.println(" start: " + start);
		System.out.println(" goal: " + goal);
		
		if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return null;
		}
		
		// the List to be returned outlining the desired path is constructed from this HashMap
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		
		// call the dijkstraSearch helper method to perform the actual search
		//boolean found = aStarSearchAlgorithm(start, goal, parentMap, nodeSearched);
		
		boolean found = graphSearchAlgorithm("astar", start, goal, parentMap, nodeSearched);
		
		// if no path found, return null
		if (!found) {
			System.out.println("No path exists");
			return null;
		}
		
		// reconstruct the path
		return constructPath(start, goal, parentMap);
	}
	
	/**
	 * Perform the actual graph search (for Dijkstra or A*) and fill the HashMap parentMap along the way
	 * 
	 * @param algoName:		The name of the algorithm search which is to be implemented
	 * @param start:		The starting location
	 * @param goal:			The goal location
	 * @param parentMap:	The HashMap that keeps track of the path followed during the search
	 * @return:				True if a path exists from start to goal, false otherwise 
	 * 
	 * g(n): actual current distance from start to n
	 * h(n): straight-line distance from n to goal
	 * f(n): predicted distance = g(n) from start to n + h(n) from n to goal
	 * 
	 */
	private boolean graphSearchAlgorithm(String algoName, GeographicPoint start, GeographicPoint goal, HashMap<GeographicPoint, GeographicPoint> parentMap, Consumer<GeographicPoint> nodeSearched) {

		/** Initialize: visited HashSet, Priority Queue **/
		HashSet<MapNode> visitedSet = new HashSet<MapNode>();
		PriorityQueue<MapNode> toExplorePQ = new PriorityQueue<MapNode>();
		
		/** Initialize: g(n) and f(n) to positive infinity for all graph MapNodes **/
		for (MapNode node: graphNodesHashMap.values()) {
			node.setCurrentDistance(Double.POSITIVE_INFINITY);
			node.setPredictedDistance(Double.POSITIVE_INFINITY);
		}
		
		MapNode startNode = graphNodesHashMap.get(start);
		
		/** Initialize: g(n) and f(n) to positive infinity for the startNode **/
		startNode.setCurrentDistance(0);
		startNode.setPredictedDistance(0);
		
		/** Enqueue {S, currentDistance=0} onto the PQ **/
		toExplorePQ.add(startNode);
		
		boolean found = false;
		int visitedNodeCount = 1;
		
		/** while PQ is not empty: Do the search **/
		while (!toExplorePQ.isEmpty()) {
			
			/** dequeue node currNode from front of queue **/
			MapNode currNode = toExplorePQ.remove();
			
			// Testing the order of Node visits
			System.out.println("");
			System.out.println("Node " + visitedNodeCount + ": " + currNode.getNodeLocation());
			visitedNodeCount++;
			
			/** if (currNode is not visited) **/
			if ( !visitedSet.contains(currNode) ) {
				
				/** add currNode to visited set **/
				visitedSet.add(currNode);
				
				/** if currNode == goal location; return parent map **/
				GeographicPoint currPoint = currNode.getNodeLocation();
				if (currPoint.distance(goal) == 0) {
					found = true;
					break;
				}
				// Else visitNeighbors and do all the associated work
				visitNeighbors(algoName, currNode, goal, visitedSet, toExplorePQ, parentMap, nodeSearched);
			}
		}
		// outside loop
		return found;
	}
	
	/*
	 * private helper method for performing the work associated with visiting neighbors in the Dijkstra and A* algorithms
	 */
	private void visitNeighbors(String algoName, MapNode currNode, GeographicPoint goal, HashSet<MapNode> visitedSet, 
			PriorityQueue<MapNode> toExplorePQ, HashMap<GeographicPoint, GeographicPoint> parentMap, Consumer<GeographicPoint> nodeSearched) {
		
		// get currNode's g(n): currentDistance
		double currNodeCurrentDistance = currNode.getCurrentDistance();
		
		List<GeographicPoint> neighborLocationsList = currNode.getMapNodeNeighborsAsPoints();
		
		ListIterator<GeographicPoint> it = neighborLocationsList.listIterator(neighborLocationsList.size());
		
		// while there are still neighbor nodes in the list iterator..
		while (it.hasPrevious()) {
			
			// get the next node (starting from the end of the list)
			GeographicPoint nextPoint = it.previous();
			
			/** Report next node, as it is explored, to consumer for visualization **/
			nodeSearched.accept(nextPoint);
			
			// ..for those neighbors, n, not in visited set already:
			 if (!visitedSet.contains(graphNodesHashMap.get(nextPoint))) {
				 
				// get next neighbor node from neighbor location
				MapNode nextNode = graphNodesHashMap.get(nextPoint);
				
				// get currNode's currentDistance
				double nextNodeCurrentDistance = nextNode.getCurrentDistance();
				
				// get edge from currNode to the current neighborNode
				MapEdge currToNextEdge = currNode.getEdgeTo(nextPoint);
				
				// get distance of currToNeighborEdge
				double currToNextEdgeDistance = currToNextEdge.getRoadDistance();
				
				// get distance of path from start to neighborNode through currNode
				double startThroughCurrToNeighborPathDistance = currNodeCurrentDistance + currToNextEdgeDistance;
				
				// get straight-line distance from neighbor to goal (set to 0 by default for dijkstra)
				double straightLineDistanceFromNeighborToGoal = 0.0;	// Dijkstra: f(n) = g(n) + 0
				
				if (algoName.equals("astar")) {
					straightLineDistanceFromNeighborToGoal = nextPoint.distance(goal);	
				}
				
				// total predicted distance from start to goal
				double totalPredictedDistance = startThroughCurrToNeighborPathDistance + straightLineDistanceFromNeighborToGoal;
				
				/** if path through currNode to neighborNode is shorter than n's currentDistance g(n) **/
				if (startThroughCurrToNeighborPathDistance < nextNodeCurrentDistance) {	
					
					/** update n's distance (from start through currNode to n + straight line from n to goal) **/
					nextNode.setCurrentDistance(startThroughCurrToNeighborPathDistance);	
					nextNode.setPredictedDistance(totalPredictedDistance);
					
					/** update currNode as n's parent in parent map **/
					parentMap.put(nextPoint, currNode.getNodeLocation());
					
					/** Enqueue {n, distance} onto the PQ **/
					toExplorePQ.add(nextNode);
				}
			}
		} 
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
		/*
		System.out.println("");
		System.out.println(" ****** Test Edge Distance ******** ");
		GeographicPoint point1 = new GeographicPoint(4.0, 1.0);
		GeographicPoint point2 = new GeographicPoint(5.0, 1.0);
		MapNode node1 = theMap.graphNodesHashMap.get(point1);
		//System.out.println(node);
		//System.out.println(node.hasEdge("main"));
		//System.out.println(node.getEdgeList());
		System.out.println(node1.getEdgeTo(point2));
		
		// Straight-line Distance Testing
		System.out.println("");
		System.out.println(" ****** Test Straight-Line Distance ******** ");
		System.out.println(point1 + " to " + point2 + " is: ");
		System.out.println(point1.distance(point2));
		*/
		
		// neighbors of a MapNode testing
		/*
		GeographicPoint fourOnePoint = new GeographicPoint(4.0, 1.0);
		MapNode fourOneNode = theMap.graphNodesHashMap.get(fourOnePoint);
		
		List<GeographicPoint> fourOneNodeNeighborsList = fourOneNode.getMapNodeNeighborsAsPoints();
		
		System.out.println("");
		System.out.println("******** from for each loop *******");
		for (GeographicPoint location: fourOneNodeNeighborsList) {
			System.out.println(location);
		}
		
		System.out.println("");
		System.out.println("******* from iterator *******");
		ListIterator<GeographicPoint> it = fourOneNodeNeighborsList.listIterator(fourOneNodeNeighborsList.size());
		while (it.hasPrevious()) {
			GeographicPoint next = it.previous();
			System.out.println(next);
		} */
		
		
		// Testing graphSearchAlgorithm for Dijkstra and A*
		GeographicPoint startPoint = new GeographicPoint(1.0, 1.0);
		GeographicPoint endPoint = new GeographicPoint(8.0, -1.0);
		//GeographicPoint startPoint = new GeographicPoint(8.0, -1.0);
		//GeographicPoint endPoint = new GeographicPoint(1.0, 1.0);
		
		// test Dijkstra
		System.out.println("");
		System.out.println(" ****** Test Dijkstra ******** ");
		System.out.println(theMap.dijkstra(startPoint, endPoint));
		
		// test A*
		System.out.println("");
		System.out.println(" ****** Test A* ******** ");
		System.out.println(theMap.aStarSearch(startPoint, endPoint));

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
