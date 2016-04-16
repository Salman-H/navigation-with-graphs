/**
 * 
 */
package roadgraph;

import geography.GeographicPoint;
import java.util.List;
import java.util.ArrayList;
import java.util.LinkedList;

/**
 * MapNode.java
 * 
 * @author Salman Hashmi
 * 
 * 			A class that represents a node or vertex in  MapGraph (an intersection on our map)
 *
 */
public class MapNode implements Comparable<MapNode>{
	/**
	 * Member variables / fields
	 */
	private GeographicPoint nodeLocation;
	private List<MapEdge> nodeEdgesList;
	private double currentDistanceFromStartNode;
	
	/**
	 * Constructor
	 * 
	 * @param nodeLocation
	 */
	public MapNode(GeographicPoint nodeLocation) {
		this.nodeLocation = nodeLocation;
		nodeEdgesList = new LinkedList<MapEdge>();
		this.currentDistanceFromStartNode = Double.POSITIVE_INFINITY;
	}
	
	/**
	 * adds the specified MapEdge to the nodeEdgesList of the MapNode
	 * 
	 * @param edge
	 */
	public void addEdgeToNode(MapEdge edge) {
		nodeEdgesList.add(edge);
	}
	
	/**
	 * If this node has an edge with the specified roadName, it returns true; otherwise false
	 * 
	 * @param roadName
	 * @return true or false
	 */
	public boolean hasEdge(String roadName) {
		for (MapEdge edge: nodeEdgesList) {
			if (edge.getRoadName().equalsIgnoreCase(roadName)) return true;
		}
		return false;
	}
	
	/**
	 * Getter for nodeEdgesList
	 * @return nodeEdgesList
	 */
	public List<MapEdge> getEdgeList() {
		return nodeEdgesList;
	}
	
	/**
	 * Gets the MapEdge from this MapNode to the specified MapNode GeographicPoint
	 * @return a MapEdge
	 */
	public MapEdge getEdgeTo(GeographicPoint toPoint) throws IllegalArgumentException {
		
		if (toPoint == null) throw new IllegalArgumentException();
		
		for (MapEdge mapEdge: nodeEdgesList) {
			if (mapEdge.getEndPoint().distance(toPoint) == 0) {
				return mapEdge;
			}
		}
		return null;
	}
	
	/**
	 * Gets the list of neighboring nodes in GeographicPoint format
	 * 
	 * @return A list of neighboring nodes in GeographicPoint format
	 */
	public List<GeographicPoint> getMapNodeNeighborsAsPoints() {
		List<GeographicPoint> nodeNeighborsAsPoints = new LinkedList<GeographicPoint>();
		for (MapEdge mapEdge: nodeEdgesList) {
			nodeNeighborsAsPoints.add(mapEdge.getEndPoint());
		}
		return nodeNeighborsAsPoints;
	}
	
	/**
	 * Setter for currentDistanceFromStartNode
	 * @param currentDistanceFromStartNode
	 */
	public void setCurrentDistance(double currentDistanceFromStartNode) {
		this.currentDistanceFromStartNode = currentDistanceFromStartNode;
	}
	
	/**
	 * Getter for currentDistanceFromStartNode
	 * @return currentDistanceFromStartNode
	 */
	public double getCurrentDistance() {
		return currentDistanceFromStartNode;
	}
	
	/**
	 * Compares this MapNode with some other MapNode by their currentDistanceFromStartNode fields
	 * 
	 * @param a MapNode otherNode
	 * @return an int; -1 if thisCurrentDistance is less than the other's, 0 if they are the same, 1 otherwise
	 */
	@Override 
	public int compareTo(MapNode otherNode) {
		Double thisCurrentDistance = new Double(this.getCurrentDistance());
		Double otherCurrentDistance = new Double(otherNode.getCurrentDistance());
		
		return (thisCurrentDistance.compareTo(otherCurrentDistance));
	}
	
	/**
	 * Return a String representation of the MapNode
	 * 
	 *  @return A String representation of the MapNode
	 */
	public String toString() {
		String s = "\n\t" + "nodelocation: " + nodeLocation + "   neighbors ->  ";
		s += "" + getMapNodeNeighborsAsPoints();
		//s += "\n\t" + "nodeEdgesList";
		//for (MapEdge edge: nodeEdgesList) {
		//	s += edge;
		//}
		return s;
	}

}
