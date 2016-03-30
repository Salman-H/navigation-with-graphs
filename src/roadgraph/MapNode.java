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
public class MapNode {
	private GeographicPoint nodeLocation;
	private List<MapEdge> nodeEdgesList;
	
	/**
	 * Constructor
	 * 
	 * @param nodeLocation
	 */
	public MapNode(GeographicPoint nodeLocation) {
		this.nodeLocation = nodeLocation;
		nodeEdgesList = new LinkedList<MapEdge>();
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
	 * Return a String representation of the MapNode
	 * 
	 *  @return A String representation of the MapNode
	 */
	public String toString() {
		String s = "\n\n ***";
		s += "MapNode with:";
		s += "\n\t" + "nodelocation: " + nodeLocation;
		s += "\n\t" + "nodeEdgesList";
		for (MapEdge edge: nodeEdgesList) {
			s += edge;
		}
		return s;
	}

}
