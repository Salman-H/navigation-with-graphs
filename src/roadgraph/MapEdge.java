/**
 * 
 */
package roadgraph;

import geography.GeographicPoint;

/**
 * MapEdge.java
 * 
 * @author Salman Hashmi
 * 
 * 		A class to represent an edge/road between two geographic points/intersections in a graph/map 
 *
 */
public class MapEdge {
	/**
	 * Member variables / fields
	 */
	private GeographicPoint startPoint;
	private GeographicPoint endPoint;
	private String roadName;
	private String roadType;
	private double roadDistance;
	
	/**
	 * no-argument constructor
	 */
	public MapEdge() {
		this.startPoint = null;
		this.endPoint = null;
	}
	
	/**
	 * Constructor
	 * @param startPoint
	 * @param endPoint
	 */
	public MapEdge(GeographicPoint startPoint, GeographicPoint endPoint, String roadName, String roadType, double roadDistance) {
		this.startPoint = startPoint;
		this.endPoint = endPoint;
		this.roadName = roadName;
		this.roadType = roadType;
		this.roadDistance = roadDistance;
	}
	
	/**
	 * Setter for roadName
	 * @param roadName
	 */
	public void setRoadName(String roadName) {
		this.roadName = roadName;
	}
	
	/**
	 * Setter for RoadType
	 * @param roadType
	 */
	public void setRoadType(String roadType) {
		this.roadType = roadType;
	}
	
	/**
	 * Setter for roadDistance
	 * @param roadDistance
	 */
	public void setRoadDistance(double roadDistance) {
		this.roadDistance = roadDistance;
	}
	
	/**
	 * Getter for endPoint
	 * @return endPoint;
	 */
	public GeographicPoint getEndPoint() {
		return endPoint;
	}
	
	/**
	 * Getter for roadName
	 * @return roadName
	 */
	public String getRoadName() {
		return roadName;
	}
	
	/**
	 * Getter for roadType
	 * @return roadType
	 */
	public String getRoadType() {
		return roadType;
	}
	
	/**
	 * Getter for roadDistance
	 * @return roadDistance
	 */
	public double getRoadDistance() {
		return roadDistance;
	}
	
	/**
	 * Return a String representation of the MapEdge
	 *  @return A String representation of the MapEdge
	 */
	public String toString() {
		String s = "\n\t\tMapEdge with:";
		s += "\n\t\t" + "startPoint: " + startPoint;
		s += "\n\t\t" + "endPoint: " + endPoint;
		s += "\n\t\t" + "roadName: " + roadName;
		s += "\n\t\t" + "roadType: " + roadType;
		s += "\n\t\t" + "roadDistance: " + roadDistance;
		s += "\n";
		return s;
	}

}
