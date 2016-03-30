/**
 * 
 */
package roadgraph;

import geography.GeographicPoint;

/**
 * @author Salman Hashmi
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
	 * Constructor
	 * @param startPoint
	 * @param endPoint
	 */
	public MapEdge(GeographicPoint startPoint, GeographicPoint endPoint) {
		this.startPoint = startPoint;
		this.endPoint = endPoint;
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
		String s = "MapEdge with:";
		s += "\n\t" + "startPoint: " + startPoint;
		s += "\n\t" + "endPoint: " + endPoint;
		s += "\n\t" + "roadName: " + roadName;
		s += "\n\t" + "roadType: " + roadType;
		s += "\n\t" + "roadDistance: " + roadDistance;
		return s;
	}

}
