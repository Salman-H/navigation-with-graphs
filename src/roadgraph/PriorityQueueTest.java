package roadgraph;

import java.util.PriorityQueue;
import java.util.Queue;

import geography.GeographicPoint;

public class PriorityQueueTest {

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		Queue<GeographicPoint> pq1 = new PriorityQueue<GeographicPoint>();
		
		GeographicPoint startPoint = new GeographicPoint(1.0, 1.0);
		MapNode startNode = new MapNode(startPoint);
		
		GeographicPoint point1 = new GeographicPoint(4.0, 1.0);
		MapNode node1 = new MapNode(point1);
		
		System.out.println("startNode currentDistance: " + startNode.getCurrentDistance());
		
	}

}
