package roadgraph;

import java.util.PriorityQueue;
import java.util.Queue;
import geography.GeographicPoint;
import java.util.Comparator;
import java.util.Collections;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


public class PriorityQueueTest {
	public static void main(String[] args) {
	 
		PriorityQueue<MepNode> pq = new PriorityQueue<MepNode>();
		
	     MepNode node0 = new MepNode(1.1);
	     node0.setCurrentDistance(9.1);
	     
	     MepNode node1 = new MepNode(4.1);
	     node1.setCurrentDistance(9.5);
	     
	     MepNode node3 = new MepNode(5.1);
	     node3.setCurrentDistance(9.2);
	     
	     pq.add(node0);
	     pq.add(node1);
	     pq.add(node3);
	     
	     while (pq.size() != 0) {
	         System.out.println(pq.remove().getCurrentDistance());
	     }
	}
}

class MepNode implements Comparable<MepNode> {
	
	private double nodeLocation;
	private double currentDistance;
	
	public MepNode(double nodeLocation) {
		this.nodeLocation = nodeLocation;
		this.currentDistance = Double.POSITIVE_INFINITY;
	}
	
	public void setCurrentDistance(double currentDistance) {
		this.currentDistance = currentDistance;
	}
	
	public double getCurrentDistance() {
		return currentDistance;
	}
	
	public int compareTo(MepNode node) {
		Double thisDistance = new Double(this.getCurrentDistance());
		Double otherDistance = new Double(node.getCurrentDistance());
	
		return (thisDistance.compareTo(otherDistance));
	}
}
/*
class MepNode {
	
	private double nodeLocation;
	private double currentDistance;
	
	public MepNode(double nodeLocation) {
		this.nodeLocation = nodeLocation;
		this.currentDistance = Double.POSITIVE_INFINITY;
	}
	
	public void setCurrentDistance(double currentDistance) {
		this.currentDistance = currentDistance;
	}
	
	public double getCurrentDistance() {
		return currentDistance;
	}
}
*/