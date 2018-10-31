package roadgraph;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import geography.GeographicPoint;

/**
 * @author kaidizhang
 * A class which represents a node in the map
 */

public class MapNode implements Comparable {
	private GeographicPoint loc;
	private Set<MapEdge> edges;
	// distance from start to this
	private double distance;
	// distance from start to this + straight line distance between this and goal
	private double estimatedDistance;
	/*
	 * constructor: set loc and edges
	 * Is arraylist the best choice?
	*/
	public MapNode(GeographicPoint location){
		loc = location;
		edges = new HashSet<MapEdge>();
		distance = 0;
		estimatedDistance = 0;
	}
	
	/* 
	 * get the list of edges
	 * @return a new list of edges, not the instance variable
	 */	
	public Set<MapEdge> getEdges(){
		return this.edges;
	}
	
	/* 
	 * get all the neighbors from the edges
	 * @return a list of MapNodes containing all the neighbors
	 */
	public Set<MapNode> getNeighbors(){
		Set<MapNode> neighbors = new HashSet<MapNode>();
		for(MapEdge e: edges){
			neighbors.add(e.getEnd());
		}
		return neighbors;
	}
	
	/*
	 * add a MapEdge to this node
	 * @param a new edge that needs to be added
	 * 
	*/
	public void nodeAddEdge(MapEdge edge){
		edges.add(edge);
	}
	
	/*
	 * return the loc of MapNode
	 * @return loc as a new GeographicPoint variable 
	 */
	public GeographicPoint getLocation(){
		return this.loc;
	}
	
	public double getNeighborDistance(MapNode other){
		double neighborDistance = 0;
		if (!getNeighbors().contains(other))
			throw new IllegalArgumentException();
		
		for(MapEdge e: edges){
			if(e.getEnd().equals(other)){
				neighborDistance = e.getLength();
				break;
			}
		}
		return neighborDistance;
	}
	
	public double getDistance(){
		return this.distance;
	}
	
	public void setDistance(double set){
		this.distance = set;
	}
	
	public double getEstimatedDistance(){
		return this.estimatedDistance;
	}
	
	public void setEstimatedDistance(double set){
		this.estimatedDistance = set;
	}
	
	/**
	 * @param other
	 * @return true if locations of the two nodes are the same
	 */
	/* (non-Javadoc)
	 * @see java.lang.Object#equals(java.lang.Object)
	 */
	public boolean equals(Object o){
		MapNode other = (MapNode)o;
		if (other == null)
			return false;
		return this.loc.equals(other.loc);
	}
	
	/**
	 * @return
	 */
	public int HashCode(){
		return loc.hashCode();
	}
	
	/*
	 * toString for printing
	 * @return string of loc
	*/
	public String toString(){
		String s = loc.toString();
		return s;
	}

	/* (non-Javadoc)
	 * @see java.lang.Comparable#compareTo(java.lang.Object)
	 */
	@Override
	public int compareTo(Object o) {
		// TODO Auto-generated method stub
		MapNode other = (MapNode)o;
		// for dijkstra: estimatedDistance = distance + 0;
		// for a star search
		if(this.estimatedDistance > other.estimatedDistance) return 1;
		else if (this.estimatedDistance < other.estimatedDistance) return -1;
		else return 0;
		//return ((Double)this.estimatedDistance).compareTo((Double)other.estimatedDistance);
	}
}
