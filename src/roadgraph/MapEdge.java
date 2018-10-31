package roadgraph;

import geography.GeographicPoint;
/*
 * @Author Kaidi
 * 
 * A class that represents an edge in the map
 * It includes start point, end point, and other parameters
*/
public class MapEdge {
	private MapNode start;
	private MapNode end;
	private String streetName;
	private String streetType;
	private double length;
	
	/*
	 * constructor
	*/
	public MapEdge(MapNode start, MapNode end, String streetName, String streetType, double length){
		this.start = start;
		this.end = end;
		this.streetName = streetName;
		this.streetType = streetType;
		this.length = length;
	}
	
	/* 
	 * get the end point of the edge
	 * @return end GeographicPoint
	 */
	public MapNode getEnd(){
		return this.end;
	}
	
	public double getLength(){
		return this.length;
	}
	
	/* 
	 * toString for printing
	 * @return string including streetName, streetType and length
	 */
	public String toString(){
		String s = streetName + " " + streetType + " " + length + " km";
		//You can also add start and end: "From "+ start.toString() + " to " + end.toString() + " " + 
		return s;
	}
}
