/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and Kaidi
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between edges.
 *
 */

public class MapGraph {
	//TODO: Add your member variables here in WEEK 2
	private Map<GeographicPoint, MapNode> nodes;
	private int numVertices;
	private int numEdges;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 2
		nodes = new HashMap<GeographicPoint, MapNode>();
		numVertices = 0;
		numEdges = 0;
		
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		return numVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2
		Set<GeographicPoint> nodeSet = nodes.keySet();
		return nodeSet;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 2
		return numEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 2
		if(location == null || nodes.containsKey(location)){
			return false;
		}
		
		MapNode node = new MapNode(location);
		
		nodes.put(location, node);
		numVertices++;
		return true;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 2
		if(from == null || to == null || roadName == null || roadType == null){
			throw new IllegalArgumentException();
		}
		if(!nodes.containsKey(from) || !nodes.containsKey(to) || length < 0){
			throw new IllegalArgumentException();
		}
		
		MapNode start = nodes.get(from);
		MapNode end = nodes.get(to);
		MapEdge edge = new MapEdge(start, end, roadName, roadType, length);
		start.nodeAddEdge(edge);
		
		numEdges++;
		
	}
	
	/*
	 * print for debugging
	*/
	public void printGraph(){
		System.out.println("The map has "+ numVertices + " nodes and " + numEdges + " edges.");
		System.out.println();
		
		/*System.out.println("Nodes in the map:");
		System.out.println(nodes.keySet());
		System.out.println();
		
		System.out.println("Edges in the map:");
		for (GeographicPoint loc: nodes.keySet()){
			System.out.println("***************");
			System.out.println("The node "+ loc + " have " + nodes.get(loc).getEdges().size() + " edges:");
			System.out.println(nodes.get(loc).getEdges());
			System.out.println("The node "+ loc + " have " + nodes.get(loc).getNeighbors().size() + " neighbors:");
			System.out.println(nodes.get(loc).getNeighbors());
			System.out.println();
		}*/
	}

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 2
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		MapNode startNode = nodes.get(start);
		MapNode goalNode = nodes.get(goal);
		
		// create a queue: first in, first out
		Queue<MapNode> queue = new LinkedList<MapNode>();
		// create a visited list, so that the nodes will not be double checked
		Set<MapNode> visited = new HashSet<MapNode>();
		// create a map: the value MapNode is the parent of the key MapNode
		Map<MapNode, MapNode> parent = new HashMap<MapNode, MapNode>();
		
		MapNode curr = startNode;
		queue.add(curr);
		visited.add(curr);
		
		while(!queue.isEmpty()){
			curr = queue.remove();
			System.out.println();
			System.out.println("Current node: "+curr);
			
			for(MapNode neighbor: curr.getNeighbors()){
				if(!visited.contains(neighbor)){
					System.out.println("Neighbor node: "+neighbor);
					visited.add(neighbor);
					queue.add(neighbor);
					nodeSearched.accept(neighbor.getLocation());
					parent.put(neighbor, curr);
					
					// return the path if neighbor is the goal
					// this is more efficient than returning the path if curr is the goal
					if (neighbor.equals(goalNode)){
						return constructPath(neighbor, startNode, parent);
					}
				}
			}
		}
		// if no path is found between start and goal
		return null;
		
	}
	
	/*
	 * helper method to construct the path from the current point to the start point
	 * parent point will be added to the front of the list until the start point is reached
	 * @param current point
	 * @param start point
	 * @param parent map constructed in BFS
	 * @return list of points from start to current
	 */	
	public List<GeographicPoint> constructPath(MapNode current, MapNode start, Map<MapNode, MapNode> parent){
		List<GeographicPoint> path = new LinkedList<GeographicPoint>();
		
		MapNode gp = current;
		path.add(0, gp.getLocation());
		while(!gp.equals(start)){
			gp = parent.get(gp);
			path.add(0, gp.getLocation());
		}
		return path;
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		MapNode startNode = nodes.get(start);
		MapNode goalNode = nodes.get(goal);
		
		PriorityQueue<MapNode> pqueue = new PriorityQueue<MapNode>();
		Set<MapNode> visited = new HashSet<MapNode>();
		Map<MapNode, MapNode> parent = new HashMap<MapNode, MapNode>();
		
		for(GeographicPoint gp: nodes.keySet()){
			nodes.get(gp).setDistance(Double.POSITIVE_INFINITY);
		}
		
		MapNode curr = startNode;
		startNode.setDistance(0);
		pqueue.add(startNode);
		int nodeCount = 0;
		while(!pqueue.isEmpty()){
			curr = pqueue.remove();
			nodeCount++;
			//System.out.println();
			//System.out.println("Current node: "+curr+" distance: "+ curr.getDistance());
			
			if(!visited.contains(curr)){
				if (curr.equals(goalNode)){
					//System.out.println("dijkstra Nodes removed from PriorityQueue: "+nodeCount);
					//System.out.println("dijkstra Nodes visited: "+(visited.size()+1));
					return constructPath(curr, startNode, parent);
				}
				visited.add(curr);
				for(MapNode neighbor: curr.getNeighbors()){
					if(!visited.contains(neighbor)){
						double updateDistance = curr.getDistance() + curr.getNeighborDistance(neighbor);
						// if updateDistance is smaller than the old distance: update the distance and add into pqueue
						if(updateDistance < neighbor.getDistance()){
							neighbor.setDistance(updateDistance);
							// Dijkstra: goalDistance = 0, so estimatedDistance = distance
							neighbor.setEstimatedDistance(updateDistance);
							//System.out.println("Neighbor node: "+neighbor+" distance: "+ neighbor.getDistance());
							
							pqueue.add(neighbor);
							parent.put(neighbor, curr);
							nodeSearched.accept(neighbor.getLocation());
						}
					}
				}
			}
		}
		// if no path is found between start and goal
		return null;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		MapNode startNode = nodes.get(start);
		MapNode goalNode = nodes.get(goal);
		
		PriorityQueue<MapNode> pqueue = new PriorityQueue<MapNode>();
		Set<MapNode> visited = new HashSet<MapNode>();
		Map<MapNode, MapNode> parent = new HashMap<MapNode, MapNode>();
		
		for(GeographicPoint gp: nodes.keySet()){
			nodes.get(gp).setDistance(Double.POSITIVE_INFINITY);
			nodes.get(gp).setEstimatedDistance(Double.POSITIVE_INFINITY);
		}
		
		startNode.setDistance(0);
		startNode.setEstimatedDistance(start.distance(goal));
		pqueue.add(startNode);
		
		MapNode curr = startNode;
		int nodeCount = 0;
		
		while(!pqueue.isEmpty()){
			curr = pqueue.remove();
			nodeCount++;
			//System.out.println();
			//System.out.println("Current node: "+curr+" estimatedDistance: "+ curr.getEstimatedDistance());
			
			if(!visited.contains(curr)){
				if (curr.equals(goalNode)){
					//System.out.println("aStarSearch Nodes removed from PriorityQueue: "+nodeCount);
					//System.out.println("aStarSearch Nodes visited: "+(visited.size()+1));
					return constructPath(curr, startNode, parent);
				}
				visited.add(curr);
				for(MapNode neighbor: curr.getNeighbors()){
					if(!visited.contains(neighbor)){
						double distance = curr.getDistance() + curr.getNeighborDistance(neighbor);
						double goalDistance = neighbor.getLocation().distance(goal);
						double estimatedDistance = distance + goalDistance;
						
						if(estimatedDistance < neighbor.getEstimatedDistance()){
							neighbor.setDistance(distance);
							neighbor.setEstimatedDistance(estimatedDistance);
							//System.out.println("Neighbor node: "+neighbor+" estimatedDistance: "+ neighbor.getEstimatedDistance());
							
							pqueue.add(neighbor);
							parent.put(neighbor, curr);
							nodeSearched.accept(neighbor.getLocation());
						}
					}
				}
			}
		}
		// if no path is found between start and goal
		return null;
	}

	/**
	 * Travelling Salesperson problem using Greedy method: travel all the vertexes and return back to the start
	 * Because the map is not a complete graph, some vertexes are visited more than once
	 * @param start
	 * @return the list of vertexes travelled
	 */
	public List<GeographicPoint> tavellingSalespersonGreedy(GeographicPoint start){
		
		MapNode startNode = nodes.get(start);
		Set<MapNode> visited = new HashSet<MapNode>();
		List<GeographicPoint> path = new ArrayList<GeographicPoint>();
		MapNode curr = startNode;
		
		while(true){
			visited.add(curr);
			System.out.println("\n"+"current node: "+ curr);
			
			MapNode next = getClosestNode(curr, visited);
			if(next != null){
				List<GeographicPoint> l = aStarSearch(curr.getLocation(), next.getLocation());
				l.remove(l.size()-1);
				path.addAll(l);
				curr = next;
			}
			// if all the nodes have been visited: curr is the last node, return to the start
			else{
				path.addAll(aStarSearch(curr.getLocation(), start));
				break;
			}
		}
		return path;
	}

	/**
	 * helper method to find the closest node that has not been visited
	 * @param curr
	 * @param visited
	 * @return null if all the nodes have been visited
	 */
	private MapNode getClosestNode(MapNode curr, Set<MapNode> visited){
		MapNode next = null;
		double closest = Double.POSITIVE_INFINITY;
		// find closest neighbor among neighbors that have not been visited
		// if all the neighbors have been visited: next will be null
		for(MapNode n: nodes.values()){
			double distance = pathDistance(aStarSearch(curr.getLocation(), n.getLocation()));
			if (!visited.contains(n) && distance < closest){
				next = n;
				closest = distance;
			}
		}
		//System.out.println("next node: "+ next);
		return next;
	}
	
	/**
	 * calculate the distance 
	 * @param a list of GeographicPoint
	 * @return distance travelled from start of the list to end of the list
	 */
	public double pathDistance(List<GeographicPoint> l){
		double distance = 0;
		for(int i=0; i<l.size()-1; i++){
			MapNode curr = nodes.get(l.get(i));
			MapNode next = nodes.get(l.get(i+1));
			distance += curr.getNeighborDistance(next);
		}
		return distance;
	}
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.println("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		theMap.printGraph();
		System.out.println("DONE.");
		List<GeographicPoint> l = theMap.tavellingSalespersonGreedy(new GeographicPoint(8,-1));
		System.out.println("Travelling salesperson from (8,-1): "+ l);
		System.out.println("Nodes travelled in total: "+ l.size());
		System.out.println("Distance travelled in total: "+ theMap.pathDistance(l));
		
		/*System.out.println("Dijkstra from (1, 1) to (8, -1): ");
		System.out.println("Path from start to goal: "+theMap.dijkstra(new GeographicPoint(1,1), new GeographicPoint(8,-1)));
		System.out.println("A Star Search from (1, 1) to (8, -1): ");
		System.out.println("Path from start to goal: "+theMap.aStarSearch(new GeographicPoint(1,1), new GeographicPoint(8,-1)));*/
		
		// You can use this method for testing.  
		
		
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/parkcrest.map", theMap);
		theMap.printGraph();
		System.out.println("DONE.");
		
		GeographicPoint start = new GeographicPoint(38.921079, -77.080801); 
		
		List<GeographicPoint> l = theMap.tavellingSalespersonGreedy(start);
		System.out.println("Travelling salesperson from (38.921079, -77.080801): "+ l);
		System.out.println("Nodes travelled in total: "+ l.size());
		System.out.println("Distance travelled in total: "+ theMap.pathDistance(l));*/
		
		//Use this code in Week 3 End of Week Quiz
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		theMap.printGraph();
		System.out.println("DONE.");
		
		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046); 
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);*/

		
		
	}
	
}
