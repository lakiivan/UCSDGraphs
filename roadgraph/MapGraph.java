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
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	// Maintain both nodes and edges as you will need to
	// be able to look up nodes by lat/lon or by roads
	// that contain those nodes.
	private HashMap<GeographicPoint,MapNode> pointNodeMap;
	private HashSet<MapEdge> edges;

	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		pointNodeMap = new HashMap<GeographicPoint,MapNode>();
		edges = new HashSet<MapEdge>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return pointNodeMap.values().size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		return pointNodeMap.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return edges.size();
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
		if (location == null) {
			return false;
		}
		MapNode n = pointNodeMap.get(location);
		if (n == null) {
			n = new MapNode(location);
			pointNodeMap.put(location, n);
			return true;
		}
		else {
			return false;
		}
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

		MapNode n1 = pointNodeMap.get(from);
		MapNode n2 = pointNodeMap.get(to);

		// check nodes are valid
		if (n1 == null)
			throw new NullPointerException("addEdge: pt1:"+from+"is not in graph");
		if (n2 == null)
			throw new NullPointerException("addEdge: pt2:"+to+"is not in graph");

		MapEdge edge = new MapEdge(roadName, roadType, n1, n2, length);
		edges.add(edge);
		n1.addEdge(edge);
		
	}
		
	/** 
	 * Get a set of neighbor nodes from a mapNode
	 * @param node  The node to get the neighbors from
	 * @return A set containing the MapNode objects that are the neighbors 
	 * 	of node
	 */
	private Set<MapNode> getNeighbors(MapNode node) {
		return node.getNeighbors();
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
			 					     GeographicPoint goal, 
			 					     Consumer<GeographicPoint> nodeSearched)
	{
		/* Note that this method is a little long and we might think
		 * about refactoring it to break it into shorter methods as we 
		 * did in the Maze search code in week 2 */
		
		// Setup - check validity of inputs
		if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		MapNode startNode = pointNodeMap.get(start);
		MapNode endNode = pointNodeMap.get(goal);
		if (startNode == null) {
			System.err.println("Start node " + start + " does not exist");
			return null;
		}
		if (endNode == null) {
			System.err.println("End node " + goal + " does not exist");
			return null;
		}

		// setup to begin BFS
		HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		Queue<MapNode> toExplore = new LinkedList<MapNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		toExplore.add(startNode);
		MapNode next = null;

		while (!toExplore.isEmpty()) {
			next = toExplore.remove();
			
			 // hook for visualization
			nodeSearched.accept(next.getLocation());
			
			if (next.equals(endNode)) break;
			Set<MapNode> neighbors = getNeighbors(next);
			for (MapNode neighbor : neighbors) {
				if (!visited.contains(neighbor)) {
					visited.add(neighbor);
					parentMap.put(neighbor, next);
					toExplore.add(neighbor);
				}
			}
		}
		if (!next.equals(endNode)) {
			System.out.println("No path found from " +start+ " to " + goal);
			return null;
		}
		// Reconstruct the parent path
		List<GeographicPoint> path =
				reconstructPath(parentMap, startNode, endNode);

		return path;
	
	}
	


	/** Reconstruct a path from start to goal using the parentMap
	 *
	 * @param parentMap the HashNode map of children and their parents
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from
	 *   start to goal (including both start and goal).
	 */
	private List<GeographicPoint>
	reconstructPath(HashMap<MapNode,MapNode> parentMap,
					MapNode start, MapNode goal)
	{
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapNode current = goal;

		while (!current.equals(start)) {
			path.addFirst(current.getLocation());
			current = parentMap.get(current);
		}

		// add start
		path.addFirst(start.getLocation());
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
		// TODO: Implement this method in WEEK 4
		System.out.println("DIJKSTRA STARTED------------------------------");
		
		// Setup - check validity of inputs
		if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		MapNode startNode = pointNodeMap.get(start);
		MapNode endNode = pointNodeMap.get(goal);
		if (startNode == null) {
			System.err.println("Start node " + start + " does not exist");
			return null;
		}
		if (endNode == null) {
			System.err.println("End node " + goal + " does not exist");
			return null;
		}

		// setup to begin BFS
		List<GeographicPoint> path = new ArrayList<GeographicPoint>();

		HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		PriorityQueue<MapNodeDistance> pq = new PriorityQueue<MapNodeDistance>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		
		MapNodeDistance nd = new MapNodeDistance(startNode, 0);
		pq.add(nd);
		int counter = 0;
		while(!pq.isEmpty()) {
			nd = pq.poll();
			counter++;
			//System.out.println(nd);
			GeographicPoint gp = nd.getNode().getLocation();
			double length = nd.getDistanceFromStart();
			if(!visited.contains(nd)) {
				visited.add(nd.getNode());
				nodeSearched.accept(gp);
				if(gp.equals(goal)) {
					System.out.println("BINGO!!!");
					//System.out.println("visited size: " + visited.size());
					break;
				}
				Set<MapEdge> neighbours = nd.getNode().getEdges();
				if(neighbours != null && neighbours.size() > 0) {
					for(MapEdge edge : neighbours) {
						//System.out.println("Edge Road Type: " + edge.getRoadType());
						GeographicPoint neighbour_gp = edge.getEndPoint();
						MapNode neighbour_node = pointNodeMap.get(neighbour_gp);
						double curr_length = length + edge.getLength();
						//curr_length = lengthConversionByType(curr_length, edge.getRoadType());
						//System.out.println(length);
						if(!visited.contains(neighbour_node)) {
							MapNodeDistance nnd = new MapNodeDistance(neighbour_node, curr_length);
							pq.add(nnd);
							if(!parentMap.containsKey(neighbour_node)) {
								parentMap.put(nnd.getNode(), nd.getNode());
							} else {
								//System.out.println("vec postoji :)");
								//parentMap.put(nnd.getNode(), nd.getNode());
							}
							if(pq.size() < 10) {
								//System.out.println(pq);
							}
						} 
					}
				}	
			}
			
		}
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		path = findPath(start, goal, parentMap);
		System.out.println("DIJKSTRA counter: " + counter);
		return path;
	}
	
	private double lengthConversionByType(double curr_length, String roadType) {
		// TODO Auto-generated method stub
		if(roadType.equals("primary")) {
			return curr_length;
		} else if(roadType.equals("secondary")) {
			return 1.2 * curr_length;
		}  else if(roadType.equals("tertiary")) {
			return 1.3 * curr_length;
		} else if(roadType.equals("residential")) {
			return 1.5 * curr_length;
		} else if(roadType.equals("living_street")) {
			return 1.7 * curr_length;
		} else if (roadType.equals("unclassified")) {
			return 2 * curr_length;
		} else {
			return 3 * curr_length;
		}
	}

	private List<GeographicPoint> findPath(GeographicPoint start, GeographicPoint goal, HashMap<MapNode, MapNode> parentMap) {
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		if(parentMap.size() > 0) {
			GeographicPoint curr_gp = goal;
			path.addFirst(goal);
			MapNode curr_mn = pointNodeMap.get(curr_gp);
			int i = 0;
			while(!curr_gp.equals(start)) {
				curr_mn = parentMap.get(curr_mn);
				curr_gp = curr_mn.getLocation();
				path.addFirst(curr_gp);
				if(i>10) {
					break;
				}
				i++;
			}
		}
		return (List)path;
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
		System.out.println("A STAR STARTED------------------------------");
		
		// Setup - check validity of inputs
		if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		MapNode startNode = pointNodeMap.get(start);
		MapNode endNode = pointNodeMap.get(goal);
		if (startNode == null) {
			System.err.println("Start node " + start + " does not exist");
			return null;
		}
		if (endNode == null) {
			System.err.println("End node " + goal + " does not exist");
			return null;
		}

		// setup to begin BFS
		List<GeographicPoint> path = new ArrayList<GeographicPoint>();

		HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		PriorityQueue<MapNodeDistance> pq = new PriorityQueue<MapNodeDistance>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		
		MapNodeDistance nd = new MapNodeDistance(startNode, 0);
		pq.add(nd);
		int counter = 0;
		while(!pq.isEmpty()) {
			nd = pq.poll();
			counter++;
			//System.out.println(nd);
			GeographicPoint gp = nd.getNode().getLocation();
			if(!visited.contains(nd)) {
				visited.add(nd.getNode());
				nodeSearched.accept(gp);
				if(gp.equals(goal)) {
					System.out.println("BINGO!!!");
					//System.out.println("visited size: " + visited.size());
					break;
				}
				Set<MapEdge> neighbours = nd.getNode().getEdges();
				if(neighbours != null && neighbours.size() > 0) {
					for(MapEdge edge : neighbours) {
						//System.out.println(edge);
						GeographicPoint neighbour_gp = edge.getEndPoint();
						MapNode neighbour_node = pointNodeMap.get(neighbour_gp);
						//System.out.println(length);
						if(!visited.contains(neighbour_node)) {
							double curr_length = neighbour_node.getLocation().distance(start) + neighbour_node.getLocation().distance(goal);
							//curr_length = lengthConversionByType(curr_length, edge.getRoadType());
							MapNodeDistance nnd = new MapNodeDistance(neighbour_node, curr_length);
							pq.add(nnd);
							if(!parentMap.containsKey(neighbour_node)) {
								parentMap.put(nnd.getNode(), nd.getNode());
							} else {
								//System.out.println("vec postoji :)");
								//parentMap.put(nnd.getNode(), nd.getNode());
							}
							if(pq.size() < 10) {
								//System.out.println(pq);
							}
						} 
					}
				}	
			}
			
		}
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		path = findPath(start, goal, parentMap);
		System.out.println("A STAR COUNTER: " + counter);
		return path;
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
	}
	
	public List<GeographicPoint> dijkstraRoadType(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstraRoadType(start, goal, temp);
	}

	public List<GeographicPoint> dijkstraRoadType(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		System.out.println("DIJKSTRA BY ROAD TYPE STARTED------------------------------");
		
		// Setup - check validity of inputs
		if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		MapNode startNode = pointNodeMap.get(start);
		MapNode endNode = pointNodeMap.get(goal);
		if (startNode == null) {
			System.err.println("Start node " + start + " does not exist");
			return null;
		}
		if (endNode == null) {
			System.err.println("End node " + goal + " does not exist");
			return null;
		}

		// setup to begin BFS
		List<GeographicPoint> path = new ArrayList<GeographicPoint>();

		HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		PriorityQueue<MapNodeDistance> pq = new PriorityQueue<MapNodeDistance>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		
		MapNodeDistance nd = new MapNodeDistance(startNode, 0);
		pq.add(nd);
		int counter = 0;
		while(!pq.isEmpty()) {
			nd = pq.poll();
			counter++;
			//System.out.println(nd);
			GeographicPoint gp = nd.getNode().getLocation();
			double length = nd.getDistanceFromStart();
			
			if(!visited.contains(nd)) {
				visited.add(nd.getNode());
				nodeSearched.accept(gp);
				if(gp.equals(goal)) {
					System.out.println("BINGO!!!");
					//System.out.println("visited size: " + visited.size());
					break;
				}
				Set<MapEdge> neighbours = nd.getNode().getEdges();
				if(neighbours != null && neighbours.size() > 0) {
					for(MapEdge edge : neighbours) {
						//System.out.println("Edge Road Type: " + edge.getRoadType());
						GeographicPoint neighbour_gp = edge.getEndPoint();
						MapNode neighbour_node = pointNodeMap.get(neighbour_gp);
						double curr_length = length + edge.getLength();
						curr_length = lengthConversionByType(curr_length, edge.getRoadType());
						//System.out.println(length);
						if(!visited.contains(neighbour_node)) {
							MapNodeDistance nnd = new MapNodeDistance(neighbour_node, curr_length);
							pq.add(nnd);
							if(!parentMap.containsKey(neighbour_node)) {
								parentMap.put(nnd.getNode(), nd.getNode());
							} else {
								//System.out.println("vec postoji :)");
								//parentMap.put(nnd.getNode(), nd.getNode());
							}
							if(pq.size() < 10) {
								//System.out.println(pq);
							}
						} 
					}
				}	
			}
			
		}
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		path = findPath(start, goal, parentMap);
		System.out.println("DIJKSTRA counter: " + counter);
		return path;
	}
	
		public List<GeographicPoint> aStarSearchRoadType(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearchRoadType(start, goal, temp);
	}
	
	public List<GeographicPoint> aStarSearchRoadType(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		System.out.println("A STAR BY ROAD TYPE STARTED------------------------------");
		
		// Setup - check validity of inputs
		if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		MapNode startNode = pointNodeMap.get(start);
		MapNode endNode = pointNodeMap.get(goal);
		if (startNode == null) {
			System.err.println("Start node " + start + " does not exist");
			return null;
		}
		if (endNode == null) {
			System.err.println("End node " + goal + " does not exist");
			return null;
		}

		// setup to begin BFS
		List<GeographicPoint> path = new ArrayList<GeographicPoint>();

		HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		PriorityQueue<MapNodeDistance> pq = new PriorityQueue<MapNodeDistance>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		
		MapNodeDistance nd = new MapNodeDistance(startNode, 0);
		pq.add(nd);
		int counter = 0;
		while(!pq.isEmpty()) {
			nd = pq.poll();
			counter++;
			//System.out.println(nd);
			GeographicPoint gp = nd.getNode().getLocation();
			if(!visited.contains(nd)) {
				visited.add(nd.getNode());
				nodeSearched.accept(gp);
				if(gp.equals(goal)) {
					System.out.println("BINGO!!!");
					//System.out.println("visited size: " + visited.size());
					break;
				}
				Set<MapEdge> neighbours = nd.getNode().getEdges();
				if(neighbours != null && neighbours.size() > 0) {
					for(MapEdge edge : neighbours) {
						//System.out.println(edge);
						GeographicPoint neighbour_gp = edge.getEndPoint();
						MapNode neighbour_node = pointNodeMap.get(neighbour_gp);
						//System.out.println(length);
						if(!visited.contains(neighbour_node)) {
							double curr_length = neighbour_node.getLocation().distance(start) + neighbour_node.getLocation().distance(goal);
							curr_length = lengthConversionByType(curr_length, edge.getRoadType());
							MapNodeDistance nnd = new MapNodeDistance(neighbour_node, curr_length);
							pq.add(nnd);
							if(!parentMap.containsKey(neighbour_node)) {
								parentMap.put(nnd.getNode(), nd.getNode());
							} else {
								//System.out.println("vec postoji :)");
								//parentMap.put(nnd.getNode(), nd.getNode());
							}
							if(pq.size() < 10) {
								//System.out.println(pq);
							}
						} 
					}
				}	
			}
			
		}
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		path = findPath(start, goal, parentMap);
		System.out.println("A STAR COUNTER: " + counter);
		return path;
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
	}
	
	public List<GeographicPoint> findPathForMultiplePoints(List<GeographicPoint> points, int algorithmType) {
		List<GeographicPoint> path = new LinkedList<GeographicPoint>();
		if(points.size() < 2) {
			System.out.println("There was no start and goal geographic point");
			return path;
		} else {
			path = callAlgorithm(points, algorithmType);
		}
		return path;
	}
	
	private List<GeographicPoint> callMonoAlgorithm(List<GeographicPoint> points, int algorithmType) {
		List<GeographicPoint> path = new LinkedList<GeographicPoint>();
		if(algorithmType == 1) {
			path = dijkstra(path.get(0), path.get(1));
		}  else if(algorithmType == 2) {
			path = aStarSearch(path.get(0), path.get(1));
		} else {
			System.out.println("Choose 1 for Dijkstra and 2 for A star. Input " + algorithmType + " was invalid!.");
		} 
		return path;
	}
	
	private List<GeographicPoint> callAlgorithm(List<GeographicPoint> points, int algorithmType) {
		List<GeographicPoint> path = new LinkedList<GeographicPoint>();
		int count = 0;
		if(algorithmType == 1) {
			for(int i = 0; i < points.size() - 1; i++) {
				List<GeographicPoint> new_path = dijkstra(points.get(i), points.get(i+1));
				for(int j = 0; j < new_path.size(); j++) {
					if(count != 0 && j == 0) {
						continue;
					} 
					path.add(new_path.get(j));
				}
				count++;
			}
		}  else if(algorithmType == 2) {
			for(int i = 0; i < points.size() - 1; i++) {
				List<GeographicPoint> new_path = aStarSearch(points.get(i), points.get(i+1));
				for(int j = 0; j < new_path.size(); j++) {
					if(count != 0 && j == 0) {
						continue;
					}
					path.add(new_path.get(j));
				}
				count++;
			}
		} else {
			System.out.println("Choose 1 for Dijkstra and 2 for A star. Input " + algorithmType + " was invalid!.");
		} 
		return path;
	}
	
	public static void main(String[] args)
	{
		/*
		 * System.out.print("Making a new map..."); MapGraph theMap = new MapGraph();
		 * System.out.print("DONE. \nLoading the map...");
		 * GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		 * System.out.println("DONE.");
		 */
		
		// You can use this method for testing.  
		
		// Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint p1 = new GeographicPoint(32.8637793, -117.2222323);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		System.out.println("DIJKSTRA route:");
		for(GeographicPoint gp : route) {
			System.out.println(gp);
		}
		
		System.out.println("A STAR route:");
		for(GeographicPoint gp : route2) {
			System.out.println(gp);
		}
		
		System.out.println();
		System.out.println("********************************************************************************************************");
		System.out.println();
		
		List<GeographicPoint> points = new LinkedList<GeographicPoint>();
		points.add(start);
		points.add(p1);
		points.add(end);
		System.out.println("Points size: " + points.size());
		
		route.clear();
		route = theMap.findPathForMultiplePoints(points, 1);
		
		route2.clear();
		route2 = theMap.findPathForMultiplePoints(points, 2);
		
		System.out.println("DIJKSTRA route:");
		for(GeographicPoint gp : route) {
			System.out.println(gp);
		}
		
		System.out.println("A STAR route:");
		for(GeographicPoint gp : route2) {
			System.out.println(gp);
		}
		
		System.out.println();
		System.out.println("********************************************************************************************************");
		System.out.println("By Road Type");
		
		
		route.clear();
		route = theMap.dijkstraRoadType(start, end);
		
		route2.clear();
		route2 = theMap.aStarSearchRoadType(start, end);
		
		System.out.println("DIJKSTRA route:");
		for(GeographicPoint gp : route) {
			System.out.println(gp);
		}
		
		System.out.println("A STAR route:");
		for(GeographicPoint gp : route2) {
			System.out.println(gp);
		}
	}
	
}
