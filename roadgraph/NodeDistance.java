package roadgraph;

import geography.GeographicPoint;

public class NodeDistance implements Comparable{
	private GeographicPoint node;
	private double distanceFromStart;
	
	public NodeDistance(GeographicPoint node, double distanceFromStart) {
		super();
		this.node = node;
		this.distanceFromStart = distanceFromStart;
	}
	
	public GeographicPoint getNode() {
		return node;
	}

	public void setNode(GeographicPoint node) {
		this.node = node;
	}

	public double getDistanceFromStart() {
		return distanceFromStart;
	}

	public void setDistanceFromStart(int distanceFromStart) {
		this.distanceFromStart = distanceFromStart;
	}

	@Override
	public int compareTo(Object o) {
		// TODO Auto-generated method stub
		/*
		 * if(!(o instanceof NodeDistance) || o == null) { return 0; }
		 */
		return ((NodeDistance) o).getDistanceFromStart() > this.getDistanceFromStart() ? -1 : 1;
	}

	@Override
	public String toString() {
		StringBuilder builder = new StringBuilder();
		builder.append("NodeDistance [node=");
		builder.append(node);
		builder.append(", distanceFromStart=");
		builder.append(distanceFromStart);
		builder.append("]");
		return builder.toString();
	}
	
	

}
