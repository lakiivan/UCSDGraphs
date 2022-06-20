package roadgraph;

public class MapNodeDistance implements Comparable<MapNodeDistance>{
	private MapNode node;
	private double distanceFromStart;

	
	public MapNodeDistance(MapNode node, double distanceFromStart) {
		super();
		this.node = node;
		this.distanceFromStart = distanceFromStart;
	}

	public MapNode getNode() {
		return node;
	}

	public void setNode(MapNode node) {
		this.node = node;
	}

	public double getDistanceFromStart() {
		return distanceFromStart;
	}

	public void setDistanceFromStart(double distanceFromStart) {
		this.distanceFromStart = distanceFromStart;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		long temp;
		temp = Double.doubleToLongBits(distanceFromStart);
		result = prime * result + (int) (temp ^ (temp >>> 32));
		return result;
	}

	@Override
	public String toString() {
		StringBuilder builder = new StringBuilder();
		builder.append(", distanceFromStart=");
		builder.append(distanceFromStart);
		builder.append("]");
		return builder.toString();
	}

	@Override
	public int compareTo(MapNodeDistance o) {
		return o.getDistanceFromStart() > this.getDistanceFromStart() ? -1 : 1;
	}

}
