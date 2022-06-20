
/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents an Edge which connects two connected Vertices
 * Nodes in the graph are intersections between
 * It holds all relevant information for the edge, from and to Geographic point, road type, road name, and length
 * It has only getters and setters methods and toString  
 *
 */

package roadgraph;

import geography.GeographicPoint;

public class Edge {
	private GeographicPoint from;
	private GeographicPoint to;
	private String roadName;
	private String roadType;
	double length;
	
	public Edge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double length) {
		super();
		this.from = from;
		this.to = to;
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
	}

	@Override
	public String toString() {
		StringBuilder builder = new StringBuilder();
		builder.append("Edge [from=");
		builder.append(from);
		builder.append(", to=");
		builder.append(to);
		builder.append(", roadName=");
		builder.append(roadName);
		builder.append(", roadType=");
		builder.append(roadType);
		builder.append(", length=");
		builder.append(length);
		builder.append("]");
		return builder.toString();
	}

	public GeographicPoint getFrom() {
		return from;
	}


	public void setFrom(GeographicPoint from) {
		this.from = from;
	}


	public GeographicPoint getTo() {
		return to;
	}


	public void setTo(GeographicPoint to) {
		this.to = to;
	}


	public String getRoadName() {
		return roadName;
	}


	public void setRoadName(String roadName) {
		this.roadName = roadName;
	}


	public String getRoadType() {
		return roadType;
	}


	public void setRoadType(String roadType) {
		this.roadType = roadType;
	}


	public double getLength() {
		return length;
	}


	public void setLength(double length) {
		this.length = length;
	}

	
	
}
