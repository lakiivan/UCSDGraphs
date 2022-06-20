package roadgraph;

import geography.GeographicPoint;

public class NodePredictedDistance implements Comparable {
	private GeographicPoint node;
	private double predictedDistance;
	
	public NodePredictedDistance(GeographicPoint node, double predictedDistance) {
		super();
		this.node = node;
		this.predictedDistance = predictedDistance;
	}
	

	public GeographicPoint getNode() {
		return node;
	}

	public void setNode(GeographicPoint node) {
		this.node = node;
	}
	
		public double getPredictedDistance() {
		return predictedDistance;
	}


	public void setPredictedDistance(double predictedDistance) {
		this.predictedDistance = predictedDistance;
	}



	@Override
	public int compareTo(Object o) {
		// TODO Auto-generated method stub
		
		return  ((NodePredictedDistance) o).getPredictedDistance() > this.getPredictedDistance() ? -1 : 1;
	}


	/*
	 * @Override public String toString() { StringBuilder builder = new
	 * StringBuilder(); builder.append("NodePredictedDistance [node=");
	 * builder.append(node); builder.append(", predictedDistance=");
	 * builder.append(predictedDistance); builder.append("]"); return
	 * builder.toString(); }
	 */
	
	 	@Override public String toString() { 
	 		StringBuilder builder = new StringBuilder(); 
	 		builder.append("NodePredictedDistance ");
	 		builder.append(", pd=");
	 		builder.append(predictedDistance); 
	 		builder.append("]"); 
	 		return builder.toString(); }
}
