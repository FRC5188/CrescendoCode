package frc.robot.util.autonomous.graph;

import edu.wpi.first.math.geometry.Pose2d;

public class Node {
    private Pose2d _pose;
    private Node[] _connectedNodes;

    public Node(Pose2d pose) {
        this(pose, null);
    }

    public Node(Pose2d pose, Node[] connectedNodes) {
        this._pose = pose;
        this._connectedNodes = connectedNodes;
    }

    public void removeConnection(Node otherNode) {
        // Find the index of the node that we want to remove then shift everything left.
        for (int i = 0; i < this._connectedNodes.length; i++) {
            if (this._connectedNodes[i] == otherNode) {
                for (int j = i; j < this._connectedNodes.length - 1; j++) {
                    this._connectedNodes[j] = this._connectedNodes[j + 1];
                }
                this._connectedNodes[this._connectedNodes.length - 1] = null;
                break;
            }
        }
    }

    public Node[] getConnections() {
        return this._connectedNodes;
    }

    public boolean isConnected(Node otherNode) {
        for (Node node : this._connectedNodes) {
            if (node == otherNode) {
                return true;
            }
        }
        return false;
    }

    public void addConnection(Node otherNode) {
        this._connectedNodes[this._connectedNodes.length] = otherNode;
    }

    public void addBiDirectionalConnection(Node otherNode) {
        this.addConnection(otherNode);
        otherNode.addConnection(this);
    }

    public Pose2d get() {
        return this._pose;
    }

    public boolean equals(Node otherNode) {
        return this._pose.equals(otherNode.get());
    }

    public String toString() {
        return this._pose.toString();
    }

    public Node clone() {
        return new Node(this._pose, this._connectedNodes);
    }
}
