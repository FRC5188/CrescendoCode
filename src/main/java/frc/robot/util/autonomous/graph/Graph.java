package frc.robot.util.autonomous.graph;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Graph {
    private Node[] _nodes;

    public Graph(Node[] nodes) {
        this._nodes = nodes;
    }

    public Node[] getNodes() {
        return this._nodes;
    }

    public Node[] getPath(Node start, Node end, Node[] currentPath) {
        // We'll first make sure that the start node isn't the end node, if it is then we'll
        // just thrown a parameter exception.
        if (start.equals(end)) {
            throw new IllegalArgumentException("The Start & End Node CANNOT BE EQUAL");
        }

        // Next we'll see if the end node is a connection of the start node. If so then we'll return the end node.
        for (Node node : start.getConnections()) {
            if (node.equals(end)) {
                currentPath[currentPath.length] = end;
                return currentPath; // Recursion should end here.
            }
        }

        // Now that we know that neither of the above edge-cases are true then we'll find the node connected
        // to our start node that's closest to the end node.
        double distance = Double.MAX_VALUE;
        Node bestNode = null; // The node closest.
        for (Node node : start.getConnections()) {
            if (getDistanceBetween(node, end) < distance) {
                distance = getDistanceBetween(node, end);
                bestNode = node;
            }
        }

        // Now we've found the node closest to the end node, we'll call this recursively until we
        // one of the edge cases are fulfilled.
        if (currentPath.length == 0) {
            currentPath[0] = start;
        }

        currentPath[currentPath.length] = bestNode;
        getPath(bestNode, end, currentPath);
        return null; // Should never get here because the recursion should end before this.
    }

    public static Graph create2024Field() {
        Node[] nodes = new Node[21];

        nodes[0] = new Node(new Pose2d(0, 0, new Rotation2d(0)));
        nodes[1] = new Node(new Pose2d(0, 0, new Rotation2d(0)));
        nodes[2] = new Node(new Pose2d(0, 0, new Rotation2d(0)));
        nodes[3] = new Node(new Pose2d(0, 0, new Rotation2d(0)));
        nodes[4] = new Node(new Pose2d(0, 0, new Rotation2d(0)));
        nodes[5] = new Node(new Pose2d(0, 0, new Rotation2d(0)));
        nodes[6] = new Node(new Pose2d(0, 0, new Rotation2d(0)));
        nodes[7] = new Node(new Pose2d(0, 0, new Rotation2d(0)));
        nodes[8] = new Node(new Pose2d(0, 0, new Rotation2d(0)));
        nodes[9] = new Node(new Pose2d(0, 0, new Rotation2d(0)));
        nodes[10] = new Node(new Pose2d(0, 0, new Rotation2d(0)));
        nodes[11] = new Node(new Pose2d(0, 0, new Rotation2d(0)));
        nodes[12] = new Node(new Pose2d(0, 0, new Rotation2d(0)));
        nodes[13] = new Node(new Pose2d(0, 0, new Rotation2d(0)));
        nodes[14] = new Node(new Pose2d(0, 0, new Rotation2d(0)));
        nodes[15] = new Node(new Pose2d(0, 0, new Rotation2d(0)));
        nodes[16] = new Node(new Pose2d(0, 0, new Rotation2d(0)));

        nodes[0].addConnection(nodes[1]);
        nodes[0].addConnection(nodes[3]);

        nodes[1].addConnection(nodes[0]);
        nodes[1].addConnection(nodes[3]);

        nodes[2].addConnection(nodes[1]);
        nodes[2].addConnection(nodes[4]);

        nodes[3].addConnection(nodes[0]);
        nodes[3].addConnection(nodes[1]);
        nodes[3].addConnection(nodes[5]);

        nodes[4].addConnection(nodes[2]);
        nodes[4].addConnection(nodes[1]);
        nodes[4].addConnection(nodes[6]);

        nodes[5].addConnection(nodes[3]);
        nodes[5].addConnection(nodes[7]);
        nodes[5].addConnection(nodes[8]);

        nodes[6].addConnection(nodes[4]);
        nodes[6].addConnection(nodes[9]);
        nodes[6].addConnection(nodes[8]);

        nodes[7].addConnection(nodes[5]);
        nodes[7].addConnection(nodes[10]);
        nodes[7].addConnection(nodes[8]);

        nodes[8].addConnection(nodes[5]);
        nodes[8].addConnection(nodes[6]);
        nodes[8].addConnection(nodes[7]);
        nodes[8].addConnection(nodes[9]);
        nodes[8].addConnection(nodes[10]);
        nodes[8].addConnection(nodes[11]);

        nodes[9].addConnection(nodes[6]);
        nodes[9].addConnection(nodes[8]);
        nodes[9].addConnection(nodes[11]);

        nodes[10].addConnection(nodes[7]);
        nodes[10].addConnection(nodes[8]);
        nodes[10].addConnection(nodes[12]);

        nodes[11].addConnection(nodes[8]);
        nodes[11].addConnection(nodes[9]);
        nodes[11].addConnection(nodes[13]);

        nodes[12].addConnection(nodes[10]);
        nodes[12].addConnection(nodes[14]);
        nodes[12].addConnection(nodes[15]);

        nodes[13].addConnection(nodes[11]);
        nodes[13].addConnection(nodes[16]);
        nodes[13].addConnection(nodes[15]);

        nodes[14].addConnection(nodes[12]);
        nodes[14].addConnection(nodes[15]);

        nodes[15].addConnection(nodes[12]);
        nodes[15].addConnection(nodes[14]);
        nodes[15].addConnection(nodes[13]);
        nodes[15].addConnection(nodes[16]);

        return new Graph(nodes);
    }

    private static double getDistanceBetween(Node start, Node end) {
        return start.get().getTranslation().getDistance(end.get().getTranslation());
    }
}
