using UnityEngine;
using System.Collections.Generic;

public class PathFinder : MonoBehaviour
{
    // Assignment 2: Implement AStar
    //
    // DO NOT CHANGE THIS SIGNATURE (parameter types + return type)
    // AStar will be given the start node, destination node and the target position, and should return 
    // a path as a list of positions the agent has to traverse to reach its destination, as well as the
    // number of nodes that were expanded to find this path
    // The last entry of the path will be the target position, and you can also use it to calculate the heuristic
    // value of nodes you add to your search frontier; the number of expanded nodes tells us if your search was
    // efficient
    //
    // Take a look at StandaloneTests.cs for some test cases

    // Helper class to store A* search information
    private class AStarEntry
    {
        public GraphNode node;           // Current node
        public GraphNode cameFrom;       // Previous node
        public Wall wallUsed;           // Wall used to get here
        public float gScore;            // Cost from start to this node
        public float fScore;            // Total estimated cost (gScore + heuristic)

        public AStarEntry(GraphNode node, GraphNode cameFrom, Wall wallUsed, float gScore, float fScore)
        {
            this.node = node;
            this.cameFrom = cameFrom;
            this.wallUsed = wallUsed;
            this.gScore = gScore;
            this.fScore = fScore;
        }
    }

    // Calculate heuristic (straight-line distance)
    private static float Heuristic(Vector3 current, Vector3 target)
    {
        return Vector3.Distance(current, target);
    }

    public static (List<Vector3>, int) AStar(GraphNode start, GraphNode destination, Vector3 target)
    {
        var frontier = new List<AStarEntry>();
        var explored = new HashSet<int>();
        var nodeInfo = new Dictionary<int, AStarEntry>();
        int nodesExpanded = 0;

        // Initialize start node
        var startEntry = new AStarEntry(
            start,
            null,
            null,
            0,
            Heuristic(start.GetCenter(), target)
        );
        frontier.Add(startEntry);
        nodeInfo[start.GetID()] = startEntry;

        while (frontier.Count > 0)
        {
            // Get node with lowest f-score
            var current = frontier[0];
            frontier.RemoveAt(0);

            if (current.node == destination)
            {
                // Build path from destination to start
                var path = new List<Vector3>();
                var currentNode = current;

                while (currentNode.cameFrom != null)
                {
                    path.Insert(0, currentNode.wallUsed.midpoint);
                    currentNode = nodeInfo[currentNode.cameFrom.GetID()];
                }
                
                path.Add(target);
                return (path, nodesExpanded);
            }

            int currentId = current.node.GetID();
            if (explored.Contains(currentId))
                continue;

            explored.Add(currentId);
            nodesExpanded++;

            // Expand neighbors
            foreach (var neighbor in current.node.GetNeighbors())
            {
                var nextNode = neighbor.GetNode();
                int nextId = nextNode.GetID();

                if (explored.Contains(nextId))
                    continue;

                float newGScore = current.gScore + Vector3.Distance(
                    current.node.GetCenter(),
                    nextNode.GetCenter()
                );

                if (!nodeInfo.TryGetValue(nextId, out var existingEntry) || newGScore < existingEntry.gScore)
                {
                    var newFScore = newGScore + Heuristic(nextNode.GetCenter(), target);
                    var newEntry = existingEntry;

                    if (existingEntry != null)
                    {
                        frontier.Remove(existingEntry);
                        existingEntry.cameFrom = current.node;
                        existingEntry.wallUsed = neighbor.GetWall();
                        existingEntry.gScore = newGScore;
                        existingEntry.fScore = newFScore;
                        newEntry = existingEntry;
                    }
                    else
                    {
                        newEntry = new AStarEntry(
                            nextNode,
                            current.node,
                            neighbor.GetWall(),
                            newGScore,
                            newFScore
                        );
                        nodeInfo[nextId] = newEntry;
                    }

                    // Insert maintaining sorted order by fScore
                    int insertIndex = 0;
                    while (insertIndex < frontier.Count && frontier[insertIndex].fScore <= newEntry.fScore)
                        insertIndex++;
                    frontier.Insert(insertIndex, newEntry);
                }
            }
        }

        // No path found - return direct path to target
        return (new List<Vector3>() { target }, nodesExpanded);
    }

    public Graph graph;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        EventBus.OnTarget += PathFind;
        EventBus.OnSetGraph += SetGraph;
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void SetGraph(Graph g)
    {
        graph = g;
    }

    // entry point
    public void PathFind(Vector3 target)
    {
        if (graph == null) return;

        GraphNode start = null;
        GraphNode destination = null;

        foreach (var n in graph.all_nodes)
        {
            if (Util.PointInPolygon(transform.position, n.GetPolygon()))
                start = n;
            if (Util.PointInPolygon(target, n.GetPolygon()))
                destination = n;
        }

        if (start != null && destination != null)
        {
            var (path, _) = AStar(start, destination, target);
            EventBus.SetPath(path);
        }
    }

    

 
}
