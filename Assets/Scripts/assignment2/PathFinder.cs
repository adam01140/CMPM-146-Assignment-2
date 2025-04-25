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

    // Insert entry into frontier maintaining sorted order by fScore
    private static void InsertSorted(List<AStarEntry> frontier, AStarEntry entry)
    {
        for (int i = 0; i < frontier.Count; i++)
        {
            if (entry.fScore < frontier[i].fScore)
            {
                frontier.Insert(i, entry);
                return;
            }
        }
        frontier.Add(entry);
    }

    public static (List<Vector3>, int) AStar(GraphNode start, GraphNode destination, Vector3 target)
    {
        // Initialize data structures
        Dictionary<int, AStarEntry> allNodes = new Dictionary<int, AStarEntry>();
        List<AStarEntry> frontier = new List<AStarEntry>();
        HashSet<int> explored = new HashSet<int>();
        int nodesExpanded = 0;

        // Create initial entry
        AStarEntry startEntry = new AStarEntry(
            start,
            null,
            null,
            0,
            Heuristic(start.GetCenter(), target)
        );

        // Add start node to frontier and allNodes
        frontier.Add(startEntry);
        allNodes[start.GetID()] = startEntry;

        while (frontier.Count > 0)
        {
            // Get node with lowest f-score
            AStarEntry current = frontier[0];
            frontier.RemoveAt(0);

            // If we reached the destination
            if (current.node == destination)
            {
                List<Vector3> path = new List<Vector3>();
                AStarEntry pathNode = current;

                // Build path backwards from destination
                while (pathNode.cameFrom != null)
                {
                    path.Insert(0, pathNode.wallUsed.midpoint);
                    pathNode = allNodes[pathNode.cameFrom.GetID()];
                }

                // Add final target position
                path.Add(target);
                return (path, nodesExpanded);
            }

            // Mark node as explored
            explored.Add(current.node.GetID());
            nodesExpanded++;

            // Expand neighbors
            foreach (GraphNeighbor neighbor in current.node.GetNeighbors())
            {
                GraphNode nextNode = neighbor.GetNode();
                int nextID = nextNode.GetID();

                // Skip if already explored
                if (explored.Contains(nextID))
                    continue;

                // Calculate new g-score
                float newGScore = current.gScore + Vector3.Distance(
                    current.node.GetCenter(),
                    nextNode.GetCenter()
                );

                // Calculate f-score for this neighbor
                float newFScore = newGScore + Heuristic(nextNode.GetCenter(), target);

                // Check if we should add or update this neighbor
                if (!allNodes.ContainsKey(nextID))
                {
                    // New node - add to frontier
                    AStarEntry newEntry = new AStarEntry(
                        nextNode,
                        current.node,
                        neighbor.GetWall(),
                        newGScore,
                        newFScore
                    );
                    InsertSorted(frontier, newEntry);
                    allNodes[nextID] = newEntry;
                }
                else
                {
                    // Existing node - update if this path is better
                    AStarEntry existingEntry = allNodes[nextID];
                    if (newGScore < existingEntry.gScore)
                    {
                        // Remove from frontier if present
                        frontier.Remove(existingEntry);
                        
                        existingEntry.cameFrom = current.node;
                        existingEntry.wallUsed = neighbor.GetWall();
                        existingEntry.gScore = newGScore;
                        existingEntry.fScore = newFScore;

                        InsertSorted(frontier, existingEntry);
                    }
                }
            }
        }

        // No path found
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

        // find start and destination nodes in graph
        GraphNode start = null;
        GraphNode destination = null;
        foreach (var n in graph.all_nodes)
        {
            if (Util.PointInPolygon(transform.position, n.GetPolygon()))
            {
                start = n;
            }
            if (Util.PointInPolygon(target, n.GetPolygon()))
            {
                destination = n;
            }
        }
        if (destination != null)
        {
            // only find path if destination is inside graph
            EventBus.ShowTarget(target);
            (List<Vector3> path, int expanded) = PathFinder.AStar(start, destination, target);

            Debug.Log("found path of length " + path.Count + " expanded " + expanded + " nodes, out of: " + graph.all_nodes.Count);
            EventBus.SetPath(path);
        }
        

    }

    

 
}
