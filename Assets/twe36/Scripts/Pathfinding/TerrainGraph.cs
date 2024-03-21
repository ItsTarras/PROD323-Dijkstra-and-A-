namespace YourUserCode
{
    using System.Collections.Generic;
    using UnityEngine;

    public class TerrainGraph : MonoBehaviour
    {
        private TerrainData tData;
        private int tWidth;
        private int tLength;
        private float gridOffset = 0.5f;

        public Node[,] grid;
        public float[,,] cost;

        private float maxHeight = 5f; // If node cost is over 5, it is considered impassable

        public TerrainGraph()
        {
            // Get reference of the active terrain on the scene
            tData = Terrain.activeTerrain.terrainData;

            // Create a representation of the graph using the terrain size
            // Taking the x (width) and z (length) values only
            // Grid offset points to the center of the node cell

            tWidth = Mathf.FloorToInt(tData.size.x);
            tLength = Mathf.FloorToInt(tData.size.z);
            grid = new Node[tWidth, tLength];
            cost = new float[tWidth, tLength, 8]; // cost towards each 8 direction from current node

            // Populate the grid with nodes
            for (int x = 0; x < tWidth; x++)
            {
                for (int z = 0; z < tLength; z++)
                {
                    grid[x, z] = new Node(x, z);
                    grid[x, z].nodeHeight = Terrain.activeTerrain.SampleHeight(new Vector3(x + gridOffset, 0, z + gridOffset));
                }
            }

            // Store costs (height difference between nodes) of the terrain in cost array
            for (int x = 0; x < tWidth; x++)
            {
                for (int z = 0; z < tLength; z++)
                {
                    if (x > 0 && z > 0 && x < tWidth - 1 && z < tLength - 1)
                    {
                        // TODO: Set the proper edge/connection cost for each of the 8 directions
                        cost[x, z, 0] = Mathf.Abs(grid[x, z].nodeHeight - grid[x - 1, z].nodeHeight); // west of current node  
                        cost[x, z, 1] = Mathf.Abs(grid[x, z].nodeHeight - grid[x - 1, z + 1].nodeHeight); // north-west of current node  
                        cost[x, z, 2] = Mathf.Abs(grid[x, z].nodeHeight - grid[x, z].nodeHeight); // north of current node  
                        cost[x, z, 3] = Mathf.Abs(grid[x, z].nodeHeight - grid[x, z].nodeHeight); // north-east of current node  
                        cost[x, z, 4] = Mathf.Abs(grid[x, z].nodeHeight - grid[x, z].nodeHeight); // east of current node  
                        cost[x, z, 5] = Mathf.Abs(grid[x, z].nodeHeight - grid[x, z].nodeHeight); // south-east of current node  
                        cost[x, z, 6] = Mathf.Abs(grid[x, z].nodeHeight - grid[x, z].nodeHeight); // south of current node  
                        cost[x, z, 7] = Mathf.Abs(grid[x, z].nodeHeight - grid[x, z].nodeHeight); // south-west of current node  
                    }
                }
            }
        }

        public List<Node> GetNeighbours(Node n)
        {
            List<Node> neighbours = new List<Node>();

            // TODO: Take all the nodes from all cardinal and ordinal directions
            // Assume current node is at Vector2(0,0)

            Vector2[] directions =
            {
                new Vector2(0, 0), // west
                new Vector2(0, 0), // north-west
                new Vector2(0, 0),  // north
                new Vector2(0, 0),  // north-east
                new Vector2(0, 0),  // east
                new Vector2(0, 0), // south-east
                new Vector2(0, 0), // south
                new Vector2(0, 0) // south-west
            };

            // Find all nodes via the 8 directions
            foreach (Vector2 dir in directions)
            {
                Vector2 v = new Vector2(dir.x, dir.y) + new Vector2(n.nodePosition.X, n.nodePosition.Y);

                // Check if the neighbouring node actually exist in the terrain
                // y here is actually the z
                bool doExist = (v.x >= 0 && v.x < tWidth && v.y >= 0 && v.y < tLength) ? true : false;

                // Check if the neighbouring node is too high. If it is, deem it impassable
                bool passable = grid[(int)v.x, (int)v.y].nodeHeight < maxHeight;

                if (doExist && passable)
                {
                    neighbours.Add(grid[(int)v.x, (int)v.y]);
                }
            }

            return neighbours;
        }

        // This function searches and returns the least cost among all the 8 neighbors of a node
        public float NextMinimumCost(Node n)
        {
            float minCost = 5000f; // dummy value

            for (int index = 0; index < 8; index++)
            {
                // TODO: Search for minimum vertical cost of current node
            }

            // Since graph is a tile grid, horizontal cost is 1
            return (minCost) + 1;
        }

    }

}
