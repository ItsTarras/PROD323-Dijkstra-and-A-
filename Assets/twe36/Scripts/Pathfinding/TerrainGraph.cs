namespace twe36
{
    using System.Collections.Generic;
    using Unity.VisualScripting.Dependencies.Sqlite;
    using UnityEngine;

    public class TerrainGraph : MonoBehaviour
    {
        private TerrainData tData;
        private int tWidth;
        private int tLength;
        private float gridOffset = 0.5f;
        public bool allowHoles = false;
        public Node[,] grid;
        public float[,,] cost;

        private float maxHeight = 5.1f; // If node cost is over 5, it is considered impassable
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
                        cost[x, z, 2] = Mathf.Abs(grid[x, z].nodeHeight - grid[x, z + 1].nodeHeight); // north of current node  
                        cost[x, z, 3] = Mathf.Abs(grid[x, z].nodeHeight - grid[x + 1, z + 1].nodeHeight); // north-east of current node  
                        cost[x, z, 4] = Mathf.Abs(grid[x, z].nodeHeight - grid[x + 1, z].nodeHeight); // east of current node  
                        cost[x, z, 5] = Mathf.Abs(grid[x, z].nodeHeight - grid[x + 1, z - 1].nodeHeight); // south-east of current node  
                        cost[x, z, 6] = Mathf.Abs(grid[x, z].nodeHeight - grid[x, z - 1].nodeHeight); // south of current node  
                        cost[x, z, 7] = Mathf.Abs(grid[x, z].nodeHeight - grid[x - 1, z - 1].nodeHeight); // south-west of current node  
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
                new Vector2(-1, 0), // west
                new Vector2(-1, 1), // north-west
                new Vector2(0, 1),  // north
                new Vector2(1, 1),  // north-east
                new Vector2(1, 0),  // east
                new Vector2(1, -1), // south-east
                new Vector2(0, -1), // south
                new Vector2(-1, -1) // south-west
            };

            // Find all nodes via the 8 directions
            foreach (Vector2 dir in directions)
            {
                Vector2 v = new Vector2(dir.x, dir.y) + new Vector2(n.nodePosition.X, n.nodePosition.Y);

                // Check if the neighbouring node actually exist in the terrain
                // y here is actually the z
                //Debug.Log(v.x >= 0 && v.x < tWidth);
                bool doExist = (v.x >= 0 && v.x < tWidth && v.y >= 0 && v.y < tLength) ? true : false;

                

                if (doExist)
                {
                    float slope = PathAlgorithm.CalculateSlope(n, grid[(int)v.x, (int)v.y]);
                    // Check if the neighbouring node is too high. If it is, deem it impassable.

                    //ALSO CHECK THE SLOPE!!!
                    float neighbourHeight = grid[(int)v.x, (int)v.y].nodeHeight;

                    //If there is no collision beneath us.
                    RaycastHit hit;

                    if(!allowHoles)
                    {
                        if (!(Physics.Raycast(new Vector3((int)v.x, neighbourHeight + 1f, (int)v.y), Vector3.down, out hit, 5f) && (slope < 90)))
                        {
                            // If the raycast hits a collider, there is an object above the node
                            return new List<Node>();
                        }
                    }


                    bool passable = (neighbourHeight < maxHeight && neighbourHeight < n.nodeHeight + 0.75f);

                    //If this node exists
                    if (passable)
                    {
                        //Get a weighted average or something.
                        neighbours.Add(grid[(int)v.x, (int)v.y]);
                    }
                }
                
            }

            return neighbours;
        }

        // This function searches and returns the least cost among all the 8 neighbors of a node
        public float NextMinimumCost(Node n)
        {
            float minCost = 5000f; // dummy value
            List<Node> neighbours = GetNeighbours(n);
            float modifier = 1f;

            if (neighbours.Count == 8)
            {
                //Remove weighting from surfaces away from holes.
                modifier = 0.5f;
            }
            float weightedAverage = 5000f;
            for (int index = 0; index < 8; index++)
            {
                //Add a slope calculation or something
                float thisCost = cost[(int)n.nodePosition.X, (int)n.nodePosition.Y, index];

                if (neighbours.Count > index)
                {
                    float slope = PathAlgorithm.CalculateSlope(n, neighbours[index]);
                    float normalSlope = slope / 180f;

                    /*
                    foreach (Node node in GetNeighbours(neighbours[index]))
                    {
                        float cost = PathAlgorithm.CalculateSlope(neighbours[index], node);
                        if (cost > 35 || cost < 0)
                            thisCost += cost;
                    }
                    */


                    Vector3 nodePoint = new Vector3(n.nodePosition.X, n.nodeHeight, n.nodePosition.Y);

                    //Apply a raycast to check if there is an object above this node. If there is, return the maximum cost of 5000f.
                    LayerMask layerMask = ~LayerMask.GetMask("Myself");

                    // Get the layer index of "Myself"
                    int myselfLayerIndex = LayerMask.NameToLayer("Myself");

                    // Shift 1 to the left by the layer index to create a bitmask for the "Myself" layer
                    int myselfLayerMask = 1 << myselfLayerIndex;

                    // Invert the bitmask to exclude the "Myself" layer
                    layerMask &= ~myselfLayerMask;

                    RaycastHit hit;
                    if (Physics.Raycast(nodePoint, Vector3.up, out hit, 1f, layerMask))
                    {
                        // If the raycast hits a collider, there is an object above the node
                        thisCost = 5000f;
                    }



                    // Convert world space to terrain space

                    //Vector3 nodeNormal = tData.GetInterpolatedNormal((int)n.nodePosition.X, (int)n.nodePosition.Y);
                    float weightedSlope = 0f;

                    if (normalSlope < 90 && normalSlope > 25)
                    {
                        weightedSlope = 1f * normalSlope;
                    }
                    else
                    {
                        weightedSlope = 2.5f * normalSlope;
                    }
                    
                    float weightedCost = 1f * thisCost;


                    weightedAverage = weightedSlope + weightedCost;
                    weightedAverage *= modifier;
                    //Calculate based on a mixture of cost and slope.
                    if (weightedAverage < minCost)
                    {
                        minCost = weightedAverage;
                    }
                }                
            }

            // Since graph is a tile grid, horizontal cost is 1
            //Debug.Log("Next cost: " + minCost);
            return (minCost) + 1;
        }
    }
}
