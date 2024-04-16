using System;
using System.Collections;
using System.Collections.Generic;
using twe36;

using UnityEngine;

public class Movement : MonoBehaviour
{

    //Movement variables.
    [Range(0, 10f)]
    public float moveForce = 1f; // Acceleration force to move in.
    private float maxLinearSpeed = 10f;
    private Rigidbody rb;
    private Vector3 moveInput;
    private bool finished = false;
    public float nodeAcceptanceRange = 1f; 
    //Introduce a variable to handle the ratios of the avoidance.
    [Range(0.0f, 1f)]
    public float avoidanceWeight = 0.5f;
    public bool generate = true;

    //Debug information.
    public bool nodeAdditionDebug = false;
    public bool debugVelocity = false;
    

    //Raycast variables.
    [Range(0, 5f)]
    public float raycastGroundDistance = 1f; // Distance of the raycast to detect ground.
    [Range(0, 5f)]
    public float raycastDistance = 3f;
    // Distance in front of the character to cast the avoidance raycasts. Keep small so it doesn't worry about ramps or somethign.
    public float avoidanceDistance = 0.5f; 
    //Stop worrying abvout ramps so adjust angle.
    [Range(0, 180)]
    public float avoidanceAngle = 45f;
    //Get the forward directions of the ball, relative to the world, plus rotation.
    RaycastHit forwardLeft;
    RaycastHit forwardRight;
    RaycastHit backward;
    RaycastHit forward;

    //Trap Logic
    public GameObject jointToDeactivate;


    public enum moveType
    {
        automatic,
        manual
    };

    //Path variables for the algorithm.
    private List<Node> path = new List<Node>(); // Paht for the object to follow.

    private List<Node> CurrentPath = new List<Node>(); // Paht for the object to follow.

    //visitedNodes list, with a dummy value to get started with.
    private List<Node> visitedPaths = new List<Node>
    {
        new Node(0, 0),
        new Node(0, 0),
        new Node(0, 0),
    };

    private Node currentTarget;


    private TerrainGraph graph; // A graph to store information.
    public Transform start; // The start transform node
    public Transform end; // The finish line.
    public moveType movementType;
    private float elapsedTime = 0f;

    void Start()
    {
        instantiateVariables();
        generatePath();
        debugPath();
        StartCoroutine(startCounter());
    }


    void instantiateVariables()
    {
        rb = GetComponent<Rigidbody>();
        graph = new TerrainGraph();
        start = transform;
        end = GameObject.FindGameObjectWithTag("goal").transform;
            //Path variables for the algorithm.
        path = new List<Node>(); // Paht for the object to follow.

        CurrentPath = new List<Node>(); // Paht for the object to follow.

        //visitedNodes list, with a dummy value to get started with.
        visitedPaths = new List<Node>
        {
            new Node(0, 0),
            new Node(0, 0),
            new Node(0, 0),
        };

        currentTarget = null;

}

void generatePath()
    {
        // Start node position
        int startX = (int)start.position.x;
        int startZ = (int)start.position.z;

        Debug.Log(graph.grid);
        Node startNode = graph.grid[startX, startZ];

        // End node position
        int endX = (int)end.position.x;
        int endZ = (int)end.position.z;

        Node endNode = graph.grid[endX, endZ];

        path = PathAlgorithm.AStar(graph, startNode, endNode, 2);
    }

    void debugPath()
    {
        //Debug the path.
        int i = 1;
        foreach (Node node in path)
        {
            Debug.Log("Node " + i + ": " + node.nodePosition);
        }
    }



    void Update()
    {
        //Playe rinput.
        float horizontalInput = Input.GetAxis("Horizontal");
        float verticalInput = Input.GetAxis("Vertical");
        moveInput = new Vector3(horizontalInput, 0f, verticalInput).normalized;


        //Constantly update the current target node that we want to move to.
        //Create Path
    }

    private IEnumerator startCounter()
    {
        while(true)
        {
            elapsedTime += 1f;
            generate = true;
            yield return new WaitForSeconds(1f);
        }
        
    }

    //Physics uses fixedUpdate for some reason. I DONT MAKE THE RULES!
    void FixedUpdate()
    { 


        forward = forwardRaycast();
        forwardLeft = forwardLeftRaycast();
        forwardRight = forwardRightRaycast();
        backward = backRaycast();

        //Runs the function every physics frame. We now have a total of 5 raycasts.
        groundMovement(forward, backward, forwardLeft, forwardRight);

        //UnityEngine.Debug.Log("Current Velocity: " + rb.velocity.magnitude);


        //We need to compare each raycast's distance, and choose the closest one to move away from for a specified time if we touch it or something.

    }

    private RaycastHit forwardRightRaycast()
    {
        Vector3 raycastOrigin = transform.position;

        //Have an initial raycast downwards (must be touching the ground.)
        Vector3 raycastDirection = (Vector3.forward + rb.velocity * 5).normalized;

        // Calculate the rotation quaternion for right direction (45 degrees)
        Quaternion rotationQuaternion = Quaternion.Euler(0, 30, 0);

        // Rotate the initial raycast direction to the left
        Vector3 rightRaycastDirection = rotationQuaternion * raycastDirection;


        //Did we hit anything?
        RaycastHit hit;
        LayerMask layerMask = ~LayerMask.GetMask("Myself");
        if (Physics.Raycast(raycastOrigin, rightRaycastDirection, out hit, raycastDistance, layerMask))
        {
            //Show the direction that the raycast is aiming, so we can check if the rotation ruins it.
            UnityEngine.Debug.DrawRay(raycastOrigin, rightRaycastDirection * raycastDistance, Color.blue);
            // Draw the contact point
            Debug.DrawRay(hit.point, hit.normal, Color.blue);

            // Return the raycast hit information.
            return hit;
        }
        else
        {
            // If no obstacle is hit, return a default RaycastHit.
            return new RaycastHit();
        }
    }
    private RaycastHit backRaycast()
    {
        Vector3 raycastOrigin = transform.position;

        //Have an initial raycast aim forwards
        Vector3 raycastDirection = (Vector3.forward - rb.velocity * 5).normalized;

        RaycastHit hit;
        LayerMask layerMask = ~LayerMask.GetMask("Myself");


        //If we detect anything, sent out a raycast signal.
        if (Physics.Raycast(raycastOrigin, raycastDirection, out hit, raycastDistance, layerMask))
        {
            //Show the direction that the raycast is aiming, so we can check if the rotation ruins it.
            UnityEngine.Debug.DrawRay(raycastOrigin, raycastDirection * raycastDistance, Color.red);

            // Return the raycast hit information.
            return hit;
        }
        else
        {
            // If no obstacle is hit, return a default RaycastHit.
            return new RaycastHit();
        }
    }


    //Generated a forward raycast with detection logic.
    private RaycastHit forwardRaycast()
    {
        Vector3 raycastOrigin = transform.position;

        //Have an initial raycast aim forwards
        Vector3 raycastDirection = (Vector3.forward + rb.velocity * 5).normalized;

        RaycastHit hit;
        LayerMask layerMask = ~LayerMask.GetMask("Myself");
        //If we detect anything, sent out a raycast signal.
        if (Physics.Raycast(raycastOrigin, raycastDirection, out hit, raycastDistance, layerMask))
        {
            //Show the direction that the raycast is aiming, so we can check if the rotation ruins it.
            UnityEngine.Debug.DrawRay(raycastOrigin, raycastDirection * raycastDistance, Color.red);

            // Return the raycast hit information.
            return hit;
        }
        else
        {
            // If no obstacle is hit, return a default RaycastHit.
            return new RaycastHit();
        }
    }

    private RaycastHit forwardLeftRaycast()
    {
        Vector3 raycastOrigin = transform.position;

        //Have an initial raycast downwards (must be touching the ground.)
        Vector3 raycastDirection = (Vector3.forward + rb.velocity * 5).normalized;

        // Calculate the rotation quaternion for left direction (45 degrees)
        Quaternion rotationQuaternion = Quaternion.Euler(0, -30, 0);

        // Rotate the initial raycast direction to the left
        Vector3 leftRaycastDirection = rotationQuaternion * raycastDirection;


        //Did we hit anything?
        RaycastHit hit;
        LayerMask layerMask = ~LayerMask.GetMask("Myself");
        if (Physics.Raycast(raycastOrigin, leftRaycastDirection, out hit, raycastDistance, layerMask))
        {
            //Show the direction that the raycast is aiming, so we can check if the rotation ruins it.
            UnityEngine.Debug.DrawRay(raycastOrigin, leftRaycastDirection * raycastDistance, Color.blue);

            // Return the raycast hit information.
            return hit;
        }
        else
        {
            // If no obstacle is hit, return a default RaycastHit.
            return new RaycastHit();
        }
    }

    //This function checks if the sphere is in contact with the ground. If it is, we can move. (Using debugging rn).
    private void groundMovement(RaycastHit forward, RaycastHit backward, RaycastHit forwardLeft, RaycastHit forwardRight)
    {
        Vector3 raycastOrigin = transform.position;

        //Have an initial raycast downwards (must be touching the ground.)
        Vector3 raycastDirection = (-Vector3.up);

        //Check the sphere is moving, and alter the raycast depending on its direction
        Vector3 velocityHint = (rb.velocity * 5).normalized;

        // Combine the raycast direction with the hint of the velocity vector
        raycastDirection = (raycastDirection + velocityHint).normalized;

        // Perform a downward raycast to detect the ground
        RaycastHit hit;

        //The raycast for the ground is enabled, and allows movement when in contact (Anti-hover).
        if (Physics.Raycast(raycastOrigin, raycastDirection, out hit, raycastGroundDistance))
        {
            autoMovement(hit, forward, backward, forwardLeft, forwardRight);

            // Show the direction that the raycast is aiming, so we can check if the rotation ruins it.
            UnityEngine.Debug.DrawRay(raycastOrigin, raycastDirection * raycastGroundDistance, Color.red);
        }
    }


    //Gets the closest point to the path the object is to.
    private Vector3 GetClosestPathPoint(List<Node> path)
    {
        //Initial infinity.
        float closestDistance = float.PositiveInfinity;

        //Assumes the end by default. Default value means we should already be at the finish line.
        Vector3 closestPoint = new Vector3(path[^1].nodePosition.X, path[^1].nodeHeight, path[^1].nodePosition.Y);

        // Iterate through each node in the path
        if(!finished)
        {
            if (visitedPaths[^1].nodePosition.X != 0 && visitedPaths[^1].nodePosition.Y != 0)
            {
                int visitedNodeIndex = path.IndexOf(visitedPaths[^1]);
                int endIndex = Mathf.Min(visitedNodeIndex + 10, path.Count);

                int count = endIndex - visitedNodeIndex;

                if (count < 0)
                {
                    count = visitedNodeIndex + 10;
                }

                CurrentPath = path.GetRange(visitedNodeIndex, count);
            }
            else
            {
                CurrentPath = path;
            }


            

            foreach (Node node in CurrentPath)
            {
                if (!visitedPaths.Contains(node))
                {
                    Vector3 nodePoint = new Vector3(node.nodePosition.X, node.nodeHeight, node.nodePosition.Y);

                    //Get the distances
                    float distance = Vector3.Distance(transform.position, nodePoint);

                    //Update the nodes
                    if (distance < closestDistance)
                    {
                        closestDistance = distance;
                        closestPoint = nodePoint;
                        currentTarget = node;
                    }
                }
            }




            //If we're near the current target node, add it to the list of visited nodes.
            if ((currentTarget.nodeHeight <= transform.position.y + 0.25f
                && (transform.position.x >= currentTarget.nodePosition.X - nodeAcceptanceRange && transform.position.x <= currentTarget.nodePosition.X + nodeAcceptanceRange) 
                && (transform.position.z >= currentTarget.nodePosition.Y - nodeAcceptanceRange && transform.position.z <= currentTarget.nodePosition.Y + nodeAcceptanceRange))
                || ((currentTarget.nodeHeight < 5.1f) && currentTarget.nodeHeight > transform.position.y + 0.25f) //If we are higher, allow a larger range of acceptance.
                && (transform.position.x >= currentTarget.nodePosition.X - nodeAcceptanceRange * 1.75f && transform.position.x <= currentTarget.nodePosition.X + nodeAcceptanceRange * 1.75f)
                && (transform.position.z >= currentTarget.nodePosition.Y - nodeAcceptanceRange * 1.75f && transform.position.z <= currentTarget.nodePosition.Y + nodeAcceptanceRange * 1.75f))
            {
                //If we are far enough into the line, remove 13 nodess previous so we don't accidentally go backwards, but can still get back on track if need be.
                int numberNodes = path.GetRange(0, path.IndexOf(currentTarget)).Count;
                if(numberNodes > 13)
                {
                    numberNodes = 13;
                }



                //Run a loop.
                foreach (Node nodes in path.GetRange(path.IndexOf(currentTarget) - numberNodes, numberNodes))
                    {
                        visitedPaths.Add(nodes);
                        visitedPaths.RemoveRange(0, 1);
                    }

                if(nodeAdditionDebug)
                {
                    Debug.Log("Added a node to the visited nodes path!");
                }


                //Add the current Node to the list of our visited Nodes.
                visitedPaths.Add(currentTarget);

                //This removes the previous node so we don't get stuck in a loop, but can go back if we need to due to knockback.
                visitedPaths.RemoveRange(0, 1);

                // If we're on the lat item of the path, stop moving.
                if (currentTarget == path[^1])
                {
                    finished = true;
                }
            }
        }



        // Return the closest point on the path
        return closestPoint;
    }


    //Moves based on the grounded raycast hit. Used for manual debugging of the movement.
    private void autoMovement(RaycastHit hit, RaycastHit forward, RaycastHit backward, RaycastHit forwardLeft, RaycastHit forwardRight)
    {
        // Use the surface normal as the up direction
        Vector3 upDirection = hit.normal;

        Vector3 inputDirection = Vector3.zero;


        //Follow along a path.
        switch (movementType)
        {
            case (moveType.automatic):
                if (path.Count > 1)
                {
                    Vector3 nodePoint = GetClosestPathPoint(path);
                    inputDirection = transform.position - nodePoint;
                }
                break;

            case (moveType.manual):
                // Use the player's input direction as the force direction
                inputDirection = -moveInput;
                break;
        }

        /* NEED TO INCORPORATE THE OBJECT AVOIDANCE PROPERLY! IT JUST DOESN'T DETECT RAYCASTS WELL!
         * */

        // Combine the avoidance direction with the input direction
        Vector3 combinedDirection = (inputDirection);

        // Calculate the force direction by combining the up direction and combined direction
        Vector3 forceDirection = Vector3.Cross(upDirection, Vector3.Cross(Vector3.up, combinedDirection)).normalized;

        // Scale the force based on the player's input
        Vector3 finalDirection = forceDirection.normalized * Mathf.Min(moveForce, 10f); // Ensure the force doesn't exceed the maximum force limit

        //Debug.Log("Final direction = " + finalDirection);
        // Move the sphere.
        rb.AddForce(finalDirection, ForceMode.Acceleration);



        //Object avoidance:
        // Apply obstacle avoidance force if obstacles are detected. WHY DID THIS TAKE SO LONG HEIAUBGIUYAGoigahi jn
        if (forward.collider != null || forwardLeft.collider != null || forwardRight.collider != null || backward.collider != null)
        {
            // Calculate the average normal of the hit points
            Vector3 averageNormal = (backward.normal + forward.normal + forwardLeft.normal + forwardRight.normal).normalized * Math.Min(moveForce, 10f);

            // Calculate the avoidance direction opposite to the average normal
            if (forward.collider != null && forward.collider.CompareTag("agent"))
            {
                rb.AddForce(forward.normal * avoidanceWeight * Mathf.Min(moveForce, 10f), ForceMode.Acceleration);
            }

            if (backward.collider != null && backward.collider.CompareTag("agent"))
            {
                rb.AddForce(backward.normal * avoidanceWeight * Mathf.Min(moveForce, 10f), ForceMode.Acceleration);
            }
                
            if (forwardLeft.collider != null && forwardLeft.collider.CompareTag("agent"))
            {
                rb.AddForce(forwardLeft.normal * avoidanceWeight * Mathf.Min(moveForce, 10f), ForceMode.Acceleration);
            }
                
            if (forwardRight.collider != null && forwardRight.collider.CompareTag("agent"))
            {
                rb.AddForce(forwardRight.normal * avoidanceWeight * Mathf.Min(moveForce, 10f), ForceMode.Acceleration);
            }


            //Basic object avoidance. Is half of agent avoidance weight.
            rb.AddForce(averageNormal * avoidanceWeight, ForceMode.Acceleration);
            //rb.AddForce(10 * Vector3.up, ForceMode.Acceleration);


            LayerMask layerMask = ~LayerMask.GetMask("Myself");

            // Get the layer index of "Myself"
            int myselfLayerIndex = LayerMask.NameToLayer("Myself");

            // Shift 1 to the left by the layer index to create a bitmask for the "Myself" layer
            int myselfLayerMask = 1 << myselfLayerIndex;

            // Invert the bitmask to exclude the "Myself" layer
            layerMask &= ~myselfLayerMask;



            //Do a little jump if we are about to hit an obstacle.
            if (forward.collider != null && forward.collider.CompareTag("obstacle") && (layerMask & (1 << forward.collider.gameObject.layer)) != 0 && generate == true)
            {
                rb.AddForce(10 * Vector3.up, ForceMode.Acceleration);
                generate = false;
                //We can instead RECALCULATE THE PATH TO GO AROUND IT!!!!
                //Check the closest node to this area point.
                //Check the shortest path between the closest node to that point before hand, and the node closest after the object.
                //Once the list is returned, remove the old object index from our node path, and add these new ones to the path.
                start = transform;
                instantiateVariables();
                generatePath();
            }

            //If we have an enemy agent appear behind us, RELEASE THE CORPSE!!!!
            if (backward.collider != null && backward.collider.CompareTag("agent") && elapsedTime > 5f)
            {
                jointToDeactivate.SetActive(false);
            }
        }






        if (debugVelocity)
        {
            Debug.Log("Current Velocity: " + rb.velocity); 
        }
        // Limit the velocity if it exceeds the maximum velocity
        if (rb.velocity.magnitude > maxLinearSpeed)
        {
            rb.velocity = rb.velocity.normalized * maxLinearSpeed;
        }
    }
}
