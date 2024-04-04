using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Specialized;
using System.Runtime.InteropServices.WindowsRuntime;
using System.Threading;
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

    //Introduce a variable to handle the ratios of the avoidance.
    [Range(0.0f, 1f)]
    public float avoidanceWeight = 0.5f;



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


    public enum moveType
    {
        automatic,
        manual
    };

    //Path variables for the algorithm.
    private List<Node> path = new List<Node>(); // Paht for the object to follow.
    private TerrainGraph graph; // A graph to store information.
    public Transform start; // The start transform node
    public Transform end; // The finish line.
    public moveType movementType;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        graph = new TerrainGraph();

    }


    void generatePath()
    {
        // Start node position
        int startX = (int)start.position.x;
        int startZ = (int)start.position.z;

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
        // Gather player input
        float horizontalInput = Input.GetAxis("Horizontal");
        float verticalInput = Input.GetAxis("Vertical");
        moveInput = new Vector3(horizontalInput, 0f, verticalInput).normalized;


        //Constantly update the current target node that we want to move to.
        //Create Path
        generatePath();

        debugPath();
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
        // Calculate the raycast origin
        Vector3 raycastOrigin = transform.position;

        //Have an initial raycast downwards (must be touching the ground.)
        Vector3 raycastDirection = (Vector3.forward + rb.velocity * 5).normalized;

        // Calculate the rotation quaternion for right direction (45 degrees)
        Quaternion rotationQuaternion = Quaternion.Euler(0, 35, 0);

        // Rotate the initial raycast direction to the left
        Vector3 rightRaycastDirection = rotationQuaternion * raycastDirection;


        //Did we hit anything?
        RaycastHit hit;

        if (Physics.Raycast(raycastOrigin, rightRaycastDirection, out hit, raycastDistance))
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
        // Calculate the raycast origin
        Vector3 raycastOrigin = transform.position;

        //Have an initial raycast aim forwards
        Vector3 raycastDirection = (Vector3.forward - rb.velocity * 5).normalized;

        RaycastHit hit;

        //If we detect anything, sent out a raycast signal.
        if (Physics.Raycast(raycastOrigin, raycastDirection, out hit, raycastDistance))
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
        // Calculate the raycast origin
        Vector3 raycastOrigin = transform.position;

        //Have an initial raycast aim forwards
        Vector3 raycastDirection = (Vector3.forward + rb.velocity * 5).normalized;

        RaycastHit hit;

        //If we detect anything, sent out a raycast signal.
        if (Physics.Raycast(raycastOrigin, raycastDirection, out hit, raycastDistance))
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
        // Calculate the raycast origin
        Vector3 raycastOrigin = transform.position;

        //Have an initial raycast downwards (must be touching the ground.)
        Vector3 raycastDirection = (Vector3.forward + rb.velocity * 5).normalized;

        // Calculate the rotation quaternion for left direction (45 degrees)
        Quaternion rotationQuaternion = Quaternion.Euler(0, -35, 0);

        // Rotate the initial raycast direction to the left
        Vector3 leftRaycastDirection = rotationQuaternion * raycastDirection;


        //Did we hit anything?
        RaycastHit hit;
        
        if (Physics.Raycast(raycastOrigin, leftRaycastDirection, out hit, raycastDistance))
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

    //Return a float distance from a particular raycast.
    private float collisionDistance(RaycastHit hit)
    {
        if (hit.collider != null)
        {
            return hit.distance;
        }
        else
        {
            return float.PositiveInfinity;
        }
    }


    //This function checks if the sphere is in contact with the ground. If it is, we can move. (Using debugging rn).
    private void groundMovement(RaycastHit forward, RaycastHit backward, RaycastHit forwardLeft, RaycastHit forwardRight)
    {
        // Calculate the raycast origin
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
        Vector3 closestPoint = Vector3.zero;

        // Iterate through each node in the path
        foreach (Node node in path)
        {
            Vector3 nodePoint = new Vector3(node.nodePosition.X, node.nodeHeight, node.nodePosition.Y);

            //Get the distances
            float distance = Vector3.Distance(transform.position, nodePoint);

            //Update the nodes
            if (distance < closestDistance)
            {
                closestDistance = distance;
                closestPoint = nodePoint;
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
        Vector3 combinedDirection = (inputDirection).normalized;

        // Calculate the force direction by combining the up direction and combined direction
        Vector3 forceDirection = Vector3.Cross(upDirection, Vector3.Cross(Vector3.up, combinedDirection)).normalized;

        // Scale the force based on the player's input
        Vector3 finalDirection = forceDirection.normalized * Mathf.Min(moveForce, 10f); // Ensure the force doesn't exceed the maximum force limit

        Debug.Log("Final direction = " + finalDirection);
        // Move the sphere.
        rb.AddForce(finalDirection, ForceMode.Acceleration);



        //Object avoidance:
        // Apply obstacle avoidance force if obstacles are detected. WHY DID THIS TAKE SO LONG HEIAUBGIUYAGoigahi jn
        if (forward.collider != null || forwardLeft.collider != null || forwardRight.collider != null || backward.collider != null)
        {
            // Calculate the average normal of the hit points
            Vector3 averageNormal = (backward.normal + forward.normal + forwardLeft.normal + forwardRight.normal).normalized * Math.Min(moveForce, 10f);

            // Calculate the avoidance direction opposite to the average normal
            rb.AddForce(averageNormal * avoidanceWeight, ForceMode.Acceleration);
        }





        // Limit the velocity if it exceeds the maximum velocity
        if (rb.velocity.magnitude > maxLinearSpeed)
        {
            rb.velocity = rb.velocity.normalized * maxLinearSpeed;
        }
    }
}
