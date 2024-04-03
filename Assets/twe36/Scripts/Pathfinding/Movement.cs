using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Specialized;
using System.Diagnostics;
using System.Runtime.InteropServices.WindowsRuntime;
using UnityEngine;

public class Movement : MonoBehaviour
{
    //Movement variables.
    [Range(0, 10f)]
    public float moveForce = 1f; // Acceleration force to move in.
    private float maxLinearSpeed = 10f;
    private Rigidbody rb;
    private Vector3 moveInput;



    //Raycast variables.
    [Range(0, 5f)]
    public float raycastGroundDistance = 1f; // Distance of the raycast to detect ground.

    [Range(0, 5f)]
    public float raycastDistance = 3f;

    // Distance in front of the character to cast the avoidance raycasts. Keep small so it doesn't worry about ramps or somethign.
    public float avoidanceDistance = 0.5f; 

    //Stop worrying abvout ramps so adjust angle.
    [Range(0, 180)]
    public float avoidanceAngle = 45f; // Angle between each avoidance raycast.




    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    void Update()
    {
        // Gather player input
        float horizontalInput = Input.GetAxis("Horizontal");
        float verticalInput = Input.GetAxis("Vertical");
        moveInput = new Vector3(horizontalInput, 0f, verticalInput).normalized;
    }

    //Physics uses fixedUpdate for some reason. I DONT MAKE THE RULES!
    void FixedUpdate()
    { 
        //Get the forward directions of the ball, relative to the world, plus rotation.
        RaycastHit forwardLeft;
        RaycastHit forwardRight;
        RaycastHit backward;
        RaycastHit forward;

        //Runs the function every physics frame. We now have a total of 5 raycasts.
        groundMovement();
        
        forward = forwardRaycast();
        forwardLeft = forwardLeftRaycast();
        forwardRight = forwardRightRaycast();
        backward = backRaycast();
        UnityEngine.Debug.Log("Current Velocity: " + rb.velocity.magnitude);


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
    private void groundMovement()
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
            playerInputMovement(hit);

            // Show the direction that the raycast is aiming, so we can check if the rotation ruins it.
            UnityEngine.Debug.DrawRay(raycastOrigin, raycastDirection * raycastGroundDistance, Color.red);
        }
    }

    //Moves based on the grounded raycast hit. Used for manual debugging of the movement.
    private void playerInputMovement(RaycastHit hit)
    {
        // Use the surface normal as the up direction
        Vector3 upDirection = hit.normal;

        // Use the player's input direction as the force direction
        Vector3 inputDirection = -moveInput;

        // Calculate the force direction by combining the up direction and input direction
        Vector3 forceDirection = Vector3.Cross(upDirection, Vector3.Cross(Vector3.up, inputDirection)).normalized;

        // Scale the force based on the player's input
        Vector3 finalForce = forceDirection * Mathf.Min(moveForce, 10f); // Ensure the force doesn't exceed the maximum force limit

        // Move the sphere.
        rb.AddForce(finalForce, ForceMode.Acceleration);

        // Limit the velocity if it exceeds the maximum velocity
        if (rb.velocity.magnitude > maxLinearSpeed)
        {
            rb.velocity = rb.velocity.normalized * maxLinearSpeed;
        }
    }
}
