using UnityEngine;
using Cinemachine;
using Unity.IO.LowLevel.Unsafe;

public class CameraManager : MonoBehaviour
{
    // Place the virtual cameras in this order in the vCam array/inspector:
    // 0 - ArenaAerialCamera
    // 1 - ArenaCamera1
    // 2 - ArenaCamera2
    // 3 - ArenaCamera3
    // 4 - ArenaFinishZoneCamera

    [SerializeField]
    private CinemachineVirtualCamera[] vCams;

    [SerializeField]
    private CinemachineFreeLook flCam;

    // The agents array will be populated during the tournament
    [SerializeField]
    private GameObject[] agents;

    private bool inFreeLook = false;

    private int index;

    private void Start()
    {
        // Get list of all spawned agents
        agents = GameObject.FindGameObjectsWithTag("Agent");

        // Choose a random agent for free look camera
        index = Random.Range(0, agents.Length-1);
        flCam.Follow = agents[index].transform;
        flCam.LookAt = agents[index].transform;
    }

    void Update()
    {
        if (Input.GetKey(KeyCode.Keypad0)) // ArenaAerialCamera
        {
            foreach (CinemachineVirtualCamera cm in vCams)
            {
                cm.Priority = 10;
            }

            flCam.Priority = 10;
            vCams[0].Priority = 20;
            inFreeLook = false;
        }

        if (Input.GetKey(KeyCode.Keypad1)) // ArenaCamera1
        {
            foreach (CinemachineVirtualCamera cm in vCams)
            {
                cm.Priority = 10;
            }

            flCam.Priority = 10;
            vCams[1].Priority = 20;
            inFreeLook = false;
        }

        if (Input.GetKey(KeyCode.Keypad2)) // ArenaCamera2
        {
            foreach (CinemachineVirtualCamera cm in vCams)
            {
                cm.Priority = 10;
            }

            flCam.Priority = 10;
            vCams[2].Priority = 20;
            inFreeLook = false;
        }

        if (Input.GetKey(KeyCode.Keypad3)) // ArenaCamera3
        {
            foreach (CinemachineVirtualCamera cm in vCams)
            {
                cm.Priority = 10;
            }

            flCam.Priority = 10;
            vCams[3].Priority = 20;
            inFreeLook = false;
        }


        if (Input.GetKey(KeyCode.Keypad4)) // ArenaFinishZoneCamera
        {
            foreach (CinemachineVirtualCamera cm in vCams)
            {
                cm.Priority = 10;
            }

            flCam.Priority = 10;
            vCams[4].Priority = 20;
            inFreeLook = false;
        }

        if (Input.GetKey(KeyCode.Keypad5)) // FreeLookCamera
        {
            foreach (CinemachineVirtualCamera cm in vCams)
            {
                cm.Priority = 10;
            }

            flCam.Priority = 20;
            inFreeLook = true;
        }

        if (Input.GetKey(KeyCode.Space) && inFreeLook) // Look at a next agent
        {
            index++;
            if(index >= agents.Length)
            {
                index = 0;
            }
            
            flCam.Follow = agents[index].transform;
            flCam.LookAt = agents[index].transform;  
        }
    }
}
