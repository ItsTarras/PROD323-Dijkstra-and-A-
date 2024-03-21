using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class ArenaManager : MonoBehaviour
{
    // This will be populated during the tournament
    [SerializeField]
    private GameObject[] prefabsOfAgents;

    [SerializeField]
    private GameObject[] spawnedAgents;

    // Add all the spawnpoints in your arena in the inspector
    [SerializeField]
    private Transform[] spawnPoints;

    private void Awake()
    {
        spawnedAgents = new GameObject[prefabsOfAgents.Length];
        SpawnAgents();
    }

    void Start()
    {
        InvokeRepeating("RespawnStuckAgent", 20f, 5f);
    }

    private void SpawnAgents()
    {
        prefabsOfAgents = ShuffleAgents(prefabsOfAgents);

        if(prefabsOfAgents.Length == spawnPoints.Length)
        {
            for (int index = 0; index < spawnPoints.Length; ++index)
            {
                spawnedAgents[index] = Instantiate(prefabsOfAgents[index], spawnPoints[index].transform.position, Quaternion.identity);
            }
        } 
        else
        {
            Debug.LogError("Mismatched number of agents and spawnpoints!");
        }
    }
    
    private void RespawnStuckAgent()
    {
        foreach(GameObject agent in spawnedAgents)
        {
            if(agent.GetComponent<Rigidbody>().velocity.magnitude < 1f)
            {
                int ranIndex = Random.Range(0, spawnPoints.Length);
                
                agent.transform.position = spawnPoints[ranIndex].transform.position;
            }
        }
    }

    private GameObject[] ShuffleAgents(GameObject[] agents)
    {
        System.Random rand = new System.Random();
        int length = agents.Length;

        // Shuffle: Fisher-Yates
        while(length > 1)
        {
            length--;
            int index = rand.Next(length + 1);
            GameObject temp = agents[index];
            agents[index] = agents[length];
            agents[length] = temp;
        }

        return agents;
    }

}
