using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class StartSpawn : MonoBehaviour
{
    public GameObject[] agentsToSpawn;
    public int numberOfAgents = 10;
    public float spawnRadius = 5f;

    void Start()
    {
        SpawnAgents();
    }

    void SpawnAgents()
    {
        List<Vector3> spawnPoints = GenerateSpawnPoints();
        for (int i = 0; i < agentsToSpawn.Length; i++)
        {
            Instantiate(agentsToSpawn[i], spawnPoints[i], Quaternion.identity);
        }
    }

    List<Vector3> GenerateSpawnPoints()
    {
        List<Vector3> spawnPoints = new List<Vector3>();

        for (int i = 0; i < numberOfAgents; i++)
        {
            Vector3 randomPoint = GetRandomPointInBounds();
            
            //While we have intersecting agents, keep getting a random point in space.
            while (Intersects(randomPoint, spawnPoints))
            {
                randomPoint = GetRandomPointInBounds();
            }
            spawnPoints.Add(randomPoint);
        }

        //Retunr the list of spawn points.
        return spawnPoints;
    }

    Vector3 GetRandomPointInBounds()
    {
        //Helper function to get a random point.
        float randomX = Random.Range(transform.position.x - spawnRadius, transform.position.x + spawnRadius);
        float randomZ = Random.Range(transform.position.z - spawnRadius, transform.position.z + spawnRadius);
        return new Vector3(randomX, transform.position.y + Random.Range(0, 2f), randomZ);
    }

    bool Intersects(Vector3 point, List<Vector3> existingSpawnPoints)
    {
        //Check if there is an intersection, and if there is, return true;
        foreach (Vector3 existingPoint in existingSpawnPoints)
        {
            //If our spawn point is too close to an existing spawn point.
            if (Vector3.Distance(point, existingPoint) < 1.5f)
            {
                return true;
            }
        }
        return false;
    }
}
