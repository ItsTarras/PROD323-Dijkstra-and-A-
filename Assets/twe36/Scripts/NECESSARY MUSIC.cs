using System.Collections;
using System.Collections.Generic;
using UnityEngine;
namespace twe36
{
    public class NECESSARYMUSIC : MonoBehaviour
    {
        public AudioClip[] songs;

        private AudioSource audioSource;

        void Start()
        {
            audioSource = GetComponent<AudioSource>();
            PlayRandomSong();
            StartCoroutine(WaitForSongEnd());
        }

        void PlayRandomSong()
        {
            if (songs.Length > 0)
            {
                int randomIndex = Random.Range(0, songs.Length);
                audioSource.clip = songs[randomIndex];
                audioSource.Play();
            }
        }


        IEnumerator WaitForSongEnd()
        {
            // Wait until the current song finishes playing
            while (audioSource.isPlaying)
            {
                yield return null;
            }

            // Play another random song once the current one is done
            PlayRandomSong();
        }
    }
}
