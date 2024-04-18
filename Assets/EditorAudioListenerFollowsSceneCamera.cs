using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace twe36
{
    public class EditorAudioListenerFollowsSceneCamera : MonoBehaviour
    {
        private void Start()
        {
            if (Application.isEditor)
            {
                foreach (AudioListener a in GameObject.FindObjectsOfType<AudioListener>())
                    a.enabled = false;

                this.gameObject.AddComponent<AudioListener>();

                StartCoroutine(FollowCamera());
            }
        }


        IEnumerator FollowCamera()
        {
            while (true)
            {
                if (Camera.current)
                {
                    this.transform.position = Camera.current.transform.position;
                }
                yield return null;
            }
        }
    }

}
