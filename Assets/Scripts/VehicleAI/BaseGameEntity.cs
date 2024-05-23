using System;
using Extensions;
using UnityEngine;

namespace VehicleAI
{
    public abstract class BaseGameEntity : MonoBehaviour
    {
        [field: SerializeField] public float BoundingRadius { get; private set; }
        [field: SerializeField] public GameWorld GameWorld { get; private set; }
        
        public int ID => GetInstanceID();

        public Vector2 Position
        {
            get => transform.position.ToVec2();
            set => transform.position = value.ToVec3(GameWorld.WorldYPos);
        }

        public abstract EntityType EntityType { get; }
        public bool IsTagged { get; private set; }
        //TODO: scale

#if UNITY_EDITOR
        private void OnDrawGizmos()
        {
            Gizmos.color = Color.black;
            Gizmos.DrawWireSphere(transform.position, BoundingRadius);
        }
#endif
        public void Tag()
        {
            IsTagged = true;
        }

        public void Untag()
        {
            IsTagged = false;
        }
    }

    public enum EntityType
    {
        None = -1,
        Vehicle = 100,
        Zombie = 200,
        Player = 300,
    }
}