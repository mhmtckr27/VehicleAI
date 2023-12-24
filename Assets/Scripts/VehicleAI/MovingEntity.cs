using Extensions;
using UnityEngine;

namespace VehicleAI
{
    public abstract class MovingEntity : BaseGameEntity
    {
        public Vector2 Velocity { get; protected set; }
        public float Speed => Velocity.magnitude;

        public Vector2 Heading
        {
            get => transform.forward.ToVec2(); 
            protected set => transform.forward = value.ToVec3();
        }
        public Vector2 Side { get; protected set; }
        [field: SerializeField] public float Mass { get; protected set; }
        [field: SerializeField] public float MaxSpeed { get; protected set; }
        [field: SerializeField] public float MaxForce { get; protected set; }
        [field: SerializeField] public float MaxTurnRate { get; protected set; }
    }

    public enum Deceleration
    {
        Fast = 1,
        Normal = 2,
        Slow = 3,
    }
}