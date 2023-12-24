using Extensions;
using UnityEngine;

namespace VehicleAI
{
    public class Vehicle : MovingEntity
    {
        [SerializeField] private bool isEvader;
        [SerializeField] private MovingEntity other;
        
        public override EntityType EntityType => EntityType.Vehicle;
        public override bool Tag => false;

        [SerializeField] private GameWorld gameWorld;

        private SteeringBehaviours _steeringBehaviours;

        private void Awake()
        {
            _steeringBehaviours = new SteeringBehaviours(this, isEvader, other);
        }

        protected void Update()
        {
            var steeringForce = _steeringBehaviours.Calculate();
            var acceleration = steeringForce / Mass;
            Velocity = Vector2.ClampMagnitude(Velocity + acceleration * Time.deltaTime, MaxSpeed);
            Position += Velocity * Time.deltaTime;
            
            Debug.LogError(Velocity);

            if (Velocity.sqrMagnitude > 0.00000001f)
            {
                Heading = Velocity.normalized;
                Side = Vector2.Perpendicular(Heading);
            }
        }
    }
}