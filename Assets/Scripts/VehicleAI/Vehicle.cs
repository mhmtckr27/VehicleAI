using UnityEngine;
using UnityEngine.Serialization;

namespace VehicleAI
{
    public class Vehicle : MovingEntity
    {
        [SerializeField] private bool isEvader;
        [SerializeField] private MovingEntity other;
        [SerializeField] public int index;
        
        public override EntityType EntityType => EntityType.Vehicle;

        private SteeringBehaviours _steeringBehaviours;

        private void Awake()
        {
            index = transform.GetSiblingIndex();
            _steeringBehaviours = new SteeringBehaviours(this, isEvader, other, transform.GetSiblingIndex());
        }

        protected void Update()
        {
            var steeringForce = _steeringBehaviours.Calculate();
            var acceleration = steeringForce / Mass;
            Velocity = Vector2.ClampMagnitude(Velocity + acceleration * Time.deltaTime, MaxSpeed);
            Position += Velocity * Time.deltaTime;

            if (Velocity.sqrMagnitude > 0.00000001f)
            {
                Heading = Velocity.normalized;
                Side = Vector2.Perpendicular(Heading);
            }
        }

        protected override void OnDrawGizmos()
        {
            base.OnDrawGizmos();
            
            _steeringBehaviours?.DrawGizmos();
        }
    }
}