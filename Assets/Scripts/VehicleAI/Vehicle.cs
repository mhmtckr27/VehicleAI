using UnityEngine;
using UnityEngine.Serialization;
using Utils;

namespace VehicleAI
{
    public class Vehicle : MovingEntity
    {
        [SerializeField] private bool isEvader;
        [SerializeField] private MovingEntity other;
        [SerializeField] public int index;
        
        public override EntityType EntityType => EntityType.Vehicle;

        private SteeringBehaviours _steeringBehaviours;
        private bool _isSmoothingOn = true;
        
        public Vector2 SmoothedHeading { get; private set; }
        private Smoother _smoother;

        private void Awake()
        {
            index = transform.GetSiblingIndex();
            _steeringBehaviours = new SteeringBehaviours(this, isEvader, other, transform.GetSiblingIndex());
            _smoother = new Smoother(10, Vector2.zero);
        }

        protected void Update()
        {
            var steeringForce = _steeringBehaviours.Calculate();
            var acceleration = steeringForce / Mass;
            Velocity = Vector2.ClampMagnitude(Velocity + acceleration * Time.deltaTime, MaxSpeed);
            Position += Velocity * Time.deltaTime;

            if (Velocity.sqrMagnitude > Mathf.Epsilon)
            {
                if (_isSmoothingOn)
                {
                    SmoothedHeading = _smoother.Update(Velocity.normalized);
                    Heading = SmoothedHeading;
                }
                else
                {
                    Heading = Velocity.normalized;
                }

                Side = Vector2.Perpendicular(Heading);
            }
            
            if (_isSmoothingOn)
            {
                SmoothedHeading = _smoother.Update(Heading);
                Heading = SmoothedHeading;
            }
        }

        protected override void OnDrawGizmos()
        {
            base.OnDrawGizmos();
            
            _steeringBehaviours?.DrawGizmos();
        }
    }
}