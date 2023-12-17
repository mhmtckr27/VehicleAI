using System;
using UnityEngine;

namespace VehicleAI
{
    public class Vehicle : MovingEntity
    {
        public override EntityType EntityType => EntityType.Vehicle;
        public override bool Tag => false;

        [SerializeField] private GameWorld gameWorld;

        private SteeringBehaviours _steeringBehaviours;
        
        private void Update()
        {
            var steeringForce = _steeringBehaviours.Calculate();
            var acceleration = steeringForce / Mass;
            Velocity = Vector2.ClampMagnitude(Velocity + acceleration * Time.deltaTime, MaxSpeed);
            Pos += Velocity * Time.deltaTime;

            if (Velocity.sqrMagnitude > 0.00000001f)
            {
                Heading = Velocity.normalized;
                Side = Vector2.Perpendicular(Heading);
            }
        }
    }
}