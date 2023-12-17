using UnityEngine;

namespace VehicleAI
{
    public class SteeringBehaviours
    {
        private MovingEntity _owner;
        private const float PanicDistanceSq = 100f * 100f;
        
        public SteeringBehaviours(MovingEntity owner)
        {
            _owner = owner;
        }
        
        public Vector2 Calculate()
        {
            return Vector2.down;
        }

        public Vector2 Seek(Vector2 targetPosition)
        {
            var desiredVelocity = (targetPosition - _owner.Position).normalized * _owner.MaxSpeed;
            return desiredVelocity - _owner.Velocity;
        }

        public Vector2 Flee(Vector2 targetPosition)
        {
            if(Vector2.SqrMagnitude(_owner.Position - targetPosition) > PanicDistanceSq)
                return Vector2.zero;
            
            var desiredVelocity = (_owner.Position - targetPosition).normalized * _owner.MaxSpeed;
            return desiredVelocity - _owner.Velocity;
        }

        public Vector2 Arrive(Vector2 targetPosition, Deceleration deceleration)
        {
            var toTarget = targetPosition - _owner.Position;
            var distance = toTarget.magnitude;
            
            if(distance <= 0)
                return Vector2.zero;

            const float decelerationTweaker = 0.3f;
            var speed = distance / ((int)deceleration * decelerationTweaker);

            speed = Mathf.Min(speed, _owner.MaxSpeed);

            var desiredVelocity = toTarget * speed / distance;
            return (desiredVelocity - _owner.Velocity);
        }

        public Vector2 Pursuit(MovingEntity evader)
        {
            var toEvader = evader.Position - _owner.Position;
            var relativeHeading = Vector2.Dot(_owner.Heading, evader.Heading);

            if (Vector2.Dot(toEvader, _owner.Heading) > 0 && relativeHeading < -0.95f)
            {
                return Seek(evader.Position);
            }

            var lookAheadTime = toEvader.magnitude / (_owner.MaxSpeed + evader.Speed);
            lookAheadTime += TurnAroundTime(evader.Position);
            return Seek(evader.Position + evader.Velocity * lookAheadTime);
        }

        public Vector2 Evade(MovingEntity pursuer)
        {
            var toPursuer = pursuer.Position - _owner.Position;

            var lookAheadTime = toPursuer.magnitude / (_owner.MaxSpeed + pursuer.Speed);
            return Flee(pursuer.Position + pursuer.Velocity * lookAheadTime);
        }

        private float TurnAroundTime(Vector2 targetPosition)
        {
            var toTarget = (targetPosition - _owner.Position.normalized);
            var dot = Vector2.Dot(_owner.Heading, toTarget);
            
            //change this value to get the desired behaviour. The higher the max turn rate of the vehicle,
            //the higher this value should be. If the vehicle is heading in the opposite direction to its target position
            //then a value of 0.5 means that this function will return a time of 1 sec for the vehicle to turn around.
            var coefficient = 0.5f;
            
            //the dot product gives a value of 1 if the target is directly ahead and -1 if it is directly behind. 
            //Subtracting 1 and multiplying by the negative of the coefficient gives a positive value proportional
            //to the rotational displacement of the vehicle and target.
            return (dot - 1f) * -coefficient;
        }
    }
}