using System;
using System.Collections.Generic;
using UnityEngine;
using Utils;
using Random = UnityEngine.Random;

namespace VehicleAI
{
    public class SteeringBehaviours
    {
        private MovingEntity _owner;
        private const float PanicDistanceSq = 100f * 100f;
        private const float WanderRadius = 1.2f;
        private const float WanderDistance = 2f;
        private const float WanderJitterPerSecond = 80f;
        private Vector2 _wanderTarget;
        private readonly bool _isEvader;
        private readonly MovingEntity _other;

        private const float MinDetectionBoxLength = 40f;
        private float _detectionBoxLength;
        private float _wallDetectionFeelerLength;
        private List<Vector2> _feelers = new();

        public SteeringBehaviours(MovingEntity owner, bool isEvader, MovingEntity other)
        {
            _owner = owner;
            _isEvader = isEvader;
            _other = other;
        }

        public virtual Vector2 Calculate()
        {
            if (_isEvader)
                return Evade(_other) + Wander();
            else
                return Pursuit(_other);
        }

        public Vector2 Seek(Vector2 targetPosition)
        {
            var desiredVelocity = (targetPosition - _owner.Position).normalized * _owner.MaxSpeed;
            return desiredVelocity - _owner.Velocity;
        }

        public Vector2 Flee(Vector2 targetPosition)
        {
            if (Vector2.SqrMagnitude(_owner.Position - targetPosition) > PanicDistanceSq)
                return Vector2.zero;

            var desiredVelocity = (_owner.Position - targetPosition).normalized * _owner.MaxSpeed;
            return desiredVelocity - _owner.Velocity;
        }

        public Vector2 Arrive(Vector2 targetPosition, Deceleration deceleration)
        {
            var toTarget = targetPosition - _owner.Position;
            var distance = toTarget.magnitude;

            if (distance <= 0)
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

        public Vector2 Wander()
        {
            var wanderJitter = WanderJitterPerSecond * Time.deltaTime;
            _wanderTarget += new Vector2(Random.Range(-1f, 1f) * wanderJitter, Random.Range(-1f, 1f) * wanderJitter);
            _wanderTarget.Normalize();
            _wanderTarget *= WanderRadius;
            var targetLocal = _wanderTarget + new Vector2(WanderDistance, 0);
            var targetWorld = _owner.Position + targetLocal;

            return targetWorld - _owner.Position;
        }

        private float TurnAroundTime(Vector2 targetPosition)
        {
            var toTarget = (targetPosition - _owner.Position).normalized;
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

        #region Avoidance

        private Vector2 ObstacleAvoidance(List<BaseGameEntity> obstacles)
        {
            _detectionBoxLength = MinDetectionBoxLength + (_owner.Speed / _owner.MaxSpeed) * MinDetectionBoxLength;
            _owner.GameWorld.TagObstaclesWithinViewRange(_owner, _detectionBoxLength);

            BaseGameEntity closestIntersectingObstacle = null;
            double distanceToClosestIntersectingPoint = Double.MaxValue;
            Vector2 localPositionOfClosestObstacle = default;

            foreach (var obstacle in obstacles)
            {
                if (!obstacle.IsTagged)
                    continue;

                //may need to use other inverse transform sh!ts.
                var localPos = _owner.transform.InverseTransformPoint(obstacle.Position);

                if (localPos.x < 0)
                    continue;

                var expandedRadius = obstacle.BoundingRadius + _owner.BoundingRadius;

                if (Mathf.Abs(localPos.y) >= expandedRadius)
                    continue;

                var centerX = localPos.x;
                var centerY = localPos.y;
                var sqrtPart = Mathf.Sqrt(expandedRadius * expandedRadius - centerY * centerY);

                var ip = centerX - sqrtPart;

                if (ip <= 0)
                    ip = centerX + sqrtPart;

                if (!(ip < distanceToClosestIntersectingPoint))
                    continue;

                distanceToClosestIntersectingPoint = ip;
                closestIntersectingObstacle = obstacle;
                localPositionOfClosestObstacle = localPos;
            }

            Vector2 steeringForce;

            if (closestIntersectingObstacle == null)
                return default;

            var multiplier = 1.0 + (_detectionBoxLength - localPositionOfClosestObstacle.x) / _detectionBoxLength;
            steeringForce.y = (float)((closestIntersectingObstacle.BoundingRadius - localPositionOfClosestObstacle.y) *
                                      multiplier);

            const double brakingWeight = 0.2;

            steeringForce.x = (float)((closestIntersectingObstacle.BoundingRadius - localPositionOfClosestObstacle.x) *
                                      brakingWeight);

            return _owner.transform.TransformDirection(steeringForce);
        }

        private Vector2 WallAvoidance(List<Wall> walls)
        {
            CreateFeelers();

            var distanceToThisIP = 0.0;
            var distanceToClosestIP = double.MaxValue;

            Wall closestWall = null;

            Vector2 steeringForce = default;
            Vector2 point = default;
            Vector2 closestPoint = default;

            foreach (var feeler in _feelers)
            {
                foreach (var wall in walls)
                {
                    if (!GeometryHelper.LineIntersection2D(_owner.Position, feeler, wall.From, wall.To, ref distanceToThisIP, point))
                        continue;

                    if (!(distanceToThisIP < distanceToClosestIP))
                        continue;
                    
                    distanceToClosestIP = distanceToThisIP;
                    closestWall = wall;
                    closestPoint = point;
                }

                if (closestWall == null)
                    continue;
                
                var overshoot = feeler - closestPoint;
                steeringForce = closestWall.Normal * overshoot.magnitude;
            }

            return steeringForce;
        }

        private void CreateFeelers()
        {
            //feeler pointing straight in front
            _feelers[0] = _owner.Position + _wallDetectionFeelerLength * _owner.Heading;

            //feeler to left
            var temp = _owner.Heading;
            temp = -Vector2.Perpendicular(temp);
            _feelers[1] = _owner.Position + _wallDetectionFeelerLength / 2.0f * temp;

            //feeler to right
            temp = _owner.Heading;
            temp = Vector2.Perpendicular(temp);
            _feelers[2] = _owner.Position + _wallDetectionFeelerLength / 2.0f * temp;
        }

        #endregion
    }
}