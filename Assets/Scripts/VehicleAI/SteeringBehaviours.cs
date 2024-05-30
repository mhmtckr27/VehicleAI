using System;
using System.Collections.Generic;
using Extensions;
using JetBrains.Annotations;
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
        private Vector2 _wanderTargetVector;
        private readonly bool _isEvader;
        private readonly MovingEntity _other;

        private const float MinDetectionBoxLength = 40f;
        private float _detectionBoxLength;
        private float _wallDetectionFeelerLength = 2;
        private List<Vector2> _feelers = new()
        {
            default,
            default,
            default
        };
        private Path _path;
        private float _waypointSeekDistance = 10;
        private readonly int _index;

        private Vector2 _steeringForce;
        
        private float _wanderAmount;
        private float _obstacleAvoidanceAmount;
        private float _separationAmount;

        private const float _wallAvoidancePriority = 100;
        private const float _obstacleAvoidancePriority = 1;
        private const float _separationPriority = 1;
        private const float _wanderPriority = 0.25f;

        private float _wallAvoidanceWeight;
        private float _obstacleAvoidanceWeight;
        private float _separationWeight;

        private List<BehaviourType> onBehaviours = new List<BehaviourType>()
        {
            BehaviourType.WallAvoidance,
            BehaviourType.Wander
        };

        private Vector2 _wanderToPosition;

        public SteeringBehaviours(MovingEntity owner, bool isEvader, MovingEntity other, int index)
        {
            _owner = owner;
            _isEvader = isEvader;
            _other = other;
            //todo : TeMP, remove
            _index = index;
        }

        public virtual Vector2 Calculate()
        {
            // if (_isEvader)
            //     return Evade(_other) + Wander();
            // else
            //     return Pursuit(_other);

            _owner.GameWorld.TagAgentsWithinRange(_owner, 10);
            
            return Calculate_WTRSP();
            
            return Cohesion(_owner.GameWorld.Agents);
            return Alignment(_owner.GameWorld.Agents);
            return Separation(_owner.GameWorld.Agents);
            
            if (_isEvader)
                return Wander();
            else
            {
                if(_index == 1)
                    return OffsetPursuit(_other, new Vector2(-10, -10));
                else if(_index == 2)
                    return OffsetPursuit(_other, new Vector2(10, -10));
                else if(_index == 3)
                    return OffsetPursuit(_other, new Vector2(-20, -20));
                else if(_index == 4)
                    return OffsetPursuit(_other, new Vector2(0, -20));
                else
                    return OffsetPursuit(_other, new Vector2(20, -20));
            }
        }

        /// <summary>
        /// Calculates the steering force using the method 'Weighted Truncated Sum'
        /// ps. It is a bad method for several problems mentioned at the Page 120
        /// </summary>
        /// <returns></returns>
        private Vector2 Calculate_WTS()
        {
            Vector2 steeringForce = default;

            steeringForce += Wander() * _wanderAmount;
            steeringForce += ObstacleAvoidance(_owner.GameWorld.Obstacles) * _obstacleAvoidanceAmount;
            steeringForce += Separation(_owner.GameWorld.Agents) * _separationAmount;

            return steeringForce;
        }
        
        /// <summary>
        /// Calculates the steering force using the method 'Weighted Truncated Running Sum with Prioritization'
        /// ps. It is the method that is used for the examples in the book
        /// </summary>
        /// <returns></returns>
        private Vector2 Calculate_WTRSP()
        {
            _steeringForce = default;
            
            Vector2 force = default;

            if (IsOn(BehaviourType.WallAvoidance))
            {
                force = WallAvoidance(_owner.GameWorld.Walls) * _wallAvoidancePriority;

                if (!AccumulateForce(ref _steeringForce, force)) 
                    return _steeringForce;
            }

            if (IsOn(BehaviourType.ObstacleAvoidance))
            {
                force = ObstacleAvoidance(_owner.GameWorld.Obstacles) * _obstacleAvoidancePriority;

                if (!AccumulateForce(ref _steeringForce, force))
                    return _steeringForce;
            }
            
            if (IsOn(BehaviourType.Separation))
            {
                force = Separation(_owner.GameWorld.Agents) * _separationPriority;

                if (!AccumulateForce(ref _steeringForce, force))
                    return _steeringForce;
            }
            
            if (IsOn(BehaviourType.Wander))
            {
                force = Wander() * _wanderPriority;

                if (!AccumulateForce(ref _steeringForce, force))
                    return _steeringForce;
            }
            
            //TODO: Add other steering forces too..
            //..
            //..
            ////

            return _steeringForce;
        }
        
        /// <summary>
        /// Calculates the steering force using the method 'Prioritized Dithering'
        /// ps. This method is cheaper than other two but accuracy is lower too.
        /// Needs fair amount of parameter/probability tweaking to work well.
        /// </summary>
        /// <returns></returns>
        private Vector2 Calculate_Dithered()
        {
            _steeringForce = default;

            const float probWallAvoidance = 0.1f;
            const float probObstacleAvoidance = 0.1f;
            const float probSeparation = 0.2f;
            const float probAlignment = 0.5f;
            const float probCohesion = 0.5f;
            const float probWander = 0.2f;

            if (IsOn(BehaviourType.WallAvoidance) && Random.Range(0, 1f) < probWallAvoidance)
            {
                _steeringForce = WallAvoidance(_owner.GameWorld.Walls) 
                                    * _wallAvoidanceWeight 
                                    / probWallAvoidance;

                if (_steeringForce != default)
                {
                    _steeringForce = _steeringForce.normalized *
                                     Mathf.Clamp(_steeringForce.magnitude, 0, _owner.MaxForce);

                    return _steeringForce;
                }
            }

            if (IsOn(BehaviourType.ObstacleAvoidance) && Random.Range(0, 1f) < probObstacleAvoidance)
            {
                _steeringForce += ObstacleAvoidance(_owner.GameWorld.Obstacles) 
                                  * _obstacleAvoidanceWeight 
                                  / probObstacleAvoidance;

                if (_steeringForce != default)
                {
                    _steeringForce = _steeringForce.normalized *
                                     Mathf.Clamp(_steeringForce.magnitude, 0, _owner.MaxForce);

                    return _steeringForce;
                }
            }

            if (IsOn(BehaviourType.Separation) && Random.Range(0, 1f) < probSeparation)
            {
                _steeringForce += Separation(_owner.GameWorld.Agents) 
                                    * _separationWeight 
                                    / probSeparation;

                if (_steeringForce != default)
                {
                    _steeringForce = _steeringForce.normalized *
                                     Mathf.Clamp(_steeringForce.magnitude, 0, _owner.MaxForce);

                    return _steeringForce;
                }
            }
            
            //etc. etc.

            return _steeringForce;
        }

        private bool AccumulateForce(ref Vector2 runningTotalForce, Vector2 forceToAdd)
        {
            var magnitudeSoFar = runningTotalForce.magnitude;

            var magnitudeRemaining = _owner.MaxForce - magnitudeSoFar;

            if (magnitudeRemaining <= 0)
                return false;

            var magnitudeToAdd = forceToAdd.magnitude;

            if (magnitudeToAdd < magnitudeRemaining)
                runningTotalForce += forceToAdd;
            else
                runningTotalForce += forceToAdd.normalized * magnitudeRemaining;

            return true;
        }

        //TODO
        public enum BehaviourType
        {
            None = -1,
            WallAvoidance = 100,
            ObstacleAvoidance = 200,
            Separation = 300,
            Wander = 400
        }
        
        //TODO
        private bool IsOn(BehaviourType behaviourType)
        {
            return onBehaviours.Contains(behaviourType);
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
            _wanderTargetVector += new Vector2(Random.Range(-1f, 1f) * wanderJitter, Random.Range(-1f, 1f) * wanderJitter);
            _wanderTargetVector.Normalize();
            _wanderTargetVector *= WanderRadius;
            var targetLocal = _wanderTargetVector + new Vector2(WanderDistance, 0);
            _wanderToPosition = _owner.Position + targetLocal;

            return _wanderToPosition - _owner.Position;
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

        private Vector2 Interpose(MovingEntity agentA, MovingEntity agentB)
        {
            var midpoint = (agentA.Position + agentB.Position) / 2;

            var etaToMidpoint = Vector2.Distance(_owner.Position, midpoint) / _owner.MaxSpeed;

            var predictedAPosition = agentA.Position + agentA.Velocity * etaToMidpoint;
            var predictedBPosition = agentB.Position + agentB.Velocity * etaToMidpoint;

            midpoint = (predictedAPosition + predictedBPosition) / 2;

            return Arrive(midpoint, Deceleration.Fast);
        }

        private Vector2 GetHidingPosition(Vector2 obstaclePos, float obstacleRadius, Vector2 targetPosition)
        {
            var distanceFromBoundary = 5;

            var distanceAway = obstacleRadius + distanceFromBoundary;

            var toObstacle = (obstaclePos - targetPosition).normalized;

            return (toObstacle * distanceAway) + obstaclePos;
        }

        private Vector2 Hide(MovingEntity target, List<BaseGameEntity> obstacles)
        {
            var distanceToClosest = float.MaxValue;
            Vector2 bestHidingSpot = default;
            
            foreach (var obstacle in obstacles)
            {
                var hidingSpot = GetHidingPosition(obstacle.Position, obstacle.BoundingRadius, target.Position);
                var distance = (hidingSpot - _owner.Position).sqrMagnitude;
                
                if(distance >= distanceToClosest)
                    continue;

                distanceToClosest = distance;
                bestHidingSpot = hidingSpot;
            }

            if (Math.Abs(distanceToClosest - float.MaxValue) < Mathf.Epsilon)
                return Evade(target);

            return Arrive(bestHidingSpot, Deceleration.Fast);
        }

        private Vector2 FollowPath()
        {
            if ((_path.CurrentWaypoint - _owner.Position).sqrMagnitude < _waypointSeekDistance)
                _path.SetNextWaypoint();

            if (!_path.IsFinished())
                return Seek(_path.CurrentWaypoint);

            return Arrive(_path.CurrentWaypoint, Deceleration.Normal);
        }

        private Vector2 OffsetPursuit(MovingEntity leader, Vector2 offset)
        {
            var worldOffsetPosition = leader.transform.TransformPoint(offset.ToVec3()).ToVec2();
            var toOffset = worldOffsetPosition - _owner.Position;

            var lookAheadTime = toOffset.magnitude / (_owner.MaxSpeed + leader.Speed);
            return Arrive(worldOffsetPosition + leader.Velocity * lookAheadTime, Deceleration.Fast);
        }

        private Vector2 Separation(List<MovingEntity> allAgents)
        {
            Vector2 steeringForce = default;

            foreach (var agent in allAgents)
            {
                if(agent == _owner || !agent.IsTagged)
                    continue;

                var toAgent = _owner.Position - agent.Position;
                steeringForce += toAgent.normalized / toAgent.magnitude;
            }

            return steeringForce;
        }

        private Vector2 Alignment(List<MovingEntity> allAgents)
        {
            Vector2 averageForward = default;

            var neighborCount = 0;
            
            foreach (var agent in allAgents)
            {
                if(agent == _owner || !agent.IsTagged)
                    continue;

                averageForward += agent.Heading;
                neighborCount++;
            }

            if (neighborCount > 0)
            {
                averageForward /= neighborCount;
                averageForward -= _owner.Heading;
            }

            return averageForward;
        }

        private Vector2 Cohesion(List<MovingEntity> allAgents)
        {
            Vector2 centerOfMass = default;
            Vector2 steeringForce = default;

            var neighborCount = 0;
            
            foreach (var agent in allAgents)
            {
                if(agent == _owner || !agent.IsTagged)
                    continue;

                centerOfMass += agent.Position;
                neighborCount++;
            }

            if (neighborCount > 0)
            {
                centerOfMass /= neighborCount;
                steeringForce = Seek(centerOfMass);
            }

            return steeringForce;
        }
        
        public void DrawGizmos()
        {
            //draw detection box
            Gizmos.color = Color.green;
            Gizmos.DrawWireCube((_owner.Position + (-_owner.Heading * _owner.BoundingRadius)).ToVec3(),
                new Vector3(_owner.BoundingRadius, _owner.BoundingRadius, _detectionBoxLength));
            
            //draw feelers
            Gizmos.color = Color.red;
            foreach (var feeler in _feelers)
            {
                // Gizmos.DrawRay(new Ray(_owner.Position.ToVec3(), feeler.ToVec3()));
                Gizmos.DrawLine(_owner.Position.ToVec3(), (feeler).ToVec3());
            }
            
            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(_wanderToPosition.ToVec3(), 0.2f);
        }
    }
}