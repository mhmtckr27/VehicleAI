using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Serialization;

namespace VehicleAI
{
    public class GameWorld : MonoBehaviour
    {
        public const float WorldYPos = 0f;

        public List<BaseGameEntity> Obstacles { get; private set; }
        public List<Wall> Walls { get; private set; }
        public List<MovingEntity> Agents { get; private set; }
        public List<BaseGameEntity> AgentsAsBaseGameEntity { get; set; }

        private void Awake()
        {
            Agents = FindObjectsByType<MovingEntity>(FindObjectsInactive.Include, FindObjectsSortMode.None).ToList();

            AgentsAsBaseGameEntity = Agents.Select(entity => entity as BaseGameEntity).ToList();
        }
        
        public void TagAgentsWithinRange(MovingEntity vehicle, float detectionBoxLength)
        {
            TagNeighbors(vehicle, AgentsAsBaseGameEntity, detectionBoxLength);
        }
        
        public void TagObstaclesWithinViewRange(MovingEntity vehicle, float detectionBoxLength)
        {
            TagNeighbors(vehicle, Obstacles, detectionBoxLength);
        }
        
        private void TagNeighbors(BaseGameEntity entity, List<BaseGameEntity> ContainerOfEntities, double radius)
        {
            foreach (var nearEntity in ContainerOfEntities)
            {
                nearEntity.Untag();

                var to = nearEntity.Position - entity.Position;

                var range = radius + nearEntity.BoundingRadius;

                if (nearEntity != entity && to.sqrMagnitude < range * range)
                    nearEntity.Tag();
            }
        }
    }
}