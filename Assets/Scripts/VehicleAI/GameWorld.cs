using System.Collections.Generic;
using UnityEngine;

namespace VehicleAI
{
    public class GameWorld : MonoBehaviour
    {
        public const float WorldYPos = 0f;

        private List<BaseGameEntity> _obstacles;

        public void TagObstaclesWithinViewRange(MovingEntity vehicle, float detectionBoxLength)
        {
            TagNeighbors(vehicle, _obstacles, detectionBoxLength);
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