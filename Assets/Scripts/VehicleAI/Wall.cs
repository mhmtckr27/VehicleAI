using UnityEngine;

namespace VehicleAI
{
    public class Wall
    {
        public Vector2 From { get; private set; }
        public Vector2 To { get; private set; }
        public Vector2 Normal { get; private set; }
    }
}