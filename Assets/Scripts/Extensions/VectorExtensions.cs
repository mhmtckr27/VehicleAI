using UnityEngine;

namespace Extensions
{
    public static class VectorExtensions
    {
        public static Vector2 ToVec2(this Vector3 vector3)
        {
            return new Vector2(vector3.x, vector3.z);
        }
        
        public static Vector3 ToVec3(this Vector2 vector3, float y = 0)
        {
            return new Vector3(vector3.x, y, vector3.y);
        }
    }
}