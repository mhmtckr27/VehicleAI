using Extensions;
using UnityEngine;

namespace VehicleAI
{
    public class Wall : MonoBehaviour
    {
        [SerializeField] private Transform fromTransform;
        [SerializeField] private Transform toTransform;
        [SerializeField] private BoxCollider _wallCollider;

        public Vector2 From => fromTransform.position.ToVec2();
        public Vector2 To => toTransform.position.ToVec2();
        public Vector2 Normal { get; private set; }

        private void Awake()
        {
        //     _wallCollider ??= GetComponent<BoxCollider>();
        //
        //     From = _wallCollider.bounds.min.ToVec2();
        //     To = _wallCollider.bounds.max.ToVec2();
        //     Normal = Vector2.Perpendicular(To - From);
            Normal = Vector2.Perpendicular(To - From);        
        }
        //
        private void OnValidate()
        {
        //     From = _wallCollider.bounds.min.ToVec2();
        //     To = _wallCollider.bounds.max.ToVec2();
            Normal = Vector2.Perpendicular(To - From);        
        }

        private void OnDrawGizmos()
        {
            Gizmos.color = Color.black;
            Gizmos.DrawLine(From.ToVec3(), To.ToVec3());
        }
    }
}