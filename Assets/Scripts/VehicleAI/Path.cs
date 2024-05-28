using System.Collections.Generic;
using UnityEngine;

namespace VehicleAI
{
    public class Path
    {
        private List<Vector2> _waypoints;

        public Vector2 CurrentWaypoint { get; private set; }

        private int _currentWaypointIndex;
        private bool _isClosedLoop;

        public void SetNextWaypoint()
        {
            _currentWaypointIndex++;

            if (_currentWaypointIndex >= _waypoints.Count)
            {
                if (_isClosedLoop)
                {
                    _currentWaypointIndex = 0;
                }
                else
                {
                    return;
                }
            }

            CurrentWaypoint = _waypoints[_currentWaypointIndex];
        }

        public bool IsFinished() => !_isClosedLoop && _currentWaypointIndex == _waypoints.Count;
    }
}