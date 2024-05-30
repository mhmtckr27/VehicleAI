using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Utils
{
    public class Smoother
    {
        private List<Vector2> _history;

        int nextUpdateSlotIndex;

        //an example of the 'zero' value of the type to be smoothed. This
        //would be something like Vector2D(0,0)
        private Vector2 _zeroValue;

        //to instantiate a Smoother pass it the number of samples you want
        //to use in the smoothing, and an exampe of a 'zero' type
        public Smoother(int sampleSize, Vector2 zeroValue)
        {
            _history = new List<Vector2>();

            for (var i = 0; i < sampleSize; i++)
            {
                _history.Add(zeroValue);
            }
        }

        //each time you want to get a new average, feed it the most recent value
        //and this method will return an average over the last SampleSize updates
        public Vector2 Update(Vector2 mostRecentValue)
        {
            //overwrite the oldest value with the newest
            _history[nextUpdateSlotIndex++] = mostRecentValue;

            //make sure m_iNextUpdateSlot wraps around. 
            if (nextUpdateSlotIndex == _history.Count)
                nextUpdateSlotIndex = 0;

            //now to calculate the average of the history list
            var sum = _zeroValue;
            
            foreach (var value in _history)
            {
                sum += value;
            }

            return sum /= _history.Count;
        }
    }
}