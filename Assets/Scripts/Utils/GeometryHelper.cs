using System;
using UnityEngine;

namespace Utils
{
    public class GeometryHelper
    {
        public static bool LineIntersection2D(Vector2 A, Vector2 B, Vector2 C, Vector2 D)
        {
            double rTop = (A.y - C.y) * (D.x - C.x) - (A.x - C.x) * (D.y - C.y);
            double sTop = (A.y - C.y) * (B.x - A.x) - (A.x - C.x) * (B.y - A.y);

            double Bot = (B.x - A.x) * (D.y - C.y) - (B.y - A.y) * (D.x - C.x);

            if (Bot == 0) //parallel
            {
                return false;
            }

            double r = rTop / Bot;
            double s = sTop / Bot;

            if ((r > 0) && (r < 1) && (s > 0) && (s < 1))
            {
                //lines intersect
                return true;
            }

            //lines do not intersect
            return false;
        }

        public static bool LineIntersection2D(Vector2 A, Vector2 B, Vector2 C, Vector2 D, ref double dist)
        {
            double rTop = (A.y - C.y) * (D.x - C.x) - (A.x - C.x) * (D.y - C.y);
            double sTop = (A.y - C.y) * (B.x - A.x) - (A.x - C.x) * (B.y - A.y);

            double Bot = (B.x - A.x) * (D.y - C.y) - (B.y - A.y) * (D.x - C.x);


            if (Bot == 0) //parallel
            {
                if (IsEqual(rTop, 0) && IsEqual(sTop, 0))
                {
                    return true;
                }

                return false;
            }

            double r = rTop / Bot;
            double s = sTop / Bot;

            if ((r > 0) && (r < 1) && (s > 0) && (s < 1))
            {
                dist = Vec2DDistance(A, B) * r;

                return true;
            }

            else
            {
                dist = 0;

                return false;
            }
        }

        public static bool LineIntersection2D(Vector2 A, Vector2 B, Vector2 C, Vector2 D, ref double dist, Vector2 point)
        {
            var rTop = (A.y - C.y) * (D.x - C.x) - (A.x - C.x) * (D.y - C.y);
            var rBot = (B.x - A.x) * (D.y - C.y) - (B.y - A.y) * (D.x - C.x);

            var sTop = (A.y - C.y) * (B.x - A.x) - (A.x - C.x) * (B.y - A.y);
            var sBot = (B.x - A.x) * (D.y - C.y) - (B.y - A.y) * (D.x - C.x);

            if ((rBot == 0) || (sBot == 0))
            {
                //lines are parallel
                return false;
            }

            var r = rTop / rBot;
            var s = sTop / sBot;

            if ((r > 0) && (r < 1) && (s > 0) && (s < 1))
            {
                dist = Vec2DDistance(A, B) * r;

                point = A + r * (B - A);

                return true;
            }

            else
            {
                dist = 0;

                return false;
            }
        }
        
        private static bool IsEqual(float a, float b)
        {
            return Mathf.Abs(a-b) < 1E-12;
        }

        private static bool IsEqual(double a, double b)
        {
            return Math.Abs(a-b) < 1E-12;
        }
        
        private static float Vec2DDistance(Vector2 v1, Vector2 v2)
        {
            var ySeparation = v2.y - v1.y;
            var xSeparation = v2.x - v1.x;

            return Mathf.Sqrt(ySeparation*ySeparation + xSeparation*xSeparation);
        }
    }
}