using System;
using System.Collections.Generic;
using Stride.Core.Mathematics;

namespace VL.Espace.Utils.Polyline
{
    public interface IPointOps<T>
    {
        float Distance(T a, T b);
        float LengthSquared(T v);
        T Subtract(T a, T b);
        float Dot(T a, T b);
        T Scale(T v, float s);
        T Add(T a, T b);
        T Lerp(T a, T b, float t);
        bool Equals(T a, T b);
        T Clamp01(T v); // Optional, not used here
    }

    public class Vector3Ops : IPointOps<Vector3>
    {
        public float Distance(Vector3 a, Vector3 b) => Vector3.Distance(a, b);
        public float LengthSquared(Vector3 v) => v.LengthSquared();
        public Vector3 Subtract(Vector3 a, Vector3 b) => a - b;
        public float Dot(Vector3 a, Vector3 b) => Vector3.Dot(a, b);
        public Vector3 Scale(Vector3 v, float s) => v * s;
        public Vector3 Add(Vector3 a, Vector3 b) => a + b;
        public Vector3 Lerp(Vector3 a, Vector3 b, float t) => Vector3.Lerp(a, b, t);
        public bool Equals(Vector3 a, Vector3 b) => a == b;
        public Vector3 Clamp01(Vector3 v) => new Vector3(MathUtil.Clamp(v.X, 0, 1), MathUtil.Clamp(v.Y, 0, 1), MathUtil.Clamp(v.Z, 0, 1));
    }

    public class Vector2Ops : IPointOps<Vector2>
    {
        public float Distance(Vector2 a, Vector2 b) => Vector2.Distance(a, b);
        public float LengthSquared(Vector2 v) => v.LengthSquared();
        public Vector2 Subtract(Vector2 a, Vector2 b) => a - b;
        public float Dot(Vector2 a, Vector2 b) => Vector2.Dot(a, b);
        public Vector2 Scale(Vector2 v, float s) => v * s;
        public Vector2 Add(Vector2 a, Vector2 b) => a + b;
        public Vector2 Lerp(Vector2 a, Vector2 b, float t) => Vector2.Lerp(a, b, t);
        public bool Equals(Vector2 a, Vector2 b) => a == b;
        public Vector2 Clamp01(Vector2 v) => new Vector2(MathUtil.Clamp(v.X, 0, 1), MathUtil.Clamp(v.Y, 0, 1));
    }

    public class FloatOps : IPointOps<float>
    {
        public float Distance(float a, float b) => Math.Abs(a - b);
        public float LengthSquared(float v) => v * v;
        public float Subtract(float a, float b) => a - b;
        public float Dot(float a, float b) => a * b;
        public float Scale(float v, float s) => v * s;
        public float Add(float a, float b) => a + b;
        public float Lerp(float a, float b, float t) => a + (b - a) * t;
        public bool Equals(float a, float b) => Math.Abs(a - b) < 1e-6f;
        public float Clamp01(float v) => MathUtil.Clamp(v, 0, 1);
    }

    public class QuaternionOps : IPointOps<Quaternion>
    {
        public float Distance(Quaternion a, Quaternion b)
        {
            float dot = MathF.Abs(Quaternion.Dot(a, b));
            dot = MathF.Min(dot, 1.0f);
            return 2.0f * MathF.Acos(dot);
        }
        public float LengthSquared(Quaternion v)
        {
            return v.LengthSquared();
        }
        public Quaternion Subtract(Quaternion a, Quaternion b)
        {
            return Quaternion.Multiply(a, Quaternion.Invert(b));
        }
        public float Dot(Quaternion a, Quaternion b)
        {
            return Quaternion.Dot(a, b);
        }
        public Quaternion Scale(Quaternion v, float s)
        {
            return Quaternion.Slerp(Quaternion.Identity, v, s);
        }
        public Quaternion Add(Quaternion a, Quaternion b)
        {
            return new Quaternion(a.X + b.X, a.Y + b.Y, a.Z + b.Z, a.W + b.W);
        }
        public Quaternion Lerp(Quaternion a, Quaternion b, float t)
        {
            return Quaternion.Slerp(a, b, t);
        }
        public bool Equals(Quaternion a, Quaternion b)
        {
            return Distance(a, b) < 1e-6f;
        }
        public Quaternion Clamp01(Quaternion v)
        {
            return Quaternion.Normalize(v);
        }
    }

    public static class PolylineSimplification
    {
        public static List<T> RdpDecimate<T>(List<T> points, float epsilon, float duplicateEpsilon = 0f, IPointOps<T> ops = null)
        {
            if (ops == null) throw new ArgumentNullException(nameof(ops));
            if (points == null || points.Count < 3)
                return points == null ? new List<T>() : new List<T>(points);

            List<T> filteredPoints = points;

            if (duplicateEpsilon > 0f)
            {
                filteredPoints = RemoveDuplicates(points, duplicateEpsilon, ops);
                if (filteredPoints.Count < 3)
                    return filteredPoints;
            }

            if (epsilon <= 0f)
                return new List<T>(filteredPoints);

            int n = filteredPoints.Count;
            var keep = new bool[n];
            keep[0] = true;
            keep[n - 1] = true;

            var stack = new Stack<(int, int)>();
            stack.Push((0, n - 1));

            while (stack.Count > 0)
            {
                var (start, end) = stack.Pop();
                float maxDist = 0f;
                int index = -1;
                for (int i = start + 1; i < end; i++)
                {
                    float dist = PerpendicularDistance(filteredPoints[i], filteredPoints[start], filteredPoints[end], ops);
                    if (dist > maxDist)
                    {
                        maxDist = dist;
                        index = i;
                    }
                }
                if (index != -1 && maxDist > epsilon)
                {
                    keep[index] = true;
                    stack.Push((start, index));
                    stack.Push((index, end));
                }
            }

            var result = new List<T>();
            for (int i = 0; i < n; i++)
                if (keep[i])
                    result.Add(filteredPoints[i]);
            return result;
        }

        public static List<T> RemoveDuplicates<T>(List<T> points, float duplicateEpsilon, IPointOps<T> ops)
        {
            var filtered = new List<T>();
            if (points == null || points.Count == 0)
                return filtered;

            filtered.Add(points[0]);
            for (int i = 1; i < points.Count; i++)
            {
                if (ops.Distance(filtered[filtered.Count - 1], points[i]) > duplicateEpsilon)
                {
                    filtered.Add(points[i]);
                }
            }
            return filtered;
        }

        private static float PerpendicularDistance<T>(T point, T lineStart, T lineEnd, IPointOps<T> ops)
        {
            var line = ops.Subtract(lineEnd, lineStart);
            float lengthSquared = ops.LengthSquared(line);
            if (lengthSquared == 0.0f) return ops.Distance(point, lineStart);

            float t = ops.Dot(ops.Subtract(point, lineStart), line) / lengthSquared;
            t = MathUtil.Clamp(t, 0f, 1f);
            var projection = ops.Add(lineStart, ops.Scale(line, t));
            return ops.Distance(point, projection);
        }

        public static List<T> ResampleDistance<T>(List<T> points, float distance, IPointOps<T> ops)
        {
            if (points == null || points.Count < 2 || distance <= 0f)
                return new List<T>(points ?? new List<T>());

            var arcLengths = new List<float> { 0f };
            for (int i = 1; i < points.Count; i++)
                arcLengths.Add(arcLengths[i - 1] + ops.Distance(points[i - 1], points[i]));

            float totalLength = arcLengths[arcLengths.Count - 1];
            int numPoints = (int)Math.Floor(totalLength / distance);
            var result = new List<T> { points[0] };

            int seg = 0;
            for (int i = 1; i <= numPoints; i++)
            {
                float targetLen = i * distance;
                while (seg < arcLengths.Count - 2 && arcLengths[seg + 1] < targetLen)
                    seg++;

                float segLen = arcLengths[seg + 1] - arcLengths[seg];
                float t = (segLen > 0f) ? (targetLen - arcLengths[seg]) / segLen : 0f;
                T p = ops.Lerp(points[seg], points[seg + 1], t);
                result.Add(p);
            }

            if (!ops.Equals(result[result.Count - 1], points[points.Count - 1]))
                result.Add(points[points.Count - 1]);

            return result;
        }

        /// <summary>
        /// Samples a value from the polyline defined by the input points, using a normalized position t [0..1].
        /// Returns the interpolated position at that relative length along the polyline.
        /// </summary>
        public static T SampleNormalized<T>(List<T> points, float t, IPointOps<T> ops)
        {
            if (points == null || points.Count == 0)
                throw new ArgumentException("Points must not be null or empty.");
            if (points.Count == 1 || t <= 0f)
                return points[0];
            if (t >= 1f)
                return points[points.Count - 1];

            // Compute cumulative arc lengths
            var arcLengths = new List<float> { 0f };
            for (int i = 1; i < points.Count; i++)
                arcLengths.Add(arcLengths[i - 1] + ops.Distance(points[i - 1], points[i]));

            float totalLength = arcLengths[arcLengths.Count - 1];
            float targetLen = t * totalLength;

            // Find the segment containing targetLen
            int seg = 0;
            while (seg < arcLengths.Count - 2 && arcLengths[seg + 1] < targetLen)
                seg++;

            float segLen = arcLengths[seg + 1] - arcLengths[seg];
            float localT = (segLen > 0f) ? (targetLen - arcLengths[seg]) / segLen : 0f;
            return ops.Lerp(points[seg], points[seg + 1], localT);
        }

        /// <summary>
        /// Samples a value from the polyline defined by the input points, using a normalized position t [0..1].
        /// Interpolates by index position only (not arc length).
        /// </summary>
        public static T SampleNormalizedByIndex<T>(List<T> points, float t, IPointOps<T> ops)
        {
            if (points == null || points.Count == 0)
                throw new ArgumentException("Points must not be null or empty.");
            if (points.Count == 1 || t <= 0f)
                return points[0];
            if (t >= 1f)
                return points[points.Count - 1];

            float ftIndex = t * (points.Count - 1);
            int idx = (int)Math.Floor(ftIndex);
            float localT = ftIndex - idx;

            if (idx >= points.Count - 1)
                return points[points.Count - 1];

            return ops.Lerp(points[idx], points[idx + 1], localT);
        }
    }
}