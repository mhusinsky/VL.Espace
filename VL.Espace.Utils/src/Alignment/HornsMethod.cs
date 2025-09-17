// Horn's Method implementation for use in VVVV Gamma
// by Matthias Husinsky and Cody (Sourcegraph AI Assistant)

using System;
using System.Collections.Generic;
using System.Linq;
using Stride.Core.Mathematics; // Stride3D's Math Library
using MathNetMatrix = MathNet.Numerics.LinearAlgebra.Matrix<float>;  // Use float for performance
using MathNetVector = MathNet.Numerics.LinearAlgebra.Vector<float>;
using MathNetNumerics = MathNet.Numerics.LinearAlgebra.Single; // Single-precision Math.NET

namespace VL.Espace.Utils.Alignment
{
    public static class HornsMethod
    {
        /// <summary>
        /// Aligns two sets of 3D points using Horn's Method, which uses quaternion-based optimization
        /// to compute the optimal rotation, translation, and optional scaling to minimize the mean squared error.
        /// This method is numerically stable and works well for small point sets.
        /// </summary>
        /// <param name="sourcePoints">The list of source points to align.</param>
        /// <param name="targetPoints">The list of target points to align to.</param>
        /// <param name="scale">The computed scaling factor (output).</param>
        /// <param name="rotationMatrix">The computed rotation matrix (output).</param>
        /// <param name="rotationQuaternion">The computed rotation as a quaternion (output).</param>
        /// <param name="translation">The computed translation vector (output).</param>
        /// <param name="allowScaling">Indicates whether scaling is allowed in the alignment. Defaults to <c>false</c>.</param>
        /// <exception cref="ArgumentException">Thrown if the point sets have different sizes or are empty.</exception>
        public static void HornsAlign(
            List<Vector3> sourcePoints,
            List<Vector3> targetPoints,
            out float scale,
            out Matrix rotationMatrix,
            out Quaternion rotationQuaternion,
            out Vector3 translation,
            bool allowScaling = false)
        {
            if (sourcePoints.Count != targetPoints.Count || sourcePoints.Count == 0)
                throw new ArgumentException("Point sets must have the same non-zero number of elements.");

            int numPoints = sourcePoints.Count;

            // Compute centroids
            Vector3 centroidSource = ComputeCentroid(sourcePoints);
            Vector3 centroidTarget = ComputeCentroid(targetPoints);

            // Center the points around the origin
            var centeredSource = CenterPoints(sourcePoints, centroidSource);
            var centeredTarget = CenterPoints(targetPoints, centroidTarget);

            // Build Horn's 4x4 matrix N
            MathNetMatrix N = BuildHornsMatrix(centeredSource, centeredTarget);

            // Find the eigenvector corresponding to the largest eigenvalue
            var eigenDecomp = N.Evd();
            var eigenValues = eigenDecomp.EigenValues;
            var eigenVectors = eigenDecomp.EigenVectors;

            // Find the index of the largest eigenvalue
            int maxIndex = 0;
            float maxEigenValue = (float)eigenValues[0].Real;
            for (int i = 1; i < eigenValues.Count; i++)
            {
                if ((float)eigenValues[i].Real > maxEigenValue)
                {
                    maxEigenValue = (float)eigenValues[i].Real;
                    maxIndex = i;
                }
            }

            // Extract the optimal quaternion (eigenvector corresponding to largest eigenvalue)
            var optimalEigenVector = eigenVectors.Column(maxIndex);

            // Horn's quaternion convention: [q0, q1, q2, q3] = [w, x, y, z]
            float qw = optimalEigenVector[0];
            float qx = optimalEigenVector[1];
            float qy = optimalEigenVector[2];
            float qz = optimalEigenVector[3];

            // Normalize quaternion
            float qNorm = (float)Math.Sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
            if (qNorm < 1e-10f)
                throw new InvalidOperationException("Degenerate quaternion computed.");

            qw /= qNorm;
            qx /= qNorm;
            qy /= qNorm;
            qz /= qNorm;

            // Ensure positive scalar part to resolve sign ambiguity
            if (qw < 0)
            {
                qw = -qw;
                qx = -qx;
                qy = -qy;
                qz = -qz;
            }

            // Create Stride quaternion (note: Stride uses XYZW order)
            rotationQuaternion = new Quaternion(qx, qy, qz, qw);

            // Convert quaternion to rotation matrix
            rotationMatrix = Matrix.RotationQuaternion(rotationQuaternion);

            // Compute scale if scaling is enabled
            scale = 1.0f;
            if (allowScaling)
            {
                scale = ComputeScale(centeredSource, centeredTarget, rotationMatrix);
            }

            // Compute translation: centroidTarget - scale * R * centroidSource
            Vector4 transformedCentroidSource = Vector4.Transform(new Vector4(centroidSource, 1.0f), rotationMatrix);
            Vector3 rotatedScaledCentroidSource = new Vector3(transformedCentroidSource.X, transformedCentroidSource.Y, transformedCentroidSource.Z) * scale;
            translation = centroidTarget - rotatedScaledCentroidSource;
        }

        /// <summary>
        /// Computes the residual error after alignment using Horn's method.
        /// </summary>
        /// <param name="sourcePoints">The source points.</param>
        /// <param name="targetPoints">The target points.</param>
        /// <param name="scale">The scaling factor from alignment.</param>
        /// <param name="rotationMatrix">The rotation matrix from alignment.</param>
        /// <param name="translation">The translation vector from alignment.</param>
        /// <returns>The root mean square error after alignment.</returns>
        public static float ComputeAlignmentError(
            List<Vector3> sourcePoints,
            List<Vector3> targetPoints,
            float scale,
            Matrix rotationMatrix,
            Vector3 translation)
        {
            if (sourcePoints.Count != targetPoints.Count)
                throw new ArgumentException("Point sets must have the same number of elements.");

            float totalSquaredError = 0f;

            for (int i = 0; i < sourcePoints.Count; i++)
            {
                // Transform source point: T + s * R * P
                Vector4 rotatedSource4 = Vector4.Transform(new Vector4(sourcePoints[i], 1.0f), rotationMatrix);
                Vector3 rotatedSource = new Vector3(rotatedSource4.X, rotatedSource4.Y, rotatedSource4.Z);
                Vector3 alignedSource = translation + scale * rotatedSource;

                // Compute squared distance to target
                Vector3 diff = alignedSource - targetPoints[i];
                totalSquaredError += diff.LengthSquared();
            }

            return (float)Math.Sqrt(totalSquaredError / sourcePoints.Count);
        }

        /// <summary>
        /// Alternative method that directly returns the optimal quaternion without matrix conversion.
        /// Useful when you primarily need the quaternion representation.
        /// </summary>
        /// <param name="sourcePoints">The list of source points to align.</param>
        /// <param name="targetPoints">The list of target points to align to.</param>
        /// <param name="scale">The computed scaling factor (output).</param>
        /// <param name="rotationQuaternion">The computed rotation as a quaternion (output).</param>
        /// <param name="translation">The computed translation vector (output).</param>
        /// <param name="allowScaling">Indicates whether scaling is allowed in the alignment.</param>
        public static void HornsAlignQuaternion(
            List<Vector3> sourcePoints,
            List<Vector3> targetPoints,
            out float scale,
            out Quaternion rotationQuaternion,
            out Vector3 translation,
            bool allowScaling = false)
        {
            HornsAlign(sourcePoints, targetPoints, out scale, out Matrix _, out rotationQuaternion, out translation, allowScaling);
        }

        private static Vector3 ComputeCentroid(List<Vector3> points)
        {
            Vector3 sum = Vector3.Zero;
            foreach (var p in points)
                sum += p;
            return sum / points.Count;
        }

        private static List<Vector3> CenterPoints(List<Vector3> points, Vector3 centroid)
        {
            return points.Select(p => p - centroid).ToList();
        }

        private static MathNetMatrix BuildHornsMatrix(List<Vector3> source, List<Vector3> target)
        {
            int numPoints = source.Count;

            // Initialize the symmetric 4x4 matrix N
            MathNetMatrix N = MathNetNumerics.DenseMatrix.Create(4, 4, 0);

            // Accumulate the cross-correlation terms
            for (int i = 0; i < numPoints; i++)
            {
                Vector3 p = source[i];  // source point
                Vector3 q = target[i];  // target point

                // Compute the cross-correlation matrix elements
                float Sxx = p.X * q.X; float Sxy = p.X * q.Y; float Sxz = p.X * q.Z;
                float Syx = p.Y * q.X; float Syy = p.Y * q.Y; float Syz = p.Y * q.Z;
                float Szx = p.Z * q.X; float Szy = p.Z * q.Y; float Szz = p.Z * q.Z;

                // Build Horn's N matrix according to the correct formulation
                N[0, 0] += Sxx + Syy + Szz;  // N11: trace of cross-correlation matrix
                N[0, 1] += Syz - Szy;        // N12
                N[0, 2] += Szx - Sxz;        // N13  
                N[0, 3] += Sxy - Syx;        // N14

                N[1, 0] += Syz - Szy;        // N21 (symmetric)
                N[1, 1] += Sxx - Syy - Szz; // N22
                N[1, 2] += Sxy + Syx;        // N23
                N[1, 3] += Szx + Sxz;        // N24

                N[2, 0] += Szx - Sxz;        // N31 (symmetric)
                N[2, 1] += Sxy + Syx;        // N32 (symmetric)
                N[2, 2] += -Sxx + Syy - Szz; // N33
                N[2, 3] += Syz + Szy;        // N34

                N[3, 0] += Sxy - Syx;        // N41 (symmetric)
                N[3, 1] += Szx + Sxz;        // N42 (symmetric)
                N[3, 2] += Syz + Szy;        // N43 (symmetric)
                N[3, 3] += -Sxx - Syy + Szz; // N44
            }

            return N;
        }

        private static float ComputeScale(List<Vector3> source, List<Vector3> target, Matrix rotationMatrix)
        {
            float numerator = 0f;
            float denominator = 0f;

            for (int i = 0; i < source.Count; i++)
            {
                // Apply rotation to source point
                Vector4 rotatedSource4 = Vector4.Transform(new Vector4(source[i], 1.0f), rotationMatrix);
                Vector3 rotatedSource = new Vector3(rotatedSource4.X, rotatedSource4.Y, rotatedSource4.Z);

                // Accumulate for scale computation
                numerator += Vector3.Dot(target[i], rotatedSource);
                denominator += Vector3.Dot(rotatedSource, rotatedSource);
            }

            return denominator > 1e-10f ? numerator / denominator : 1.0f;
        }
    }
}