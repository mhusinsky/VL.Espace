// Procrustes Analysis implementation for use in VVVV Gamma
// by Matthias Husinsky and Cody (Sourcegraph AI Assistant)

using System;
using System.Collections.Generic;
using System.Linq;
using Stride.Core.Mathematics; // Stride3D's Math Library
using MathNetMatrix = MathNet.Numerics.LinearAlgebra.Matrix<float>;  // Use float for performance
using MathNetNumerics = MathNet.Numerics.LinearAlgebra.Single; // Single-precision Math.NET

namespace VL.Espace.Utils.Alignment
{
    public static class ProcrustesAnalysis
    {
        /// <summary>
        /// Aligns two sets of 3D points using Procrustes Analysis, which computes the optimal
        /// rotation, translation, and scaling to minimize the mean squared error between the points.
        /// This is similar to Kabsch-Umeyama but uses the classical Procrustes approach.
        /// </summary>
        /// <param name="sourcePoints">The list of source points to align.</param>
        /// <param name="targetPoints">The list of target points to align to.</param>
        /// <param name="scale">The computed scaling factor (output).</param>
        /// <param name="rotationMatrix">The computed rotation matrix (output).</param>
        /// <param name="rotationQuaternion">The computed rotation as a quaternion (output).</param>
        /// <param name="translation">The computed translation vector (output).</param>
        /// <param name="allowScaling">Indicates whether scaling is allowed in the alignment. Defaults to <c>true</c>.</param>
        /// <param name="allowReflection">Indicates whether reflection is allowed (improper rotations). Defaults to <c>false</c>.</param>
        /// <exception cref="ArgumentException">Thrown if the point sets have different sizes or are empty.</exception>
        public static void ProcrustesAlign(
            List<Vector3> sourcePoints,
            List<Vector3> targetPoints,
            out float scale,
            out Matrix rotationMatrix,
            out Quaternion rotationQuaternion,
            out Vector3 translation,
            bool allowScaling = true,
            bool allowReflection = false)
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

            // Compute scaling factors (Frobenius norms)
            float normSource = ComputeFrobeniusNorm(centeredSource);
            float normTarget = ComputeFrobeniusNorm(centeredTarget);

            // Normalize point sets for rotation computation
            var normalizedSource = allowScaling ? NormalizePoints(centeredSource, normSource) : centeredSource;
            var normalizedTarget = allowScaling ? NormalizePoints(centeredTarget, normTarget) : centeredTarget;

            // Compute cross-covariance matrix
            MathNetMatrix covarianceMatrix = ComputeCovarianceMatrix(normalizedSource, normalizedTarget);

            // Perform Singular Value Decomposition (SVD)
            var svd = covarianceMatrix.Svd();
            MathNetMatrix U = svd.U;
            MathNetMatrix Vt = svd.VT;
            var singularValues = svd.S;

            // Compute optimal rotation/reflection
            MathNetMatrix rotationMat = Vt.Transpose() * U.Transpose();

            // Handle reflection (ensure proper rotation if not allowed)
            if (!allowReflection && rotationMat.Determinant() < 0)
            {
                // Flip the last column of V (or equivalently, last row of Vt)
                var VtFixed = Vt.ToArray();
                VtFixed[2, 0] *= -1;
                VtFixed[2, 1] *= -1;
                VtFixed[2, 2] *= -1;
                rotationMat = MathNetNumerics.DenseMatrix.OfArray(VtFixed).Transpose() * U.Transpose();
            }

            // Convert Math.NET rotation matrix to Stride3D Matrix
            rotationMatrix = ConvertToStrideMatrix(rotationMat);

            // Convert Rotation Matrix to Quaternion
            rotationQuaternion = Quaternion.RotationMatrix(rotationMatrix);

            // Compute scale factor
            if (allowScaling && normSource > 1e-10f) // Avoid division by zero
            {
                // Procrustes scaling: ratio of norms, adjusted by trace of rotation applied to normalized data
                float traceSum = 0f;
                for (int i = 0; i < Math.Min(3, singularValues.Count); i++)
                {
                    traceSum += singularValues[i];
                }

                // Handle reflection case for scaling
                if (allowReflection || rotationMat.Determinant() >= 0)
                {
                    scale = (normTarget / normSource) * traceSum;
                }
                else
                {
                    // Subtract the last singular value if we flipped the rotation
                    scale = (normTarget / normSource) * (traceSum - 2 * singularValues[singularValues.Count - 1]);
                }
            }
            else
            {
                scale = 1.0f;
            }

            // Compute translation: target centroid - scaled rotated source centroid
            Vector4 transformedCentroidSource = Vector4.Transform(new Vector4(scale * centroidSource, 1.0f), rotationMatrix);
            translation = centroidTarget - new Vector3(transformedCentroidSource.X, transformedCentroidSource.Y, transformedCentroidSource.Z);
        }

        /// <summary>
        /// Computes the Procrustes distance between two aligned point sets.
        /// This is the sum of squared distances after optimal alignment.
        /// </summary>
        /// <param name="sourcePoints">The source points.</param>
        /// <param name="targetPoints">The target points.</param>
        /// <param name="scale">The scaling factor from alignment.</param>
        /// <param name="rotationMatrix">The rotation matrix from alignment.</param>
        /// <param name="translation">The translation vector from alignment.</param>
        /// <returns>The Procrustes distance (sum of squared errors).</returns>
        public static float ComputeProcrustesDistance(
            List<Vector3> sourcePoints,
            List<Vector3> targetPoints,
            float scale,
            Matrix rotationMatrix,
            Vector3 translation)
        {
            if (sourcePoints.Count != targetPoints.Count)
                throw new ArgumentException("Point sets must have the same number of elements.");

            float totalDistance = 0f;

            for (int i = 0; i < sourcePoints.Count; i++)
            {
                // Transform source point
                Vector4 transformedSource = Vector4.Transform(new Vector4(scale * sourcePoints[i], 1.0f), rotationMatrix);
                Vector3 alignedSource = new Vector3(transformedSource.X, transformedSource.Y, transformedSource.Z) + translation;

                // Compute squared distance to target
                Vector3 diff = alignedSource - targetPoints[i];
                totalDistance += diff.LengthSquared();
            }

            return totalDistance;
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

        private static float ComputeFrobeniusNorm(List<Vector3> points)
        {
            float sumSquared = 0f;
            foreach (var p in points)
            {
                sumSquared += p.LengthSquared();
            }
            return (float)Math.Sqrt(sumSquared);
        }

        private static List<Vector3> NormalizePoints(List<Vector3> points, float norm)
        {
            if (norm < 1e-10f) return points; // Avoid division by zero
            return points.Select(p => p / norm).ToList();
        }

        private static MathNetMatrix ComputeCovarianceMatrix(List<Vector3> source, List<Vector3> target)
        {
            int numPoints = source.Count;
            MathNetMatrix H = MathNetNumerics.DenseMatrix.Create(3, 3, 0);

            for (int i = 0; i < numPoints; i++)
            {
                MathNetMatrix sourceVec = VectorToMathNetMatrix(source[i]);
                MathNetMatrix targetVec = VectorToMathNetMatrix(target[i]);
                H += sourceVec * targetVec.Transpose();
            }

            return H;
        }

        private static MathNetMatrix VectorToMathNetMatrix(Vector3 v)
        {
            return MathNetNumerics.DenseMatrix.OfArray(new float[,] { { v.X }, { v.Y }, { v.Z } });
        }

        private static Matrix ConvertToStrideMatrix(MathNetMatrix rotationMat)
        {
            // Convert Math.NET row-major matrix to Stride's column-major format by transposing
            MathNetMatrix transposed = rotationMat.Transpose();

            return new Matrix(
                transposed[0, 0], transposed[0, 1], transposed[0, 2], 0,
                transposed[1, 0], transposed[1, 1], transposed[1, 2], 0,
                transposed[2, 0], transposed[2, 1], transposed[2, 2], 0,
                0, 0, 0, 1);
        }
    }
}


