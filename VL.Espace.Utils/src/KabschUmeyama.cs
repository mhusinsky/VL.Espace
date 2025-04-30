// KabschUmeyama implementation for use in VVVV Gamma
// by Matthias Husinsky and ChatGPT

using System;
using System.Collections.Generic;
using System.Linq;
using Stride.Core.Mathematics; // Stride3D's Math Library
using MathNetMatrix = MathNet.Numerics.LinearAlgebra.Matrix<float>;  // Use float for performance
using MathNetNumerics = MathNet.Numerics.LinearAlgebra.Single; // Single-precision Math.NET

public static class KabschUmeyama
{
    /// <summary>
    /// Aligns two sets of 3D points using the Kabsch-Umeyama algorithm, which computes the optimal
    /// rotation, translation, and optional scaling to minimize the mean squared error between the points.
    /// </summary>
    /// <param name="sourcePoints">The list of source points to align.</param>
    /// <param name="targetPoints">The list of target points to align to.</param>
    /// <param name="scale">The computed scaling factor (output).</param>
    /// <param name="rotationMatrix">The computed rotation matrix (output).</param>
    /// <param name="rotationQuaternion">The computed rotation as a quaternion (output).</param>
    /// <param name="translation">The computed translation vector (output).</param>
    /// <param name="allowScaling">Indicates whether scaling is allowed in the alignment. Defaults to <c>false</c>.</param>
    /// <exception cref="ArgumentException">Thrown if the point sets have different sizes or are empty.</exception>

    public static void KabschUmeyamaAlign(
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

        // Compute covariance matrix (Math.NET type)
        MathNetMatrix covarianceMatrix = ComputeCovarianceMatrix(centeredSource, centeredTarget);

        // Perform Singular Value Decomposition (SVD)
        var svd = covarianceMatrix.Svd();
        MathNetMatrix U = svd.U;
        MathNetMatrix Vt = svd.VT;

        // Compute optimal rotation
        MathNetMatrix rotationMat = Vt.Transpose() * U.Transpose();

        // Ensure a proper rotation matrix (determinant should be +1)
        if (rotationMat.Determinant() < 0)
        {
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

        // Compute scale if scaling is enabled (Umeyama algorithm)
        scale = 1.0f;
        if (allowScaling)
        {
            scale = ComputeScale(centeredSource, centeredTarget, rotationMat);
        }

        // Compute scaled and rotated centroid source
        Vector4 transformedCentroidSource = Vector4.Transform(new Vector4(scale * centroidSource, 1.0f), rotationMatrix);
        translation = centroidTarget - new Vector3(transformedCentroidSource.X, transformedCentroidSource.Y, transformedCentroidSource.Z);
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

    private static float ComputeScale(List<Vector3> source, List<Vector3> target, MathNetMatrix rotationMat)
    {
        float num = 0;
        float den = 0;
        int count = source.Count;

        for (int i = 0; i < count; i++)
        {
            // Convert source to Math.NET Vector and apply rotation
            var sourceVec = VectorToMathNetMatrix(source[i]);
            var rotatedSourceVec = rotationMat * sourceVec;

            // Compute scaling factor numerator and denominator
            var targetVec = VectorToMathNetMatrix(target[i]);
            num += targetVec.PointwiseMultiply(rotatedSourceVec).Enumerate().Sum();
            den += rotatedSourceVec.PointwiseMultiply(rotatedSourceVec).Enumerate().Sum();
        }

        return num / den;
    }
}
