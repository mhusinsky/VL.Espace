// RANSAC-based Point Alignment implementation for use in VVVV Gamma
// by Matthias Husinsky and Cody (Sourcegraph AI Assistant)

using System;
using System.Collections.Generic;
using System.Linq;
using Stride.Core.Mathematics; // Stride3D's Math Library

namespace VL.Espace.Utils.Alignment
{
    public static class RansacAlignment
    {
        /// <summary>
        /// Represents the result of a RANSAC alignment operation.
        /// </summary>
        public struct RansacResult
        {
            public float Scale;
            public Matrix RotationMatrix;
            public Quaternion RotationQuaternion;
            public Vector3 Translation;
            public int InlierCount;
            public List<int> InlierIndices;
            public float InlierRatio;
            public float RmsError;
            public bool Success;
        }

        /// <summary>
        /// Aligns two sets of 3D points using RANSAC for robust estimation.
        /// This method can handle outliers and mismatched correspondences by finding
        /// the transformation that best fits the largest subset of point pairs.
        /// </summary>
        /// <param name="sourcePoints">The list of source points to align.</param>
        /// <param name="targetPoints">The list of target points to align to.</param>
        /// <param name="distanceThreshold">Maximum distance for a point to be considered an inlier.</param>
        /// <param name="maxIterations">Maximum number of RANSAC iterations.</param>
        /// <param name="minInliers">Minimum number of inliers required for a valid solution.</param>
        /// <param name="allowScaling">Whether to allow scaling in the transformation.</param>
        /// <param name="alignmentMethod">The underlying alignment method to use.</param>
        /// <param name="confidenceLevel">Desired confidence level (0.0 to 1.0) for finding a good model.</param>
        /// <param name="earlyTermination">Whether to terminate early when a good solution is found.</param>
        /// <returns>RansacResult containing the alignment parameters and statistics.</returns>
        /// <exception cref="ArgumentException">Thrown if the point sets have different sizes or are too small.</exception>
        public static RansacResult RansacAlign(
            List<Vector3> sourcePoints,
            List<Vector3> targetPoints,
            float distanceThreshold = 0.1f,
            int maxIterations = 1000,
            int minInliers = 4,
            bool allowScaling = false,
            AlignmentMethod alignmentMethod = AlignmentMethod.Kabsch,
            float confidenceLevel = 0.99f,
            bool earlyTermination = true)
        {
            if (sourcePoints.Count != targetPoints.Count)
                throw new ArgumentException("Point sets must have the same number of elements.");

            if (sourcePoints.Count < 3)
                throw new ArgumentException("Need at least 3 point pairs for 3D alignment.");

            if (minInliers < 3)
                minInliers = 3;

            int numPoints = sourcePoints.Count;
            var random = new Random();

            // Best model found so far
            var bestResult = new RansacResult
            {
                Success = false,
                InlierCount = 0,
                InlierIndices = new List<int>(),
                Scale = 1.0f,
                RotationMatrix = Matrix.Identity,
                RotationQuaternion = Quaternion.Identity,
                Translation = Vector3.Zero
            };

            // Adaptive iteration count based on outlier ratio
            int adaptiveMaxIterations = maxIterations;

            for (int iteration = 0; iteration < adaptiveMaxIterations; iteration++)
            {
                try
                {
                    // 1. Randomly sample minimum required points (3 for 3D alignment)
                    var sampleIndices = RandomSample(numPoints, 3, random);
                    var sampleSource = sampleIndices.Select(i => sourcePoints[i]).ToList();
                    var sampleTarget = sampleIndices.Select(i => targetPoints[i]).ToList();

                    // 2. Fit model to sample points
                    if (!FitModel(sampleSource, sampleTarget, alignmentMethod, allowScaling,
                        out float sampleScale, out Matrix sampleRotation, out Quaternion sampleQuaternion, out Vector3 sampleTranslation))
                    {
                        continue; // Skip this iteration if model fitting failed
                    }

                    // 3. Count inliers for this model
                    var inlierIndices = new List<int>();
                    float totalSquaredError = 0f;

                    for (int i = 0; i < numPoints; i++)
                    {
                        // Transform source point
                        Vector4 transformed = Vector4.Transform(new Vector4(sampleScale * sourcePoints[i], 1.0f), sampleRotation);
                        Vector3 alignedSource = new Vector3(transformed.X, transformed.Y, transformed.Z) + sampleTranslation;

                        // Check if it's an inlier
                        float distance = Vector3.Distance(alignedSource, targetPoints[i]);
                        if (distance <= distanceThreshold)
                        {
                            inlierIndices.Add(i);
                            totalSquaredError += distance * distance;
                        }
                    }

                    // 4. Update best model if this one is better
                    if (inlierIndices.Count > bestResult.InlierCount && inlierIndices.Count >= minInliers)
                    {
                        bestResult.Scale = sampleScale;
                        bestResult.RotationMatrix = sampleRotation;
                        bestResult.RotationQuaternion = sampleQuaternion;
                        bestResult.Translation = sampleTranslation;
                        bestResult.InlierCount = inlierIndices.Count;
                        bestResult.InlierIndices = new List<int>(inlierIndices);
                        bestResult.InlierRatio = (float)inlierIndices.Count / numPoints;
                        bestResult.RmsError = inlierIndices.Count > 0 ? (float)Math.Sqrt(totalSquaredError / inlierIndices.Count) : float.MaxValue;
                        bestResult.Success = true;

                        // Update adaptive iteration count based on current inlier ratio
                        if (earlyTermination)
                        {
                            adaptiveMaxIterations = Math.Min(maxIterations,
                                ComputeAdaptiveIterations(bestResult.InlierRatio, confidenceLevel, 3));
                        }
                    }

                    // Early termination if we have a very good solution
                    if (earlyTermination && bestResult.InlierRatio > 0.8f && iteration > 50)
                    {
                        break;
                    }
                }
                catch
                {
                    // Skip this iteration if any error occurs
                    continue;
                }
            }

            // 5. Refine the best model using all inliers (optional but recommended)
            if (bestResult.Success && bestResult.InlierCount >= minInliers)
            {
                var inlierSource = bestResult.InlierIndices.Select(i => sourcePoints[i]).ToList();
                var inlierTarget = bestResult.InlierIndices.Select(i => targetPoints[i]).ToList();

                if (FitModel(inlierSource, inlierTarget, alignmentMethod, allowScaling,
                    out float refinedScale, out Matrix refinedRotation, out Quaternion refinedQuaternion, out Vector3 refinedTranslation))
                {
                    bestResult.Scale = refinedScale;
                    bestResult.RotationMatrix = refinedRotation;
                    bestResult.RotationQuaternion = refinedQuaternion;
                    bestResult.Translation = refinedTranslation;

                    // Recompute RMS error with refined model
                    float totalSquaredError = 0f;
                    foreach (int idx in bestResult.InlierIndices)
                    {
                        Vector4 transformed = Vector4.Transform(new Vector4(refinedScale * sourcePoints[idx], 1.0f), refinedRotation);
                        Vector3 alignedSource = new Vector3(transformed.X, transformed.Y, transformed.Z) + refinedTranslation;
                        float distance = Vector3.Distance(alignedSource, targetPoints[idx]);
                        totalSquaredError += distance * distance;
                    }
                    bestResult.RmsError = (float)Math.Sqrt(totalSquaredError / bestResult.InlierCount);
                }
            }

            return bestResult;
        }

        /// <summary>
        /// Enumeration of available alignment methods for RANSAC.
        /// </summary>
        public enum AlignmentMethod
        {
            Kabsch,
            Procrustes,
            Horn
        }

        /// <summary>
        /// Simplified RANSAC alignment that returns the same output format as other alignment methods.
        /// </summary>
        public static bool RansacAlignSimple(
            List<Vector3> sourcePoints,
            List<Vector3> targetPoints,
            out float scale,
            out Matrix rotationMatrix,
            out Quaternion rotationQuaternion,
            out Vector3 translation,
            float distanceThreshold = 0.1f,
            int maxIterations = 1000,
            bool allowScaling = false,
            AlignmentMethod alignmentMethod = AlignmentMethod.Kabsch)
        {
            var result = RansacAlign(sourcePoints, targetPoints, distanceThreshold, maxIterations,
                4, allowScaling, alignmentMethod);

            scale = result.Scale;
            rotationMatrix = result.RotationMatrix;
            rotationQuaternion = result.RotationQuaternion;
            translation = result.Translation;

            return result.Success;
        }

        private static List<int> RandomSample(int totalCount, int sampleSize, Random random)
        {
            var indices = new List<int>();
            var available = Enumerable.Range(0, totalCount).ToList();

            for (int i = 0; i < sampleSize; i++)
            {
                int randomIndex = random.Next(available.Count);
                indices.Add(available[randomIndex]);
                available.RemoveAt(randomIndex);
            }

            return indices;
        }

        private static bool FitModel(
            List<Vector3> sourcePoints,
            List<Vector3> targetPoints,
            AlignmentMethod method,
            bool allowScaling,
            out float scale,
            out Matrix rotationMatrix,
            out Quaternion rotationQuaternion,
            out Vector3 translation)
        {
            scale = 1.0f;
            rotationMatrix = Matrix.Identity;
            rotationQuaternion = Quaternion.Identity;
            translation = Vector3.Zero;

            try
            {
                switch (method)
                {
                    case AlignmentMethod.Kabsch:
                        KabschUmeyama.KabschUmeyamaAlign(sourcePoints, targetPoints,
                            out scale, out rotationMatrix, out rotationQuaternion, out translation, allowScaling);
                        return true;

                    case AlignmentMethod.Procrustes:
                        ProcrustesAnalysis.ProcrustesAlign(sourcePoints, targetPoints,
                            out scale, out rotationMatrix, out rotationQuaternion, out translation, allowScaling);
                        return true;

                    case AlignmentMethod.Horn:
                        HornsMethod.HornsAlign(sourcePoints, targetPoints,
                            out scale, out rotationMatrix, out rotationQuaternion, out translation, allowScaling);
                        return true;

                    default:
                        return false;
                }
            }
            catch
            {
                return false;
            }
        }

        private static int ComputeAdaptiveIterations(float inlierRatio, float confidence, int sampleSize)
        {
            if (inlierRatio <= 0.0f || inlierRatio >= 1.0f)
                return int.MaxValue;

            // Formula: log(1-confidence) / log(1-inlierRatio^sampleSize)
            double outlierRatio = 1.0 - inlierRatio;
            double denominator = Math.Log(1.0 - Math.Pow(inlierRatio, sampleSize));

            if (Math.Abs(denominator) < 1e-10)
                return 1;

            int iterations = (int)Math.Ceiling(Math.Log(1.0 - confidence) / denominator);
            return Math.Max(1, Math.Min(iterations, 100000)); // Reasonable bounds
        }
    }
}

