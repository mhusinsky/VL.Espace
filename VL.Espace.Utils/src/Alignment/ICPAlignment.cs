// C#
using System;
using System.Collections.Generic;
using System.Linq;
using Stride.Core.Mathematics;


namespace VL.Espace.Utils.Alignment
{


    /// <summary>
    /// Provides Iterative Closest Point (ICP) alignment functionality for 3D point clouds.
    /// ICP can handle point clouds of different sizes by iteratively finding correspondences
    /// and computing optimal transformations.
    /// </summary>
    public static class ICPAlignment
    {
        /// <summary>
        /// Aligns sourcePoints to targetPoints using the ICP algorithm.
        /// </summary>
        /// <param name="sourcePoints">Points to be transformed.</param>
        /// <param name="targetPoints">Reference points.</param>
        /// <param name="maxIterations">Maximum ICP iterations.</param>
        /// <param name="tolerance">Convergence tolerance (RMS error).</param>
        /// <param name="allowScaling">Allow scaling in alignment.</param>
        /// <returns>Tuple: (scale, rotationMatrix, rotationQuaternion, translation, rmsError, success)</returns>
        public static (float scale, Matrix rotationMatrix, Quaternion rotationQuaternion, Vector3 translation, float rmsError, bool success)
            ICP(List<Vector3> sourcePoints, List<Vector3> targetPoints, int maxIterations = 50, float tolerance = 1e-5f, bool allowScaling = false)
        {
            if (sourcePoints == null || targetPoints == null || sourcePoints.Count < 3 || targetPoints.Count < 3)
                return (1f, Matrix.Identity, Quaternion.Identity, Vector3.Zero, float.MaxValue, false);

            // Initialize cumulative transformation
            float cumulativeScale = 1f;
            Matrix cumulativeRotation = Matrix.Identity;
            Vector3 cumulativeTranslation = Vector3.Zero;

            // Work with copies to avoid modifying the original points
            var transformedSource = new List<Vector3>(sourcePoints);

            float prevError = float.MaxValue;

            for (int iter = 0; iter < maxIterations; iter++)
            {
                // 1. Find closest target for each transformed source point
                var correspondences = new List<(Vector3 src, Vector3 tgt)>();
                foreach (var src in transformedSource)
                {
                    var tgt = targetPoints.OrderBy(t => Vector3.Distance(src, t)).First();
                    correspondences.Add((src, tgt));
                }

                // 2. Extract paired lists
                var srcList = correspondences.Select(p => p.src).ToList();
                var tgtList = correspondences.Select(p => p.tgt).ToList();

                // 3. Compute alignment for this iteration
                KabschUmeyama.KabschUmeyamaAlign(srcList, tgtList, out float iterScale, out Matrix iterRotation, out Quaternion iterRotationQ, out Vector3 iterTranslation, allowScaling);

                // 4. Apply transformation to all source points
                for (int i = 0; i < transformedSource.Count; i++)
                {
                    Vector4 v = Vector4.Transform(new Vector4(iterScale * transformedSource[i], 1.0f), iterRotation);
                    transformedSource[i] = new Vector3(v.X, v.Y, v.Z) + iterTranslation;
                }

                // 5. Update cumulative transformation
                // T_total = T_cumulative * T_iter (apply iter transformation to the already accumulated result)
                cumulativeScale *= iterScale;
                cumulativeRotation = cumulativeRotation * iterRotation;
                Vector4 transformedCumTranslation = Vector4.Transform(new Vector4(cumulativeTranslation, 1.0f), iterRotation);
                cumulativeTranslation = iterScale * new Vector3(transformedCumTranslation.X, transformedCumTranslation.Y, transformedCumTranslation.Z) + iterTranslation;

                // 6. Compute RMS error
                float error = 0f;
                for (int i = 0; i < correspondences.Count; i++)
                {
                    float dist = Vector3.Distance(transformedSource[i], correspondences[i].tgt);
                    error += dist * dist;
                }
                error = (float)Math.Sqrt(error / correspondences.Count);

                // 7. Check convergence
                if (Math.Abs(prevError - error) < tolerance)
                {
                    var cumulativeRotationQ = Quaternion.RotationMatrix(cumulativeRotation);
                    return (cumulativeScale, cumulativeRotation, cumulativeRotationQ, cumulativeTranslation, error, true);
                }

                prevError = error;
            }

            var finalRotationQ = Quaternion.RotationMatrix(cumulativeRotation);
            return (cumulativeScale, cumulativeRotation, finalRotationQ, cumulativeTranslation, prevError, true);
        }
    }
}