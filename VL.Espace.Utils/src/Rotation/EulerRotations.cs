/* ============================================================================
   QuaternionToEuler (Process Node)
   Converts a Stride/VL quaternion into Euler angles in three simultaneous
   representations: Radians, Degrees, and Cycles.

   Copyright (c) 2025 Matthias Husinsky, GPT-5

   WHAT IT DOES
   1. Takes an input Quaternion (auto‑normalizes if slightly off unit length).
   2. Decomposes it into intrinsic Tait‑Bryan Euler angles for the selected Order.
   3. Provides the angles as:
      - Radians (optional pin): raw signed angles (can be negative, may exceed ±π).
      - Degrees (optional pin): normalized to [0, 360).
      - Cycles (always visible): normalized to [0, 1), where 1 cycle = 2π radians.

   ORDER SEMANTICS (Intuitive / User-Friendly)
   Order ABC means: rotate about local A, then local B, then local C.
   If you built your quaternion as:
       q = qA(angleA) * qB(angleB) * qC(angleC)
   (i.e. you multiplied in the same left→right order you conceptually applied),
   selecting Order = ABC will return (angleA, angleB, angleC) in the corresponding
   axis components (X,Y,Z) of the output vectors.

   EXAMPLE
   // You want: rotate 30° about Z, then 90° about X, then 45° about Y.
   Quaternion q =
       Quaternion.RotationZ(30°) *
       Quaternion.RotationX(90°) *
       Quaternion.RotationY(45°);
   Set:
       Input = q
       Order = ZXY
   Result (Radians ≈):
       X ≈ 1.570796 (≈90°)
       Y ≈ 0.785398 (≈45°)
       Z ≈ 0.523599 (≈30°)
   Degrees:
       (90, 45, 30)
   Cycles:
       (0.25, 0.125, 0.08333…)

   OUTPUT RANGES
   - Radians: Unwrapped; use if you need continuity or to perform your own wrapping.
   - Degrees: Wrapped to [0, 360).
   - Cycles: Wrapped to [0, 1).
   All three correspond to the SAME underlying orientation; choose what fits your patch.

   OPTIONAL PINS
   Radians and Degrees are marked Optional so you can hide them to keep patches clean.
   Cycles stays always visible as the compact canonical output.

   CACHING / PERFORMANCE
   The node caches the last quaternion + order:
   - Recomputes only if Quaternion or Order changed.
   - Lightweight enough for per‑frame use.

   GIMBAL LOCK
   Standard intrinsic Euler decomposition is susceptible to gimbal lock
   (singularities when the middle angle approaches ±90° depending on order).
   In such cases one degree of freedom is lost and one angle is set to 0 by
   the resolver to keep a stable result. If you need singularity‑free interpolation
   or blending, operate directly on quaternions instead.

   WHEN TO USE WHICH OUTPUT
   - Radians: Further math, trigonometry, filtering.
   - Degrees: UI display or systems expecting degrees.
   - Cycles: Normalized modulation (e.g. feeding LFOs, phase sliders).

   RIGHT‑HANDED NOTE
   Assumes the standard right‑handed axis system (Stride/VL default). Positive
   angles follow the right‑hand rule about +X, +Y, +Z.

   LIMITATIONS / ASSUMPTIONS
   - Only proper Tait‑Bryan orders (no repeated axes).
   - No left‑handed or extrinsic variant built in (can be added if needed).
   - Quaternion sign ambiguity (q and -q represent same rotation) is inherently handled.

   QUICK TROUBLESHOOTING
   - Angles appear under a different order than expected:
     Ensure you composed the quaternion in the same A→B→C order you selected.
   - Sudden angle flips near ±180° or ±90°:
     That’s normal Euler wrapping; switch to Radians for custom smoothing or use quaternions.

   EXTENDING
   - Add a ComposeIntrinsic helper if you want round‑trip tests inside patches.
   - Add alternative conventions (extrinsic, left‑handed) by introducing a mode flag
     and adjusting the internal order mapping.

   TAGS
   quaternion, euler, rotation, angles, convert, cycles, radians, degrees

   ============================================================================ */

#nullable enable
using System;
using System.ComponentModel;
using Stride.Core.Mathematics;
using VL.Core.Import;
using VL.Model;

namespace VL.Espace.Utils.Transform 
{
    public enum EulerOrder
    {
        XYZ,
        XZY,
        YXZ,
        YZX,
        ZXY,
        ZYX
    }

    [ProcessNode(
        Name = "QuaternionToEuler",
        Category = "3D.Transform",
        Summary = "Decomposes a quaternion into intrinsic Tait-Bryan Euler angles",
        Remarks = "Select order ABC if you built q = qA*qB*qC (A then B then C). Example: q = qZ*qX*qY -> Order=ZXY.",
        Tags = "quaternion,euler,rotation,angles")]
    public sealed class QuaternionToEuler
    {
        // Inputs
        public Quaternion Input
        {
            get => _input;
            set => _input = value;
        }

        public EulerOrder Order
        {
            get => _order;
            set => _order = value;
        }

        // Outputs
        [Pin(Visibility = PinVisibility.Optional)]
        public Vector3 Radians => _radians;

        [Pin(Visibility = PinVisibility.Optional)]
        public Vector3 Degrees => _degrees;

        public Vector3 Cycles => _cycles;

        // Internal state
        Quaternion _input;
        EulerOrder _order = EulerOrder.XYZ;

        Vector3 _radians;
        Vector3 _degrees;
        Vector3 _cycles;

        Quaternion _lastInput;
        EulerOrder _lastOrder;
        Vector3 _lastEulerRadians;
        bool _hasCached;

        const float TwoPi = MathF.PI * 2f;
        const float Rad2Deg = 180f / MathF.PI;
        const float QuaternionCompareEpsilon = 1e-9f;

        public void Update()
        {
            bool changed = !_hasCached || _order != _lastOrder || !ApproximatelyEqual(_input, _lastInput);
            if (!changed)
                return;

            var q = _input;
            float lenSq = q.LengthSquared();
            if (Math.Abs(1f - lenSq) > 1e-5f && lenSq > float.Epsilon)
                q = Quaternion.Normalize(q);

            // Reverse internally so that external (intuitive) order ABC matches qA*qB*qC.
            var internalOrder = ReverseOrder(_order);
            _lastEulerRadians = ComputeEuler(q, internalOrder);

            _lastInput = _input;
            _lastOrder = _order;
            _hasCached = true;

            _radians = _lastEulerRadians;
            _degrees = new Vector3(
                NormalizeDegrees(_radians.X * Rad2Deg),
                NormalizeDegrees(_radians.Y * Rad2Deg),
                NormalizeDegrees(_radians.Z * Rad2Deg)
            );
            _cycles = new Vector3(
                NormalizeCycle(_radians.X / TwoPi),
                NormalizeCycle(_radians.Y / TwoPi),
                NormalizeCycle(_radians.Z / TwoPi)
            );
        }

        // Map intuitive ABC to internal CBA
        static EulerOrder ReverseOrder(EulerOrder o) => o switch
        {
            EulerOrder.XYZ => EulerOrder.ZYX,
            EulerOrder.XZY => EulerOrder.YZX,
            EulerOrder.YXZ => EulerOrder.ZXY,
            EulerOrder.YZX => EulerOrder.XZY,
            EulerOrder.ZXY => EulerOrder.YXZ,
            EulerOrder.ZYX => EulerOrder.XYZ,
            _ => o
        };

        // Helpers
        static float NormalizeCycle(float v)
        {
            v %= 1f;
            if (v < 0f) v += 1f;
            return v;
        }

        static float NormalizeDegrees(float d)
        {
            d %= 360f;
            if (d < 0f) d += 360f;
            return d;
        }

        static bool ApproximatelyEqual(in Quaternion a, in Quaternion b) =>
            Math.Abs(a.X - b.X) <= QuaternionCompareEpsilon &&
            Math.Abs(a.Y - b.Y) <= QuaternionCompareEpsilon &&
            Math.Abs(a.Z - b.Z) <= QuaternionCompareEpsilon &&
            Math.Abs(a.W - b.W) <= QuaternionCompareEpsilon;

        // Decomposition (standard formulas)
        static Vector3 ComputeEuler(in Quaternion q, EulerOrder order)
        {
            float x = q.X, y = q.Y, z = q.Z, w = q.W;

            float xx = x * x;
            float yy = y * y;
            float zz = z * z;
            float xy = x * y;
            float xz = x * z;
            float yz = y * z;
            float wx = w * x;
            float wy = w * y;
            float wz = w * z;

            float m11 = 1f - 2f * (yy + zz);
            float m12 = 2f * (xy - wz);
            float m13 = 2f * (xz + wy);

            float m21 = 2f * (xy + wz);
            float m22 = 1f - 2f * (xx + zz);
            float m23 = 2f * (yz - wx);

            float m31 = 2f * (xz - wy);
            float m32 = 2f * (yz + wx);
            float m33 = 1f - 2f * (xx + yy);

            static float Clamp(float v) => v < -1f ? -1f : (v > 1f ? 1f : v);
            const float G = 0.9999999f;

            float ax, ay, az;

            switch (order)
            {
                case EulerOrder.XYZ:
                    ay = MathF.Asin(Clamp(m13));
                    if (Math.Abs(m13) < G)
                    {
                        ax = MathF.Atan2(-m23, m33);
                        az = MathF.Atan2(-m12, m11);
                    }
                    else
                    {
                        ax = MathF.Atan2(m32, m22);
                        az = 0f;
                    }
                    return new Vector3(ax, ay, az);

                case EulerOrder.YXZ:
                    ax = MathF.Asin(-Clamp(m23));
                    if (Math.Abs(m23) < G)
                    {
                        ay = MathF.Atan2(m13, m33);
                        az = MathF.Atan2(m21, m22);
                    }
                    else
                    {
                        ay = MathF.Atan2(-m31, m11);
                        az = 0f;
                    }
                    return new Vector3(ax, ay, az);

                case EulerOrder.ZXY:
                    ax = MathF.Asin(Clamp(m32));
                    if (Math.Abs(m32) < G)
                    {
                        ay = MathF.Atan2(-m31, m33);
                        az = MathF.Atan2(-m12, m22);
                    }
                    else
                    {
                        ay = 0f;
                        az = MathF.Atan2(m21, m11);
                    }
                    return new Vector3(ax, ay, az);

                case EulerOrder.ZYX:
                    ay = MathF.Asin(-Clamp(m31));
                    if (Math.Abs(m31) < G)
                    {
                        ax = MathF.Atan2(m32, m33);
                        az = MathF.Atan2(m21, m11);
                    }
                    else
                    {
                        ax = 0f;
                        az = MathF.Atan2(-m12, m22);
                    }
                    return new Vector3(ax, ay, az);

                case EulerOrder.YZX:
                    az = MathF.Asin(Clamp(m21));
                    if (Math.Abs(m21) < G)
                    {
                        ax = MathF.Atan2(-m23, m22);
                        ay = MathF.Atan2(-m31, m11);
                    }
                    else
                    {
                        ax = 0f;
                        ay = MathF.Atan2(m13, m33);
                    }
                    return new Vector3(ax, ay, az);

                case EulerOrder.XZY:
                    az = MathF.Asin(-Clamp(m12));
                    if (Math.Abs(m12) < G)
                    {
                        ax = MathF.Atan2(m32, m22);
                        ay = MathF.Atan2(m13, m11);
                    }
                    else
                    {
                        ax = MathF.Atan2(-m23, m33);
                        ay = 0f;
                    }
                    return new Vector3(ax, ay, az);

                default:
                    return Vector3.Zero;
            }
        }
    }

    [ProcessNode(
        Name = "EulerRotation",
        Category = "3D.Transform",
        Summary = "Takes angles in cycles (X,Y,Z) and an order ABC; applies intrinsic rotations about local A then B then C. Compose as q = qA*qB*qC and R = R_A*R_B*R_C (e.g., ZXY matches qZ*qX*qY and Rz*Rx*Ry). Returns the composed Quaternion and Rotation Matrix.",
        Tags = "rotation, cycles, quaternion, matrix, tait-bryan, order")]
    public sealed class EulerRotation
    {
        // -------- Inputs --------
        // Angles as cycles where 1.0 = 360° = 2π radians; values can be outside [0,1).
        public Vector3 Angles
        {
            get => _angles;
            set => _angles = value;
        }

        [DefaultValue(EulerOrder.ZXY)]
        public EulerOrder Order
        {
            get => _order;
            set => _order = value;
        }

        // -------- Outputs --------
        public Matrix Matrix => _matrix;
        public Quaternion Quaternion => _quaternion;

        // -------- Internal State --------
        Vector3 _angles;
        EulerOrder _order = EulerOrder.XZY; 

        Matrix _matrix;
        Quaternion _quaternion;

        Vector3 _lastCycles;
        EulerOrder _lastOrder;
        bool _hasCached;

        const float TwoPi = MathF.PI * 2f;

        // -------- Process --------
        public void Update()
        {
            bool changed = !_hasCached || _order != _lastOrder || _angles != _lastCycles;
            if (!changed)
                return;

            // Convert cycles to radians (do not wrap; allow any real input)
            var radians = _angles * TwoPi;

            // Elementary rotations
            var qx = Quaternion.RotationX(radians.X);
            var qy = Quaternion.RotationY(radians.Y);
            var qz = Quaternion.RotationZ(radians.Z);

            var Rx = Matrix.RotationX(radians.X);
            var Ry = Matrix.RotationY(radians.Y);
            var Rz = Matrix.RotationZ(radians.Z);

            // Compose according to intuitive left->right order ABC (intrinsic: local A, then B, then C)
            switch (_order)
            {
                case EulerOrder.XYZ:
                    _quaternion = qx * qy * qz;
                    _matrix = Rx * Ry * Rz;
                    break;

                case EulerOrder.XZY:
                    _quaternion = qx * qz * qy;
                    _matrix = Rx * Rz * Ry;
                    break;

                case EulerOrder.YXZ:
                    _quaternion = qy * qx * qz;
                    _matrix = Ry * Rx * Rz;
                    break;

                case EulerOrder.YZX:
                    _quaternion = qy * qz * qx;
                    _matrix = Ry * Rz * Rx;
                    break;

                case EulerOrder.ZXY:
                    _quaternion = qz * qx * qy;
                    _matrix = Rz * Rx * Ry;
                    break;

                case EulerOrder.ZYX:
                    _quaternion = qz * qy * qx;
                    _matrix = Rz * Ry * Rx;
                    break;

                default:
                    _quaternion = Quaternion.Identity;
                    _matrix = Matrix.Identity;
                    break;
            }

            // Normalize quaternion for numerical hygiene
            _quaternion = Stride.Core.Mathematics.Quaternion.Normalize(_quaternion);

            _lastCycles = _angles;
            _lastOrder = _order;
            _hasCached = true;
        }
    }
    
}