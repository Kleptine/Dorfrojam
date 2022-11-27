using UnityEngine;
using UnityEngine.Assertions;

namespace Utilities.Unity
{
    public static class UnityMath
    {
        /// <summary>Determine the signed angle between two vectors, with normal 'n' as the rotation axis.</summary>
        public static float AngleSigned(Vector3 v1, Vector3 v2, Vector3 n)
        {
            return Mathf.Atan2(Vector3.Dot(n, Vector3.Cross(v1, v2)), Vector3.Dot(v1, v2)) * Mathf.Rad2Deg;
        }

        /// <summary>Returns true if all of the components of the two vectors are within tolerance.</summary>
        public static bool AreEqualWithinTolerance(Vector3 a, Vector3 b, float tolerance)
        {
            if (Mathf.Abs(a.x - b.x) < tolerance
                && Mathf.Abs(a.y - b.y) < tolerance
                && Mathf.Abs(a.z - b.z) < tolerance)
            {
                return true;
            }

            return false;
        }

        /// <summary>
        ///     Transforms a vector from the Houdini (Y-up, Z forward, X-flipped) coordinate space to the Unity coordinate
        ///     space.
        /// </summary>
        public static Vector3 TransformFromHoudini(Vector3 v1)
        {
            return new Vector3(-v1.x, v1.y, v1.z);
        }

        /// <summary>Returns the forward direction of this quaternion orientation.</summary>
        public static Vector3 Forward(this Quaternion q)
        {
            return q * Vector3.forward;
        }

        /// <summary>Rotates a vector around a pivot position.</summary>
        public static Vector3 RotateAround(this Vector3 position, Vector3 pivot, Vector3 rotationAxis,
                                           float rotationAngleDeg)
        {
            Vector3 diff = position - pivot;
            Quaternion rotation = Quaternion.AngleAxis(rotationAngleDeg, rotationAxis);
            diff = rotation * diff;
            return pivot + diff;
        }

        /// <summary>Checks if this quaternion is either the positive or negative identity quaternion.</summary>
        public static bool IsIdentity(this Quaternion q)
        {
            return q == Quaternion.identity || q == new Quaternion(0, 0, 0, -1);
        }

        /// <summary>
        ///     Computes the quaternion that takes source to destination. For example:
        ///     <code> dest.Difference(source) * source == dest </code>
        /// </summary>
        public static Quaternion Difference(this Quaternion destination, Quaternion source)
        {
            return destination * Quaternion.Inverse(source);
        }

        /// <summary>The magnitude of a quaternion.</summary>
        public static float Magnitude(this Quaternion quat)
        {
            return Mathf.Sqrt(quat.SquareMagnitude());
        }

        /// <summary>The square magnitude of a quaternion.</summary>
        public static float SquareMagnitude(this Quaternion quat)
        {
            return quat.x * quat.x + quat.y * quat.y + quat.z * quat.z + quat.w * quat.w;
        }

        /// <summary>
        ///     Returns the Swing/Twist decomposition of a Quaternion. Useful for smooth lerping. See google for more
        ///     information.
        /// </summary>
        /// <param name="twistAxis">The starting forward direction that the rotation q is being applied to.</param>
        /// <param name="q">A rotation to decompose</param>
        /// <param name="swing">The swing component of q.</param>
        /// <param name="twist">The twist component of q.</param>
        public static void SwingTwistDecomposition(this Quaternion q, Vector3 twistAxis, out Quaternion swing,
                                                   out Quaternion twist)
        {
            // Vector part projected onto twist axis
            Vector3 projection = Vector3.Dot(twistAxis, new Vector3(q.x, q.y, q.z)) * twistAxis;

            // Twist quaternion
            twist = new Quaternion(projection.x, projection.y, projection.z, q.w);

            // Singularity close to 180deg
            // ReSharper disable once CompareOfFloatsByEqualityOperator
            if (twist.SquareMagnitude() == 0.0f)
            {
                twist = Quaternion.identity;
            }
            else
            {
                twist = Quaternion.Normalize(twist);
            }

            // Set swing
            swing = q * Quaternion.Inverse(twist);
        }

        /// <summary>Calculates the distance from this point to a given ray. </summary>
        /// <remarks>Uses the technique from this video: https://youtu.be/tYUtWYGUqgw </remarks>
        public static float DistanceToRay(this Vector3 point, Ray ray)
        {
            return Vector3.Cross(point - ray.origin, ray.direction).magnitude;
        }

        /// <summary>Returns the distance between the two points, along the ground (XZ) plane.</summary>
        public static float DistanceAlongFloor(this Vector3 point, Vector3 secondPoint)
        {
            return Vector3.Distance(point.Zero(y: true), secondPoint.Zero(y: true));
        }

        /// <summary>Zeroes out specific components on a vector.</summary>
        /// <remarks>Ideally this will get inlined and evaluated at compile time, most of the time.</remarks>
        public static Vector3 Zero(this Vector3 vector, bool x = false, bool y = false, bool z = false)
        {
            if (x)
            {
                vector.x = 0;
            }

            if (y)
            {
                vector.y = 0;
            }

            if (z)
            {
                vector.z = 0;
            }

            return vector;
        }

        /// <summary>Return the vector projected onto the XZ plane (i.e. set the Y component to zero)</summary>
        /// <param name="vector"></param>
        /// <returns>The projected vector</returns>
        public static Vector3 ToXZ(this Vector3 vector)
        {
            return new Vector3(vector.x, 0, vector.z);
        }

        /// <summary>
        ///     Like the <see cref="Transform.LookAt(UnityEngine.Transform,UnityEngine.Vector3)" /> function, but on rotates
        ///     the object on the ground (XZ plane).
        /// </summary>
        public static void LookAtXZ(this Transform transform, Vector3 worldPosition)
        {
            worldPosition.y = transform.position.y;
            transform.LookAt(worldPosition);
        }

        /// <summary>Copies certain components from one vector to another, and returns the result.</summary>
        /// <remarks>Note that you have to use the returned value.</remarks>
        public static Vector3 Copy(this Vector3 vector, Vector3 target, bool x = false, bool y = false, bool z = false)
        {
            if (x)
            {
                target.x = vector.x;
            }

            if (y)
            {
                target.y = vector.y;
            }

            if (z)
            {
                target.z = vector.z;
            }

            return target;
        }

        /// <summary>
        ///     Calculates a vector offset of this point from the given ray. Subtracting this value from the point will place
        ///     it on the ray. This vector represents the shortest distance from the ray to the point.
        /// </summary>
        public static Vector3 OffsetFromRay(this Vector3 point, Ray ray)
        {
            Vector3 relativePoint = point - ray.origin; // A vector from the ray origin to the point
            Vector3 projection =
                Vector3.Project(relativePoint, ray.direction); // Vector from the ray origin to the projected point.
            Vector3
                orthogonalProjection =
                    relativePoint - projection; // A vector from the projected point to the original point.
            return orthogonalProjection; // Invert the vector to give the offset
        }

        /// <summary>
        ///     Transforms a local-space position to world-space using this transform. Ignores any scale component of the
        ///     transform.
        /// </summary>
        /// <remarks>
        ///     Suitable for transforming to the Rigidbody coordinate space because that space is unscaled, but matches the
        ///     rotation and translation of the GameObject's transform.
        /// </remarks>
        public static Vector3 TransformPointUnscaled(this Transform transform, Vector3 position)
        {
            Matrix4x4 localToWorldMatrix = Matrix4x4.TRS(transform.position, transform.rotation, Vector3.one);
            return localToWorldMatrix.MultiplyPoint3x4(position);
        }

        /// <summary>Transforms a world-space position to local-space using this transform.</summary>
        /// <remarks>
        ///     Suitable for transforming from the Rigidbody coordinate space because that space is unscaled, but matches the
        ///     rotation and translation of the GameObject's transform.
        /// </remarks>
        public static Vector3 InverseTransformPointUnscaled(this Transform transform, Vector3 position)
        {
            Matrix4x4 worldToLocalMatrix = Matrix4x4.TRS(transform.position, transform.rotation, Vector3.one).inverse;
            return worldToLocalMatrix.MultiplyPoint3x4(position);
        }

        /// <summary>Transforms a local-space vector to world-space using this transform.</summary>
        /// <remarks>
        ///     Suitable for transforming to the Rigidbody coordinate space because that space is unscaled, but matches the
        ///     rotation and translation of the GameObject's transform.
        /// </remarks>
        public static Vector3 TransformVectorUnscaled(this Transform transform, Vector3 vector)
        {
            Matrix4x4 localToWorldMatrix = Matrix4x4.TRS(transform.position, transform.rotation, Vector3.one);
            return localToWorldMatrix.MultiplyVector(vector);
        }

        /// <summary>Transforms a local-space vector to world-space using this transform.</summary>
        /// <remarks>
        ///     Suitable for transforming from the Rigidbody coordinate space because that space is unscaled, but matches the
        ///     rotation and translation of the GameObject's transform.
        /// </remarks>
        public static Vector3 InverseTransformVectorUnscaled(this Transform transform, Vector3 vector)
        {
            Matrix4x4 worldToLocalMatrix = Matrix4x4.TRS(transform.position, transform.rotation, Vector3.one).inverse;
            return worldToLocalMatrix.MultiplyVector(vector);
        }

        /// <summary>
        ///     Intends to be the same as <see cref="Rigidbody.AddForce(Vector3,ForceMode)" />, except that the angular
        ///     acceleration caused by the force may be limited.
        ///     <para>The final, applied angular acceleration vector is magnitude-clamped. </para>
        /// </summary>
        /// <param name="body">The body to apply the force to.</param>
        /// <param name="force">The world space force to apply.</param>
        /// <param name="position">The world space position to apply the force.</param>
        /// <param name="maxAngularAcceleration">The maximum magnitude of the angular acceleration applied.</param>
        /// <param name="forceMode">
        ///     May be Force or Acceleration. In acceleration mode, we just multiply the mass into the force,
        ///     like Unity does.
        /// </param>
        /// <remarks>
        ///     This function is mathematically correct as far as I am aware. Note, though, that when using it in a spring
        ///     system, because we limit the angular acceleration, this can cause the spring to convert much more slowly (or
        ///     potentially spiral outwards). Capping angular acceleration can be helpful, but isn't a physical action. Because of
        ///     this, sometimes you can have strange behaviors.
        /// </remarks>
        public static void AddForceWithLimits(this Rigidbody body, Vector3 force, Vector3 position,
                                              float maxAngularAcceleration, ForceMode forceMode = ForceMode.Force)
        {
            Assert.AreNotEqual(forceMode, ForceMode.VelocityChange);
            Assert.AreNotEqual(forceMode, ForceMode.Impulse);

            if (forceMode == ForceMode.Acceleration)
            {
                // We've passed in an acceleration, not a force, so multiply it by the mass to get
                // the linear force we need to apply. (F = m * a)

                // Note: it might seem like we should do something similar for the torque
                // (ie. multiplying the torque by the moment of inertia). In practice, this
                // doesn't work, and I believe it's because there's no clear formula for deriving
                // an angular acceleration from an acceleration at a point. It's more stable to just 
                // use this (mass including) value to calculate the torque. I believe this is what 
                // Unity does under the hood in Acceleration force mode.
                force *= body.mass;
            }

            // A force at a point can be decomposed into two components: the same force at the center of mass, and a torque.
            // We break the force into its component pieces, so we can limit the angular acceleration (torque):

            // Mass-local space is the the coordinate system centered at the center-of-mass of the rigid body
            // and oriented such that the XYZ unit vectors are aligned with the 3 primary axis of inertia.
            // We use this space for torque calculation because that's the coordinate space of the (diagonal) inertia tensor.
            Vector3 massLocalPosition = body.transform.InverseTransformPointUnscaled(position) - body.centerOfMass;

            // Note: this calculation requires that the transform is synced with the current transformation of the RigidBody
            // This is not *always* the case, but usually they are almost precisely the same
            Vector3 localForce = body.transform.InverseTransformVectorUnscaled(force);

            Vector3 massLocalForce = Quaternion.Inverse(body.inertiaTensorRotation) * localForce;
            Vector3 massLocalTorque = Vector3.Cross(massLocalPosition, massLocalForce);

            // The angular acceleration caused by this force, in mass-local space.
            Vector3 massLocalAngularAccel = new Vector3(
                massLocalTorque.x / body.inertiaTensor.x,
                massLocalTorque.y / body.inertiaTensor.y,
                massLocalTorque.z / body.inertiaTensor.z
            );

            Vector3 clampedMassLocalAngularAccel =
                Vector3.ClampMagnitude(massLocalAngularAccel, maxAngularAcceleration);

            // Transform the angular acceleration back to local space so we can apply it.
            Vector3 clampedMassLocalTorque = new Vector3(
                clampedMassLocalAngularAccel.x * body.inertiaTensor.x,
                clampedMassLocalAngularAccel.y * body.inertiaTensor.y,
                clampedMassLocalAngularAccel.z * body.inertiaTensor.z
            );

            Vector3 clampedLocalTorque = body.inertiaTensorRotation * clampedMassLocalTorque;

            // Apply the local, now clamped, torque to the rigidbody.
            body.AddRelativeTorque(clampedLocalTorque, ForceMode.Force);

            // Apply the local force to the center of mass.
            Vector3 linearForce = localForce;
            body.AddRelativeForce(linearForce, ForceMode.Force);
        }

        /// <summary>
        ///     A variation on "GetPointVelocity" that returns the velocity of a point, after a force has been applied. Note
        ///     that the position at which this function calculates the velocity is integrated forward. This returns the velocity
        ///     of this point after integration if no other forces were applied.
        /// </summary>
        /// <remarks>Additionally returns the (speculated) updated position of the point.</remarks>
        /// <param name="body">Body to access</param>
        /// <param name="force">The speculative world space force that would be applied.</param>
        /// <param name="forcePosition">The world space location of the force applied.</param>
        /// <param name="velReadPosition">The world space point to read the velocity of.</param>
        /// <param name="finalReadPos">The output location of the velReadPosition, after it has been integrated one timestep.</param>
        public static Vector3 GetPointVelocityAfterForce(this Rigidbody body, Vector3 force, Vector3 forcePosition,
                                                         Vector3 velReadPosition, out Vector3 finalReadPos)
        {
            Vector3 deltaLinearVelocity = force / body.mass * Time.deltaTime;

            Vector3 massLocalPosition = body.transform.InverseTransformPointUnscaled(forcePosition) - body.centerOfMass;

            Vector3 localForce = body.transform.InverseTransformVectorUnscaled(force);

            Vector3 massLocalForce = Quaternion.Inverse(body.inertiaTensorRotation) * localForce;
            Vector3 massLocalTorque = Vector3.Cross(massLocalPosition, massLocalForce);

            // The angular acceleration caused by this force, in mass-local space.
            Vector3 massLocalAngularAccel = new Vector3(
                massLocalTorque.x / body.inertiaTensor.x,
                massLocalTorque.y / body.inertiaTensor.y,
                massLocalTorque.z / body.inertiaTensor.z
            );

            // In PhysX, a zero tensor value on an axis implies that the object has infinite mass in that axis.
            // Usually this means the "Fix Rotation" box was checked for the axis.
            // ReSharper disable CompareOfFloatsByEqualityOperator
            if (body.inertiaTensor.x == 0.0)
            {
                massLocalAngularAccel.x = 0;
            }

            if (body.inertiaTensor.y == 0.0)
            {
                massLocalAngularAccel.y = 0;
            }

            if (body.inertiaTensor.z == 0.0)
            {
                massLocalAngularAccel.z = 0;
            }
            // ReSharper enable CompareOfFloatsByEqualityOperator

            Vector3 localAngularAccel = body.inertiaTensorRotation * massLocalAngularAccel;
            Vector3 deltaAngularVel = body.transform.TransformVectorUnscaled(localAngularAccel) * Time.deltaTime;

            Vector3 finalLinearVel = body.velocity + deltaLinearVelocity;
            Vector3 finalAngularVel = body.angularVelocity + deltaAngularVel;

            // TODO(john): This is not perfectly accurate, and should instead use the angular update
            // approximation noted in this article. However, it is quite close, and should only be revisited if we are
            // having issues with stability.
            // https://johnaustin.io/articles/2020/when-an-angular-velocity-isnt-an-angular-velocity
            Vector3 advectedPointPos = AdvectPointByVelocities(
                finalLinearVel, finalAngularVel, body.worldCenterOfMass,
                velReadPosition);

            Vector3 advectedRadialToPoint = advectedPointPos - body.worldCenterOfMass;
            Vector3 velocityAtPoint = finalLinearVel + Vector3.Cross(finalAngularVel, advectedRadialToPoint);
            finalReadPos = advectedPointPos;

            return velocityAtPoint;
        }

        public static Vector3 GetPointVelocity(Vector3 bodyCenterOfMass, Vector3 bodyVelocity,
                                               Vector3 bodyAngularVelocity,
                                               Vector3 pointPosition)
        {
            Vector3 radialToPoint = pointPosition - bodyCenterOfMass;

            return bodyVelocity + Vector3.Cross(bodyAngularVelocity, radialToPoint);
        }

        /// <summary>Advects a point on a rigid body based on one time step of the given motion.</summary>
        /// <param name="velocity">The velocity of the rigid body.</param>
        /// <param name="angularVelocity">The Angular Velocity of the rigid body.</param>
        /// <param name="bodyCenterOfMassWorldSpace">The center of mass of the body in world space.</param>
        /// <param name="advectedPointWorldSpace">The world space point on the body to advect.</param>
        /// <returns>The new location of the advected point, after the given motion.</returns>
        public static Vector3 AdvectPointByVelocities(Vector3 velocity, Vector3 angularVelocity,
                                                      Vector3 bodyCenterOfMassWorldSpace,
                                                      Vector3 advectedPointWorldSpace)
        {
            Vector3 radialToPoint = advectedPointWorldSpace - bodyCenterOfMassWorldSpace;
            Vector3 advectedPointDelta = velocity * Time.deltaTime +
                                         Vector3.Cross(angularVelocity * Time.deltaTime, radialToPoint);
            return advectedPointWorldSpace + advectedPointDelta;
        }
    }
}
