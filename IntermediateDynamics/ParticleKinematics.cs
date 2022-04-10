using MathNet.Spatial.Euclidean;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Expr = MathNet.Symbolics.SymbolicExpression;

namespace IntermediateDynamics
{
    public class ParticleKinematics
    {
        /// <summary>
        /// Calculates velocity along path (ie odometer speed).
        /// Ginsberg, Jerry.Engineering Dynamics(pp. 33). Cambridge University Press. Kindle Edition.
        /// </summary>
        /// <param name="s">Arc length along the curve measure from some starting 
        /// point to point of interest as a function of time s(t).
        /// </param>
        /// <returns>Velocity along path v or s_dot</returns>
        /// <remarks>2.1.10</remarks>
        public static VectorExpr3D SolveForOdometerSpeed(VectorExpr3D s)
        {
            return new VectorExpr3D(s.X.Differentiate("t"), s.Y.Differentiate("t"), s.Z.Differentiate("t"));
        }

        /// <summary>
        /// Calculates velocity magnitude and direction 
        /// ie if stop spinning where do we start going towards and how fast
        /// Ginsberg, Jerry.Engineering Dynamics(pp. 33). Cambridge University Press. Kindle Edition.
        /// </summary>
        /// <param name="s_dot">the speed of the particle aka v.</param>
        /// <param name="e_t">the tangential unit vector (direction particle is heading).</param>
        /// <returns>Velocity in tangential direction</returns>
        /// <remarks>2.1.10</remarks>
        public static VectorExpr3D SolveForVelocityVector(VectorExpr3D s_dot, VectorExpr3D e_t)
        {
            return s_dot * e_t;
        }

        /// <summary>
        /// Calculates acceleration magnitude and direction by combining tangential (linear) and normal (centripetal)
        /// accelerations
        /// Ginsberg, Jerry.Engineering Dynamics(pp. 33). Cambridge University Press. Kindle Edition.
        /// </summary>
        /// <param name="v_dot">the tangential (linear) acceleration.</param>
        /// <param name="e_t">the tangential unit vector (direction particle is heading).</param>
        /// <param name="s_dot">The speed of the particle aka v.</param>
        /// <param name="rho">The radius of curvature.</param>
        /// <param name="e_n">The normal unit vector (direction particle is relative to center of rotation).</param>
        /// <returns>Acceleration of particle in magnitude and direction</returns>
        /// <remarks>2.1.10</remarks>
        public static VectorExpr3D SolveForAccelerationVector(VectorExpr3D v_dot, VectorExpr3D e_t, VectorExpr3D s_dot, double rho, VectorExpr3D e_n)
        {
            return v_dot * e_t + (s_dot * s_dot) / rho * e_n;
        }

        /// <summary>
        /// Calculates radius of curvature from acceleration vector
        /// Ginsberg, Jerry.Engineering Dynamics(pp. 33). Cambridge University Press. Kindle Edition.
        /// </summary>
        /// <param name="a">The acceleration of the particle.</param>
        /// <param name="v_dot">the tangential (linear) acceleration.</param>
        /// <param name="e_t">the tangential unit vector (direction particle is heading).</param>
        /// <param name="s_dot">The speed of the particle aka v.</param>
        /// <returns>Radius of curvature of particle</returns>
        /// <remarks>2.1.10</remarks>
        public static double SolveForRadiusOfCurvature(Vector3D a, double v_dot, Vector3D e_t, double s_dot)
        {
            var partOne = (a - v_dot * e_t);
            return (s_dot * s_dot) / partOne.Magnitude();
        }

        /// <summary>
        /// Calculates unit normal vector from acceleration vector
        /// Ginsberg, Jerry.Engineering Dynamics(pp. 37). Cambridge University Press. Kindle Edition.
        /// </summary>
        /// <param name="a">The acceleration of the particle.</param>
        /// <param name="v_dot">the tangential (linear) acceleration.</param>
        /// <param name="e_t">the tangential unit vector (direction particle is heading).</param>
        /// <param name="s_dot">The speed of the particle aka v.</param>
        /// <param name="rho">The radius of curvature.</param>
        /// <returns>Unit normal vector from particle to center of curvature.</returns>
        /// <remarks>2.1.10</remarks>
        public static Vector3D SolveForUnitNormalVector(Vector3D a, double v_dot, Vector3D e_t, double s_dot, double rho)
        {
            var partOne = (a - v_dot * e_t);
            var partTwo = (s_dot * s_dot / rho);
            return partOne / partTwo;
        }

        /// <summary>
        /// Calculates center of curvature
        /// Ginsberg, Jerry.Engineering Dynamics(pp. 37). Cambridge University Press. Kindle Edition.
        /// </summary>
        /// <param name="r">Particle location vector</param>
        /// <param name="rho">The radius of curvature.</param>
        /// <param name="e_n">The normal unit vector (direction particle is relative to center of rotation).</param>
        /// <returns>Vector of center of curvature</returns>
        /// <remarks>2.1.9</remarks>
        public static Vector3D SolveForCenterOfCurvature(Vector3D r, double rho, Vector3D e_n)
        {
            return r + rho * e_n;
        }

        /// <summary>
        /// Calculates binormal unit vector
        /// Ginsberg, Jerry.Engineering Dynamics(pp. 34). Cambridge University Press. Kindle Edition.
        /// </summary>
        /// <param name="e_t">The tangential unit vector of the particle.</param>
        /// <param name="e_n">The normal unit vector of the particle.</param>
        /// <returns>The binormal unit vector of the particle.</returns>
        /// <remarks>2.1.10</remarks>
        public static Vector3D SolveForBinormalUnitVector(Vector3D e_t, Vector3D e_n)
        {
            return e_t.CrossProduct(e_n);
        }

        /// <summary>
        /// Calculates external tangential forces
        /// Ginsberg, Jerry.Engineering Dynamics(pp. 37-38). Cambridge University Press. Kindle Edition.
        /// </summary>
        /// <param name="e_t">The tangential unit vector of the particle.</param>
        /// <param name="m">The mass of the particle.</param>
        /// <param name="v_dot">The tangential acceleration of the particle.</param
        /// <returns>The external tangential forces of particle.</returns>
        public static double SolveForExternalTangentialForces(Vector3D e_t, double m, double v_dot)
        {
            return m * v_dot - (-m * Constants.GRAVITATIONAL_ACCELERATION_ON_EARTH * e_t.Z);
        }

        /// <summary>
        /// Calculates external normal forces
        /// Ginsberg, Jerry.Engineering Dynamics(pp. 37-38). Cambridge University Press. Kindle Edition.
        /// </summary>
        /// <param name="e_n">The normal unit vector of the particle.</param>
        /// <param name="m">The mass of the particle.</param>
        /// <param name="s_dot">The speed of the particle aka v.</param
        /// <param name="rho">The radius of curvature of the particle.</param
        /// <returns>The external normal forces of particle.</returns>
        public static double SolveForExternalNormalForces(Vector3D e_n, double m, double s_dot, double rho)
        {
            return m * s_dot * s_dot / rho - (-m * Constants.GRAVITATIONAL_ACCELERATION_ON_EARTH * e_n.Z);
        }

        /// <summary>
        /// Calculates external binormal forces
        /// Ginsberg, Jerry.Engineering Dynamics(pp. 37-38). Cambridge University Press. Kindle Edition.
        /// </summary>
        /// <param name="e_b">The binormal unit vector of the particle.</param>
        /// <param name="m">The mass of the particle.</param>
        /// <returns>The external binormal forces of particle.</returns>
        public static double SolveForExternalBinormalForces(Vector3D e_b, double m)
        {
            return -(-m * Constants.GRAVITATIONAL_ACCELERATION_ON_EARTH * e_b.Z);
        }
    }
}
