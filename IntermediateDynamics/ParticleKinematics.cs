using MathNet.Numerics.Integration;
using MathNet.Spatial.Euclidean;
using System;
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
        /// Calculates bi-normal unit vector
        /// Ginsberg, Jerry.Engineering Dynamics(pp. 34). Cambridge University Press. Kindle Edition.
        /// </summary>
        /// <param name="e_t">The tangential unit vector of the particle.</param>
        /// <param name="e_n">The normal unit vector of the particle.</param>
        /// <returns>The bi-normal unit vector of the particle.</returns>
        /// <remarks>2.1.10</remarks>
        public static Vector3D SolveForBiNormalUnitVector(Vector3D e_t, Vector3D e_n)
        {
            return e_t.CrossProduct(e_n);
        }

        /// <summary>
        /// Calculates external tangential forces
        /// Ginsberg, Jerry.Engineering Dynamics(pp. 37-38). Cambridge University Press. Kindle Edition.
        /// </summary>
        /// <param name="e_t">The tangential unit vector of the particle.</param>
        /// <param name="m">The mass of the particle.</param>
        /// <param name="v_dot">The tangential acceleration of the particle.</param>
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
        /// <param name="s_dot">The speed of the particle aka v.</param>
        /// <param name="rho">The radius of curvature of the particle.</param>
        /// <returns>The external normal forces of particle.</returns>
        public static double SolveForExternalNormalForces(Vector3D e_n, double m, double s_dot, double rho)
        {
            return m * s_dot * s_dot / rho - (-m * Constants.GRAVITATIONAL_ACCELERATION_ON_EARTH * e_n.Z);
        }

        /// <summary>
        /// Calculates external bi-normal forces
        /// Ginsberg, Jerry.Engineering Dynamics(pp. 37-38). Cambridge University Press. Kindle Edition.
        /// </summary>
        /// <param name="e_b">The bi-normal unit vector of the particle.</param>
        /// <param name="m">The mass of the particle.</param>
        /// <returns>The external bi-normal forces of particle.</returns>
        public static double SolveForExternalBiNormalForces(Vector3D e_b, double m)
        {
            return -(-m * Constants.GRAVITATIONAL_ACCELERATION_ON_EARTH * e_b.Z);
        }

        /// <summary>
        /// Calculates r_bar_prime from first derivatives of components.
        /// Ginsberg, Jerry.Engineering Dynamics(pp. 41). Cambridge University Press. Kindle Edition.
        /// </summary>
        /// <param name="x_prime">The first derivative of the x component function of the particle.</param>
        /// <param name="y_prime">The first derivative of the y component function of the particle.</param>
        /// <param name="z_prime">The first derivative of the z component function of the particle.</param>
        /// <returns>r_bar_prime</returns>
        public static Expr SolveForRBarPrimeFromFirstDerivativeComponents(Expr x_prime, Expr y_prime, Expr z_prime)
        {
            var toReturn = x_prime * x_prime + y_prime * y_prime + z_prime * z_prime;
            toReturn = Expr.Parse("(" + toReturn.ToString() + ")^0.5");
            return toReturn;
        }

        /// <summary>
        /// Root finds parametric from expected integral equivalence.
        /// Ginsberg, Jerry.Engineering Dynamics(pp. 41-42). Cambridge University Press. Kindle Edition.
        /// </summary>
        /// <param name="integral_equivalence">In the example it is s(B_p). At the interested time, what value do the components make?</param>
        /// <param name="lower_bound_of_integral">In the example it is 0. What is the parametric at the initial component state?</param>
        /// <param name="max_search_value">How high to look before giving up?</param>
        /// <param name="function_to_integrate">In the example it is s'. At the interested time, what is the derivative of the vector that the components make up?</param>
        /// <param name="margin">How close does the approximation need to be to the correct answer?</param>
        /// <returns>r_bar_prime</returns>
        public static double RootFindParametric(double integral_equivalence, double lower_bound_of_integral, double max_search_value, Func<double, double> function_to_integrate, double margin)
        {
            for (double i = lower_bound_of_integral + 0.01; i <= max_search_value; i += 0.01)
            {
                var toCheck = GaussLegendreRule.Integrate(function_to_integrate, lower_bound_of_integral, i, 5);
                if (Math.Abs(toCheck - integral_equivalence) < margin)
                {
                    return i;
                }
            }
            return -1;
        }

        /// <summary>
        /// Gets the tangent unit vector from parametric functions.
        /// Ginsberg, Jerry.Engineering Dynamics(pp. 37). Cambridge University Press. Kindle Edition.
        /// </summary>
        /// <param name="r_bar_prime">r'</param>
        /// <param name="s_prime">s'</param>
        /// <returns>e_t</returns>
        public static Vector3D GetTangentUnitVector(Vector3D r_bar_prime, double s_prime)
        {
            return r_bar_prime / s_prime;
        }

        /// <summary>
        /// Gets the normal unit vector from parametric functions.
        /// Ginsberg, Jerry.Engineering Dynamics(pp. 39). Cambridge University Press. Kindle Edition.
        /// </summary>
        /// <param name="r_bar_prime">r'</param>
        /// <param name="r_bar_double_prime">r'</param>
        /// <param name="s_prime">s'</param>
        /// <returns>e_n</returns>
        public static Vector3D GetNormalUnitVector(Vector3D r_bar_prime, Vector3D r_bar_double_prime, double s_prime)
        {
            var partOne = (s_prime * s_prime * r_bar_double_prime) - (r_bar_prime.DotProduct(r_bar_double_prime) * r_bar_prime);
            var partTwo = s_prime * Math.Pow((s_prime * s_prime) * (r_bar_double_prime.DotProduct(r_bar_double_prime)) - Math.Pow(r_bar_prime.DotProduct(r_bar_double_prime), 2), 0.5);
            return partOne / partTwo;
        }

        /// <summary>
        /// Gets the radius of curvature from parametric functions.
        /// Ginsberg, Jerry.Engineering Dynamics(pp. 39). Cambridge University Press. Kindle Edition.
        /// </summary>
        /// <param name="r_bar_prime">r'</param>
        /// <param name="r_bar_double_prime">r'</param>
        /// <param name="s_prime">s'</param>
        /// <returns>rho</returns>
        public static double GetRadiusOfCurvatureParametric(Vector3D r_bar_prime, Vector3D r_bar_double_prime, double s_prime)
        {
            var partOne = s_prime * s_prime * s_prime;
            var partTwo = Math.Pow((r_bar_double_prime.DotProduct(r_bar_double_prime) * s_prime * s_prime - Math.Pow(r_bar_prime.DotProduct(r_bar_double_prime), 2)), 0.5);
            return partOne / partTwo;
        }
    }
}
