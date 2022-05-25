using System.Collections.Generic;
using Expr = MathNet.Symbolics.SymbolicExpression;

namespace IntermediateDynamics
{
    public class Forces
    {
        /// <summary>
        /// The resultant force acting on a particle is proportional to the acceleration of the particle.
        /// Ginsberg, Jerry.Engineering Dynamics(pp. 14). Cambridge University Press. Kindle Edition.
        /// </summary>
        /// <param name="forces">All effects acting on particle.</param>
        /// <param name="mass">Amount of matter of particle</param>
        /// <returns>The acceleration vector of the particle</returns>
        public static VectorExpr3D SolveForAcceleration(List<VectorExpr3D> forces, double mass)
        {
            var toReturn = new VectorExpr3D(Expr.Zero, Expr.Zero, Expr.Zero);
            foreach (VectorExpr3D v in forces)
            {
                toReturn += v;
            }
            return toReturn / mass;
        }

        /// <summary>
        /// The resultant force acting on a particle is proportional to the acceleration of the particle.
        /// Ginsberg, Jerry.Engineering Dynamics(pp. 14). Cambridge University Press. Kindle Edition.
        /// </summary>
        /// <param name="acceleration">The acceleration vector of the particle</param>
        /// <param name="mass">Amount of matter of particle</param>
        /// <returns>The resultant force acting on a particle</returns>
        public static VectorExpr3D SolveForSumOfForces(VectorExpr3D acceleration, double mass)
        {
            return acceleration * mass;
        }

        /// <summary>
        /// The magnitude of the attractive force exerted between the Earth and a body of mass.
        /// Ginsberg, Jerry.Engineering Dynamics(pp. 15). Cambridge University Press. Kindle Edition.
        /// </summary>
        /// <param name="r">Distance between the centers of mass of Earth and the particle.</param>
        /// <param name="mass">Amount of matter of particle</param>
        /// <returns>The resultant force magnitude acting on a particle</returns>
        /// <remarks>1.2.4</remarks>
        public static double SolveForGravitationalForceMagnitude(double r, double mass)
        {
            return Constants.UNIVERSAL_GRAVITATIONAL_CONSTANT_G * Constants.MASS_OF_EARTH * mass / (r * r);
        }

        /// <summary>
        /// Calculates gravitational acceleration on a planet given its mass and radius
        /// Ginsberg, Jerry.Engineering Dynamics(pp. 16). Cambridge University Press. Kindle Edition.
        /// </summary>
        /// <returns>Gravitational Acceleration magnitude.</returns>
        /// <remarks>1.2.6</remarks>
        public static double GravitationalAccelerationMagnitude(double massOfPlanet, double radiusOfPlanet)
        {
            return Constants.UNIVERSAL_GRAVITATIONAL_CONSTANT_G * massOfPlanet / (radiusOfPlanet * radiusOfPlanet);
        }
    }
}
