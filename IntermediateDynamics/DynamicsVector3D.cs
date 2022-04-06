using MathNet.Spatial.Euclidean;
using System;
using MathNet.Symbolics;
using Expr = MathNet.Symbolics.SymbolicExpression;

namespace IntermediateDynamics
{
    public static class DynamicsVector3D
    {
        public const string I_HAT_LATEX = @"\hat{\textbf{i}}";
        public const string J_HAT_LATEX = @"\hat{\textbf{j}}";
        public const string K_HAT_LATEX = @"\hat{\textbf{k}}";
        public const string UNIT_HAT_LATEX = @"\hat{\textbf{e}}";

        /// <summary>
        /// "The utility of a component representation is that operations can be performed on the individual components without recourse to diagrams."
        /// Ginsberg, Jerry.Engineering Dynamics(pp. 3). Cambridge University Press. Kindle Edition.
        /// </summary>
        /// <param name="vector">The vector which we would like to see the representation of.</param>
        /// <returns>LaTex string of the component representation of the given <c>vector</c>.</returns>
        /// <remarks>1.1.5</remarks>
        public static string ComponentRepresentation(this Vector3D vector)
        {
            return vector.X + I_HAT_LATEX + "+" + vector.Y + J_HAT_LATEX + "+" + vector.Z + K_HAT_LATEX;
        }

        /// <summary>
        /// Bolds and arrows a variable expression so it is in vector notation.
        /// </summary>
        /// <param name="variable">A variable, ie. x</param>
        /// <returns>The LaTex vector notation of the variable, ie \vec{x} .</returns>
        public static string LaTex(Expr variable)
        {
            return @"\vec{"+variable.ToLaTeX()+"}";
        }

        /// <summary>
        /// "By the Pythagorean theorem the magnitude of is [the sqrt of the components squared]".
        /// Ginsberg, Jerry.Engineering Dynamics(p. 4). Cambridge University Press. Kindle Edition.
        /// </summary>
        /// <param name="vector">The vector which we would like to see the magnitude of.</param>
        /// <returns>Decimal value of the magnitude of the given <c>vector</c>.</returns>
        /// <remarks>1.1.6</remarks>
        public static double Magnitude(this Vector3D vector)
        {
            return Math.Sqrt(vector.X * vector.X + vector.Y * vector.Y + vector.Z * vector.Z);
        }

        /// <summary>
        /// Vertical bars around a variable expression so it is in magnitude notation.
        /// </summary>
        /// <param name="variable">A variable, ie. x</param>
        /// <returns>The LaTex vector notation of the magnitude of the variable, ie \abs{x}</returns>
        public static string MagnitudeLaTex(Expr variable)
        {
            return @"\abs{" + variable.ToLaTeX() + "}";
        }

        /// <summary>
        /// "In many situations we need to construct a unit vector parallel to a vector. This is readily obtained from the preceding equation as
        /// [vector divided by its magnitude]".
        /// Ginsberg, Jerry.Engineering Dynamics(p. 4). Cambridge University Press. Kindle Edition.
        /// </summary>
        /// <param name="vector">The vector which we would like to see the unit vector of.</param>
        /// <returns>Vector with magnitude of 1 parallel to the given <c>vector</c></returns>
        /// <remarks>1.1.7</remarks>
        public static Vector3D UnitVector(this Vector3D vector)
        {
            return vector / vector.Magnitude();
        }

        /// <summary>
        /// Bold and arrows e around a variable expression so it is in unit vector notation.
        /// </summary>
        /// <param name="variable">A variable, ie. x</param>
        /// <returns>The LaTex vector notation of the magnitude of the variable, ie \abs{x}</returns>
        public static string UnitVectorLaTex(Expr variable)
        {
            return UNIT_HAT_LATEX + "_{" + variable.ToLaTeX() + "}";
        }

        /// <summary>
        /// REIMPLMENTED FOR LEARNING (equivalent to AngleTo)
        /// When vectors are placed tail to tail what is the angle between them?
        /// Ginsberg, Jerry.Engineering Dynamics(p. 4). Cambridge University Press. Kindle Edition.
        /// </summary>
        /// <param name="one">The first of the compared vectors.</param>
        /// <param name="two">The second of the compared vectors.</param>
        /// <returns>The arccos of the dot product and magnitude of the given <c>one</c> and <c>two</c> vectors.</returns>
        /// <remarks>1.1.9</remarks>
        public static double AngleBetween(this Vector3D one, Vector3D two)
        {
            return Math.Acos(one.DotProduct(two) / (one.Magnitude() * two.Magnitude()));
        }




    }
}
