using MathNet.Numerics.LinearAlgebra;
using MathNet.Spatial.Euclidean;
using System;

namespace IntermediateDynamics
{
    public static class DynamicsVector3D
    {
        /// <summary>
        /// "The utility of a component representation is that operations can be performed on the individual components without recourse to diagrams."
        /// Ginsberg, Jerry.Engineering Dynamics(pp. 3). Cambridge University Press. Kindle Edition.
        /// </summary>
        /// <param name="vector">The vector which we would like to see the representation of.</param>
        /// <returns>LaTex string of the component representation of the given <c>vector</c>.</returns>
        /// <remarks>1.1.5</remarks>
        public static string ComponentRepresentation(this Vector3D vector)
        {
            string toReturn = "";
            if (vector.X != 0)
            {
                toReturn += vector.X + LaTexHelpers.I_HAT_LATEX;
                if (vector.Y != 0)
                {
                    toReturn += "+" + vector.Y + LaTexHelpers.J_HAT_LATEX;
                    if (vector.Z != 0)
                    {
                        toReturn += "+" + vector.Z + LaTexHelpers.K_HAT_LATEX;
                    }
                }
                else
                {
                    if (vector.Z != 0)
                    {
                        toReturn += "+" + vector.Z + LaTexHelpers.K_HAT_LATEX;
                    }
                }
            }
            else
            {
                if (vector.Y != 0)
                {
                    toReturn += vector.Y + LaTexHelpers.J_HAT_LATEX;
                    if (vector.Z != 0)
                    {
                        toReturn += "+" + vector.Z + LaTexHelpers.K_HAT_LATEX;
                    }
                }
                else
                {
                    if (vector.Z != 0)
                    {
                        toReturn += vector.Z + LaTexHelpers.K_HAT_LATEX;
                    }
                }
            }

            return toReturn;
        }

        /// <summary>
        /// "The utility of a component representation is that operations can be performed on the individual components without recourse to diagrams."
        /// Ginsberg, Jerry.Engineering Dynamics(pp. 3). Cambridge University Press. Kindle Edition.
        /// </summary>
        /// <param name="vector">The vector which we would like to see the representation of.</param>
        /// <returns>LaTex string of the component representation of the given <c>vector</c>.</returns>
        /// <remarks>1.1.5</remarks>
        public static string ComponentRepresentation(this Vector2D vector)
        {
            string toReturn = "";
            if (vector.X != 0)
            {
                toReturn += vector.X + LaTexHelpers.I_HAT_LATEX;
                if (vector.Y != 0)
                {
                    toReturn += "+" + vector.Y + LaTexHelpers.J_HAT_LATEX;
                }
            }
            else
            {
                if (vector.Y != 0)
                {
                    toReturn += vector.Y + LaTexHelpers.J_HAT_LATEX;
                }
            }

            return toReturn;
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
        /// "By the Pythagorean theorem the magnitude of is [the sqrt of the components squared]".
        /// Ginsberg, Jerry.Engineering Dynamics(p. 4). Cambridge University Press. Kindle Edition.
        /// </summary>
        /// <param name="vector">The vector which we would like to see the magnitude of.</param>
        /// <returns>Decimal value of the magnitude of the given <c>vector</c>.</returns>
        /// <remarks>1.1.6</remarks>
        public static double Magnitude(this Vector2D vector)
        {
            return Math.Sqrt(vector.X * vector.X + vector.Y * vector.Y);
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
        /// "In many situations we need to construct a unit vector parallel to a vector. This is readily obtained from the preceding equation as
        /// [vector divided by its magnitude]".
        /// Ginsberg, Jerry.Engineering Dynamics(p. 4). Cambridge University Press. Kindle Edition.
        /// </summary>
        /// <param name="vector">The vector which we would like to see the unit vector of.</param>
        /// <returns>Vector with magnitude of 1 parallel to the given <c>vector</c></returns>
        /// <remarks>1.1.7</remarks>
        public static Vector2D UnitVector(this Vector2D vector)
        {
            return vector / vector.Magnitude();
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

        /// <summary>
        /// REIMPLMENTED FOR LEARNING (equivalent to AngleTo)
        /// When vectors are placed tail to tail what is the angle between them?
        /// Ginsberg, Jerry.Engineering Dynamics(p. 4). Cambridge University Press. Kindle Edition.
        /// </summary>
        /// <param name="one">The first of the compared vectors.</param>
        /// <param name="two">The second of the compared vectors.</param>
        /// <returns>The arccos of the dot product and magnitude of the given <c>one</c> and <c>two</c> vectors.</returns>
        /// <remarks>1.1.9</remarks>
        public static double AngleBetween(this Vector2D one, Vector2D two)
        {
            return Math.Acos(one.DotProduct(two) / (one.Magnitude() * two.Magnitude()));
        }

        public static VectorExpr3D ConvertToExpr(this Vector3D toConvert)
        {
            return new VectorExpr3D(toConvert.X, toConvert.Y, toConvert.Z);
        }

        public static Matrix<double> GetMatrix(this Vector3D toConvert)
        {
            Matrix<double> toReturn = Matrix<double>.Build.Dense(3, 1);
            toReturn[0,0] = toConvert.X;
            toReturn[1,0] = toConvert.Y;
            toReturn[2,0] = toConvert.Z;
            return toReturn;
        }

        public static Vector3D FromMatrix(this Matrix<double> toConvert)
        {
            return new Vector3D(toConvert[0,0], toConvert[1,0], toConvert[2,0]);
        }
    }
}
