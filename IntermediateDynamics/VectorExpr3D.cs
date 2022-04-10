using MathNet.Spatial.Euclidean;
using MathNet.Symbolics;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Expr = MathNet.Symbolics.SymbolicExpression;

namespace IntermediateDynamics
{
    /// <summary>
    /// Vector3D is numerical - this class is symbolic so we can see full expressions.
    /// </summary>
    public class VectorExpr3D
    {
        /// <summary>
        /// The X Component.
        /// </summary>
        public Expr X { get; set; }

        /// <summary>
        /// The Y Component.
        /// </summary>
        public Expr Y { get; set; }

        /// <summary>
        /// The Z Component.
        /// </summary>
        public Expr Z { get; set; }

        private VectorExpr3D()
        {

        }

        public VectorExpr3D(Expr x, Expr y, Expr z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        public static VectorExpr3D operator *(VectorExpr3D v1, double c)
        {
            return new VectorExpr3D(v1.X * c, v1.Y * c, v1.Z * c);
        }

        public static VectorExpr3D operator *(VectorExpr3D v1, VectorExpr3D v2)
        {
            return new VectorExpr3D(v1.X * v2.X, v1.Y * v2.Y, v1.Z * v2.Z);
        }

        public static VectorExpr3D operator +(VectorExpr3D v1, VectorExpr3D v2)
        {
            return new VectorExpr3D(v1.X + v2.X, v1.Y + v2.Y, v1.Z + v2.Z);
        }

        public static VectorExpr3D operator +(VectorExpr3D v1, double c)
        {
            return new VectorExpr3D(v1.X + c, v1.Y + c, v1.Z + c);
        }

        public static VectorExpr3D operator /(VectorExpr3D v1, VectorExpr3D v2)
        {
            return new VectorExpr3D(v1.X / v2.X, v1.Y / v2.Y, v1.Z / v2.Z);
        }

        public static VectorExpr3D operator /(VectorExpr3D v1, double c)
        {
            return new VectorExpr3D(v1.X / c, v1.Y / c, v1.Z / c);
        }

        public static VectorExpr3D operator -(VectorExpr3D v1, VectorExpr3D v2)
        {
            return new VectorExpr3D(v1.X - v2.X, v1.Y - v2.Y, v1.Z - v2.Z);
        }

        public static VectorExpr3D operator -(VectorExpr3D v1, double c)
        {
            return new VectorExpr3D(v1.X - c, v1.Y - c, v1.Z - c);
        }

        public VectorExpr3D CrossProduct(VectorExpr3D v2)
        {
            return new VectorExpr3D(Y * v2.Z - Z * v2.Y, Z * v2.X - X * v2.Z, X * v2.Y - Y * v2.X);
        }

        /// <summary>
        /// Hacks the definition of a derivative to get a approximation of velocity at a certain time using a small interval
        /// Ginsberg, Jerry.Engineering Dynamics(pp. 12). Cambridge University Press. Kindle Edition.
        /// </summary>
        /// <param name="time">Time at which to evaluate the derivative.</param>
        /// <param name="interval">dt: The small length of time over which a change occurs ie 1 ms.</param>
        /// <returns></returns>
        public Vector3D GetFiniteCentralDifferenceApproximation(double time, double interval)
        {
            var partOne = SolveAtTime(time + interval / 2.0);
            var partTwo = SolveAtTime(time - interval / 2.0);
            return (partOne - partTwo) / interval;
        }

        /// <summary>
        /// Evaluates the vector expression at the given time.
        /// </summary>
        /// <param name="time">Time to evaluate the symbolic expression at to get a constant.</param>
        /// <returns></returns>
        public Vector3D SolveAtTime(double time)
        {
            var symbols = new Dictionary<string, FloatingPoint> { { "t", time } };
            return new Vector3D(X.Evaluate(symbols).RealValue, Y.Evaluate(symbols).RealValue, Z.Evaluate(symbols).RealValue);
        }

        /// <summary>
        /// Returns the position vector based on distance, angle in the horizontal xy plane, and angle of elevation with respect to time.
        /// APPROXIMATELY BECAUSE: Does not consider analytical derivatives.
        /// Ginsberg, Jerry.Engineering Dynamics(p. 11). Cambridge University Press. Kindle Edition.
        /// </summary>
        /// <param name="r">Distance with respect to time. r(t)</param>
        /// <param name="theta">Angle in the horizontal xy plane in radians. theta(t)</param>
        /// <param name="beta">Angle of elevation in radians. beta(t)</param>
        /// <returns>Position Vector based on given parameters.</returns>
        /// <remarks>Formula (1) in example 1.2</remarks>
        public static VectorExpr3D PositionVectorApprox(Expr r, Expr theta, Expr beta)
        {
            var cosBeta = Expr.Parse("cos(" + beta.ToString() + ")");
            var cosTheta = Expr.Parse("cos(" + theta.ToString() + ")");
            var sinBeta = Expr.Parse("sin(" + beta.ToString() + ")");
            var sinTheta = Expr.Parse("sin(" + theta.ToString() + ")");
            return new VectorExpr3D(r * cosBeta * cosTheta, r * cosBeta * sinTheta, r * sinBeta);
        }

        /// <summary>
        /// Returns the position vector based on distance, angle in the horizontal xy plane, and angle of elevation with respect to time.
        /// Ginsberg, Jerry.Engineering Dynamics(p. 12). Cambridge University Press. Kindle Edition.
        /// </summary>
        /// <param name="r">Distance with respect to time. r(t)</param>
        /// <param name="theta">Angle in the horizontal xy plane in radians. theta(t)</param>
        /// <param name="beta">Angle of elevation in radians. beta(t)</param>
        /// <returns>Position Vector based on given parameters.</returns>
        /// <remarks>Formula (1) in example 1.2</remarks>
        public static VectorExpr3D PositionVector(Expr r, Expr theta, Expr beta)
        {
            var cosBeta = Expr.Parse("cos(" + beta.ToString() + ")");
            var cosTheta = Expr.Parse("cos(" + theta.ToString() + ")");
            var sinBeta = Expr.Parse("sin(" + beta.ToString() + ")");
            var sinTheta = Expr.Parse("sin(" + theta.ToString() + ")");
            var thetaDot = theta.Differentiate(Expr.Variable("t"));
            var betaDot = beta.Differentiate(Expr.Variable("t"));
            var rDot = r.Differentiate(Expr.Variable("t"));

            var partOne = new VectorExpr3D(rDot * cosBeta * cosTheta, rDot * cosBeta * sinTheta, rDot * sinBeta);
            var partTwo = new VectorExpr3D(thetaDot * -(r * cosBeta * sinTheta), thetaDot * r * cosBeta * cosTheta, Expr.Zero);
            var partThree = new VectorExpr3D(betaDot * -(r * sinBeta * cosTheta), betaDot * -(r * sinBeta * sinTheta), betaDot * r * cosBeta);
            return partOne + partTwo + partThree;
        }

        /// <summary>
        /// "The utility of a component representation is that operations can be performed on the individual components without recourse to diagrams."
        /// Ginsberg, Jerry.Engineering Dynamics(pp. 3). Cambridge University Press. Kindle Edition.
        /// </summary>
        /// <param name="vector">The vector which we would like to see the representation of.</param>
        /// <returns>LaTex string of the component representation of the given <c>vector</c>.</returns>
        /// <remarks>1.1.5</remarks>
        public string ComponentRepresentation()
        {
            string toReturn = "";
            if (X.ToString() != "0")
            {
                toReturn += X.ToLaTeX() + LaTexHelpers.I_HAT_LATEX;
                if (Y.ToString() != "0")
                {
                    toReturn += "+" + Y.ToLaTeX() + LaTexHelpers.J_HAT_LATEX;
                    if (Z.ToString() != "0")
                    {
                        toReturn += "+" + Z.ToLaTeX() + LaTexHelpers.K_HAT_LATEX;
                    }
                }
                else
                {
                    if (Z.ToString() != "0")
                    {
                        toReturn += "+" + Z.ToLaTeX() + LaTexHelpers.K_HAT_LATEX;
                    }
                }
            }
            else
            {
                if (Y.ToString() != "0")
                {
                    toReturn += Y.ToLaTeX() + LaTexHelpers.J_HAT_LATEX;
                    if (Z.ToString() != "0")
                    {
                        toReturn += "+" + Z.ToLaTeX() + LaTexHelpers.K_HAT_LATEX;
                    }
                }
                else
                {
                    if (Z.ToString() != "0")
                    {
                        toReturn += Z.ToLaTeX() + LaTexHelpers.K_HAT_LATEX;
                    }
                }
            }

            return toReturn;
        }
    }
}
