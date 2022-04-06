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

        public static string ComponentRepresentation(this Vector3D vector)
        {
            return vector.X + I_HAT_LATEX + "+" + vector.Y + J_HAT_LATEX + "+" + vector.Z + K_HAT_LATEX;
        }
    }
}
