using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Expr = MathNet.Symbolics.SymbolicExpression;

namespace IntermediateDynamics
{
    public static class LaTexHelpers
    {
        public const string I_HAT_LATEX = @"\hat{\textbf{i}}";
        public const string J_HAT_LATEX = @"\hat{\textbf{j}}";
        public const string K_HAT_LATEX = @"\hat{\textbf{k}}";
        public const string UNIT_HAT_LATEX = @"\hat{\textbf{e}}";

        /// <summary>
        /// Bolds and arrows a variable expression so it is in vector notation.
        /// </summary>
        /// <param name="variable">A variable, ie. x</param>
        /// <returns>The LaTex vector notation of the variable, ie \vec{x} .</returns>
        public static string VectorizeLaTex(Expr variable)
        {
            return @"\vec{" + variable.ToLaTeX() + "}";
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
        /// Bold and arrows e around a variable expression so it is in unit vector notation.
        /// </summary>
        /// <param name="variable">A variable, ie. x</param>
        /// <returns>The LaTex vector notation of the magnitude of the variable, ie \abs{x}</returns>
        public static string UnitVectorLaTex(Expr variable)
        {
            return UNIT_HAT_LATEX + "_{" + variable.ToLaTeX() + "}";
        }
    }
}
