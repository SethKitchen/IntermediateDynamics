using MathNet.Symbolics;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Expr = MathNet.Symbolics.SymbolicExpression;

namespace IntermediateDynamics
{
    public class Util
    {
        public static double SolveForVariable(double lower_bound, double max_search_value, string variable, double rhsAns, double margin, Expr toSolve)
        {
            for (double i = lower_bound + 0.001; i <= max_search_value; i += 0.001)
            {
                var symbols = new Dictionary<string, FloatingPoint> { { variable, i } };
                FloatingPoint ans = toSolve.Evaluate(symbols);
                if (Math.Abs(ans.RealValue - rhsAns) < margin)
                {
                    return i;
                }
            }
            return 0;
        }
    }
}
