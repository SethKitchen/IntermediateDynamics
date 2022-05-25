using MathNet.Numerics.LinearAlgebra;
using MathNet.Symbolics;
using System.Collections.Generic;
using Expr = MathNet.Symbolics.SymbolicExpression;

namespace IntermediateDynamics
{
    public class RelativeMotion
    {
        public static Matrix<double> GetRotationMatrixXAxis(double theta_x)
        {
            Dictionary<string, FloatingPoint> symbols = new() { { "θ", theta_x } };
            Matrix<double> toReturn = Matrix<double>.Build.Dense(3, 3);
            toReturn[0, 0] = 1;
            toReturn[0, 1] = 0;
            toReturn[0, 2] = 0; 
            toReturn[1, 0] = 0;
            toReturn[1, 1] = Expr.Parse("cos(θ)").Evaluate(symbols).RealValue;
            toReturn[1, 2] = Expr.Parse("sin(θ)").Evaluate(symbols).RealValue;
            toReturn[2, 0] = 0;
            toReturn[2, 1] = Expr.Parse("-sin(θ)").Evaluate(symbols).RealValue;
            toReturn[2, 2] = Expr.Parse("cos(θ)").Evaluate(symbols).RealValue;
            return toReturn;
        }

        public static Matrix<double> GetRotationMatrixYAxis(double theta_y)
        {
            Dictionary<string, FloatingPoint> symbols = new() { { "θ", theta_y } };
            Matrix<double> toReturn = Matrix<double>.Build.Dense(3, 3);
            toReturn[0, 0] = Expr.Parse("cos(θ)").Evaluate(symbols).RealValue;
            toReturn[0, 1] = 0;
            toReturn[0, 2] = Expr.Parse("-sin(θ)").Evaluate(symbols).RealValue;
            toReturn[1, 0] = 0;
            toReturn[1, 1] = 1;
            toReturn[1, 2] = 0;
            toReturn[2, 0] = Expr.Parse("sin(θ)").Evaluate(symbols).RealValue;
            toReturn[2, 1] = 0;
            toReturn[2, 2] = Expr.Parse("cos(θ)").Evaluate(symbols).RealValue;
            return toReturn;
        }

        public static Matrix<double> GetRotationMatrixZAxis(double theta_z)
        {
            Dictionary<string, FloatingPoint> symbols = new() { { "θ", theta_z } };
            Matrix<double> toReturn = Matrix<double>.Build.Dense(3, 3);
            toReturn[0, 0] = Expr.Parse("cos(θ)").Evaluate(symbols).RealValue;
            toReturn[0, 1] = Expr.Parse("sin(θ)").Evaluate(symbols).RealValue;
            toReturn[0, 2] = 0;
            toReturn[1, 0] = Expr.Parse("-sin(θ)").Evaluate(symbols).RealValue;
            toReturn[1, 1] = Expr.Parse("cos(θ)").Evaluate(symbols).RealValue;
            toReturn[1, 2] = 0;
            toReturn[2, 0] = 0;
            toReturn[2, 1] = 0;
            toReturn[2, 2] = 1;
            return toReturn;
        }        
    }
}
