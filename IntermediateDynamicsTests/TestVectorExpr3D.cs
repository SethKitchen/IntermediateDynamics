using NUnit.Framework;
using IntermediateDynamics;
using MathNet.Spatial.Euclidean;
using Expr = MathNet.Symbolics.SymbolicExpression;
using System;

namespace IntermediateDynamicsTests
{
    public class TestVectorExpr3D
    {
        [SetUp]
        public void Setup()
        {
        }

        /// <summary>
        ///  Ginsberg, Jerry.Engineering Dynamics(p. 11). Cambridge University Press. Kindle Edition.
        /// </summary>
        [Test]
        public void Test_Ex_1_2_1()
        {
            var r_t = Expr.Parse("2000 + 100 * t");
            var theta_t = Expr.Parse("3.14159265/2 * (1 - 2.71828^(-0.15 * t))");
            var beta_t = Expr.Parse("3.14159265/3 - 0.1*t^(0.5)");
            VectorExpr3D toSolve = VectorExpr3D.PositionVectorApprox(r_t, theta_t, beta_t);
            Vector3D ans = toSolve.GetFiniteCentralDifferenceApproximation(2, 1e-3);
            Assert.AreEqual(Math.Round(ans.X, 1), 19.0);
            Assert.AreEqual(Math.Round(ans.Y, 1), 266.3);
            Assert.AreEqual(Math.Round(ans.Z, 1), 30.7);
        }

        /// <summary>
        ///  Ginsberg, Jerry.Engineering Dynamics(p. 11). Cambridge University Press. Kindle Edition.
        /// </summary>
        [Test]
        public void Test_Ex_1_2_2()
        {
            var r_t = Expr.Parse("2000 + 100 * t");
            var theta_t = Expr.Parse("3.14159265/2 * (1 - 2.71828^(-0.15 * t))");
            var beta_t = Expr.Parse("3.14159265/3 - 0.1*t^(0.5)");
            VectorExpr3D toSolve = VectorExpr3D.PositionVector(r_t, theta_t, beta_t);
            Vector3D ans = toSolve.SolveAtTime(2);
            Assert.AreEqual(Math.Round(ans.X, 1), 19.0);
            Assert.AreEqual(Math.Round(ans.Y, 1), 266.3);
            Assert.AreEqual(Math.Round(ans.Z, 1), 30.7);
        }
    }
}