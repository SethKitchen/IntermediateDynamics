using NUnit.Framework;
using IntermediateDynamics;
using MathNet.Spatial.Euclidean;
using MathNet.Symbolics;
using Expr = MathNet.Symbolics.SymbolicExpression;
using System;

namespace IntermediateDynamicsTests
{
    public class TestForces
    {
        [SetUp]
        public void Setup()
        {
        }

        /// <summary>
        ///  Ginsberg, Jerry.Engineering Dynamics(p. 16). Cambridge University Press. Kindle Edition.
        /// </summary>
        [Test]
        public void Test_Eq_1_2_6()
        {
            Assert.AreEqual(Math.Round(Constants.GRAVITATIONAL_ACCELERATION_ON_EARTH, 1), 9.8);
            Assert.AreEqual(Constants.GRAVITATIONAL_ACCELERATION_ON_EARTH, Forces.GravitationalAccelerationMagnitude(Constants.MASS_OF_EARTH, Constants.RADIUS_OF_EARTH));
        }
    }
}