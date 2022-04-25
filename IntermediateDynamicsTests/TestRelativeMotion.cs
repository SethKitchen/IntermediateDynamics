using IntermediateDynamics;
using NUnit.Framework;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Spatial.Euclidean;
using Expr = MathNet.Symbolics.SymbolicExpression;
using MathNet.Numerics.Integration;
using MathNet.Symbolics;

namespace IntermediateDynamicsTests
{
    class TestRelativeMotion
    {
        /// <summary>
        ///  Ginsberg, Jerry.Engineering Dynamics(p. 96). Cambridge University Press. Kindle Edition.
        /// </summary>
        [Test]
        public void Test_Ex_3_1()
        {
            var r_bar_a = new Vector3D(-250, 400, -500);
            var r_bar_b = new Vector3D(400, -600, 200);


        }
    }
}
