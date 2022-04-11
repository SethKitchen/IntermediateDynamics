using IntermediateDynamics;
using NUnit.Framework;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Spatial.Euclidean;
using Expr = MathNet.Symbolics.SymbolicExpression;

namespace IntermediateDynamicsTests
{
    class TestParticleKinematics
    {
        /// <summary>
        ///  Ginsberg, Jerry.Engineering Dynamics(p. 35-36). Cambridge University Press. Kindle Edition.
        /// </summary>
        [Test]
        public void Test_Ex_2_1()
        {
            var r_ao = new Vector3D(4 * Math.Cos(60 * Math.PI / 180) * Math.Cos(75 * Math.PI / 180), 4 * Math.Cos(60 * Math.PI / 180) * Math.Sin(75 * Math.PI / 180), 4 * Math.Sin(60 * Math.PI / 180));
            var r_bo = new Vector3D(4, 0, 0);
            var e_ao = r_ao.UnitVector();
            var r_ba = r_bo - r_ao;
            var e_ba = r_ba.UnitVector();
            var e_oa = -e_ao;

            var a_bar = 10 * Constants.GRAVITATIONAL_ACCELERATION_ON_EARTH * e_oa;
            var v_dot = a_bar * e_ba;

            double rho = ParticleKinematics.SolveForRadiusOfCurvature(a_bar, v_dot, e_ba, 500);
            Assert.AreEqual(rho, 3395.4021526034639);

            Vector3D e_n = ParticleKinematics.SolveForUnitNormalVector(a_bar, v_dot, e_ba, 500, rho);

            Assert.AreEqual(e_n, new Vector3D(-0.7514684033780997, -0.32134612112329397, -0.5762221005509793));

            Vector3D center = ParticleKinematics.SolveForCenterOfCurvature(r_ao, rho, e_n);

            Assert.AreEqual(center, new Vector3D(-2551.0197963532833, -1089.1674597402275, -1953.0416589733472));

            Vector3D e_b = ParticleKinematics.SolveForBinormalUnitVector(e_ba, e_n);

            Assert.AreEqual(e_b, new Vector3D(-8.326672684688674E-17, 0.8733693623277108, -0.4870584738481488));

            double externalTangentialForces = ParticleKinematics.SolveForExternalTangentialForces(e_ba, 5, v_dot);

            Assert.AreEqual(externalTangentialForces, 291.06869244870018);

            double externalNormalForces = ParticleKinematics.SolveForExternalNormalForces(e_n, 5, 500, rho);

            Assert.AreEqual(externalNormalForces, 339.91574254246422);

            double externalBinormalForces = ParticleKinematics.SolveForExternalBinormalForces(e_b, 5);

            Assert.AreEqual(externalBinormalForces, -23.861029201124598);

        }
    }
}
