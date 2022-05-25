using NUnit.Framework;
using IntermediateDynamics;
using MathNet.Spatial.Euclidean;
using System;

namespace IntermediateDynamicsTests
{
    public class TestDynamicsVector3D
    {
        [SetUp]
        public void Setup()
        {
        }

        /// <summary>
        ///  Ginsberg, Jerry.Engineering Dynamics(p. 7). Cambridge University Press. Kindle Edition.
        /// </summary>
        [Test]
        public void Test_Eq_1_1_23()
        {
            Vector3D A = new(1, 2, 3);
            Vector3D B = new(3, -1, -5);
            Vector3D C = A.CrossProduct(B);
            Vector3D e = C / C.Magnitude();
            Assert.AreEqual(e, C.UnitVector());
            double A1 = A.DotProduct(e);
            Assert.AreEqual(A1, 0);
            double B1 = B.DotProduct(e);
            Assert.AreEqual(B1, 0);
        }

        /// <summary>
        ///  Ginsberg, Jerry.Engineering Dynamics(p. 7). Cambridge University Press. Kindle Edition.
        /// </summary>
        [Test]
        public void Test_Ex_1_1()
        {
            double theta = 25 * Math.PI / 180;
            double gamma = 40 * Math.PI / 180;
            Vector3D r_ba = 2 * new Vector3D(Math.Cos(theta), Math.Sin(theta), 0);
            Assert.AreEqual(r_ba, new Vector3D(1.8126155740732999, 0.8452365234813989, 0));
            string componentRepresentation = r_ba.ComponentRepresentation();
            Assert.AreEqual(componentRepresentation, @"1.8126155740732999\hat{\textbf{i}}+0.8452365234813989\hat{\textbf{j}}");
            Vector3D r_cb = new(1.5 * Math.Cos(gamma) * Math.Cos(theta), 1.5 * Math.Cos(gamma) * Math.Sin(theta), 1.5 * Math.Sin(gamma));
            Assert.AreEqual(r_cb, new Vector3D(1.0414080660223257, 0.4856165564505969, 0.9641814145298089));
            Vector3D r_ca = r_ba + r_cb;
            Assert.AreEqual(r_ca, new Vector3D(2.8540236400956256, 1.3308530799319958, 0.9641814145298089));
            Vector3D r_dc = new Vector3D(3.5, 0, 0) - r_ca;
            Vector3D e_dc = r_dc.UnitVector();
            Assert.AreEqual(e_dc, new Vector3D(0.36582343488472086, -0.7536765665847329, -0.5460264164582086));
            Vector3D F = 5000 * e_dc;
            Assert.AreEqual(F, new Vector3D(1829.1171744236042, -3768.3828329236644, -2730.132082291043));
            double F_BC = F.DotProduct(r_cb.UnitVector());
            Assert.AreEqual(F_BC, -1704.9828856825459d);
            Vector3D M_A = r_ca.CrossProduct(F);
            Assert.AreEqual(M_A, new Vector3D(0, 9555.46228801865, -13189.339915232826));
            double M_AB = M_A.DotProduct(r_ba.UnitVector());
            Assert.AreEqual(M_AB, 4038.3128622912491d);
        }

        [Test]
        public void Test_Component_Representation()
        {
            Vector3D v = new Vector3D(1, 2, 3);
            Assert.AreEqual(v.ComponentRepresentation(), @"1\hat{\textbf{i}}+2\hat{\textbf{j}}+3\hat{\textbf{k}}");
        }
    }
}