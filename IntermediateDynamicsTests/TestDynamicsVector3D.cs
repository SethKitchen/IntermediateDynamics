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
            Assert.Less(Math.Abs(r_cb.X - 1.0414080660223257), 0.001);
            Assert.Less(Math.Abs(r_cb.Y - 0.4856165564505969), 0.001);
            Assert.Less(Math.Abs(r_cb.Z- 0.9641814145298089), 0.001);
            Vector3D r_ca = r_ba + r_cb;
            Assert.Less(Math.Abs(r_ca.X - 2.8540236400956256), 0.001);
            Assert.Less(Math.Abs(r_ca.Y - 1.3308530799319958), 0.001);
            Assert.Less(Math.Abs(r_ca.Z - 0.9641814145298089), 0.001);
            Vector3D r_dc = new Vector3D(3.5, 0, 0) - r_ca;
            Vector3D e_dc = r_dc.UnitVector();
            Assert.Less(Math.Abs(e_dc.X - 0.36582343488472086), 0.001);
            Assert.Less(Math.Abs(e_dc.Y + 0.7536765665847329), 0.001);
            Assert.Less(Math.Abs(e_dc.Z + 0.5460264164582086), 0.001);
            Vector3D F = 5000 * e_dc;
            Assert.Less(Math.Abs(F.X - 1829.1171744236042), 0.001);
            Assert.Less(Math.Abs(F.Y + 3768.3828329236644), 0.001);
            Assert.Less(Math.Abs(F.Z + 2730.132082291043), 0.001);
            double F_BC = F.DotProduct(r_cb.UnitVector());
            Assert.Less(Math.Abs(F_BC + 1704.9828856825459), 0.001);
            Vector3D M_A = r_ca.CrossProduct(F);
            Assert.Less(Math.Abs(M_A.X - 0), 0.001);
            Assert.Less(Math.Abs(M_A.Y - 9555.46228801865), 0.001);
            Assert.Less(Math.Abs(M_A.Z + 13189.339915232826), 0.001);
            double M_AB = M_A.DotProduct(r_ba.UnitVector());
            Assert.Less(Math.Abs(M_AB - 4038.3128622912491), 0.001);
        }

        [Test]
        public void Test_Component_Representation()
        {
            Vector3D v = new Vector3D(1, 2, 3);
            Assert.AreEqual(v.ComponentRepresentation(), @"1\hat{\textbf{i}}+2\hat{\textbf{j}}+3\hat{\textbf{k}}");
        }
    }
}