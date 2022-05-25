using IntermediateDynamics;
using NUnit.Framework;
using System;
using MathNet.Spatial.Euclidean;

namespace IntermediateDynamicsTests
{
    class TestRelativeMotion
    {
        /// <summary>
        ///  Ginsberg, Jerry.Engineering Dynamics(p. 105). Cambridge University Press. Kindle Edition.
        /// </summary>
        [Test]
        public void Test_Ex_3_3()
        {
            var R1 = RelativeMotion.GetRotationMatrixYAxis(65 * Math.PI / 180);
            var R2 = RelativeMotion.GetRotationMatrixZAxis(-145 * Math.PI / 180);
            var R = R1 * R2;
            Assert.AreEqual(R[0, 0], -0.34618861305875404);
            Assert.AreEqual(R[0, 1], -0.24240387650610415);
            Assert.AreEqual(R[0, 2], -0.90630778703664994);
            Assert.AreEqual(R[1, 0], 0.57357643635104638d);
            Assert.AreEqual(R[1, 1], -0.81915204428899158);
            Assert.AreEqual(R[1, 2], 0);
            Assert.AreEqual(R[2, 0], -0.74240387650610384);
            Assert.AreEqual(R[2, 1], -0.51983679072568478);
            Assert.AreEqual(R[2, 2], 0.42261826174069944d);

            Vector3D point1InXYZ = new Vector3D(2, -4, 3);
            Vector3D point1InXyz = (R * point1InXYZ.GetMatrix()).FromMatrix();
            Assert.AreEqual(point1InXyz.X, -2.4416850812030413);
            Assert.AreEqual(point1InXyz.Y, 4.4237610498580588);
            Assert.AreEqual(point1InXyz.Z, 1.8623941951126297);

            Vector3D point2InXyz = new Vector3D(2, -4, 3);
            Vector3D point2InXYZ = (R.Transpose() * point2InXyz.GetMatrix()).FromMatrix();
            Assert.AreEqual(point2InXYZ.X, -5.2138946010400051);
            Assert.AreEqual(point2InXYZ.Y, 1.2322900519667037);
            Assert.AreEqual(point2InXYZ.Z, -0.5447607888512016);
        }
    }
}
