using IntermediateDynamics;
using NUnit.Framework;
using System;
using System.Collections.Generic;
using MathNet.Spatial.Euclidean;
using Expr = MathNet.Symbolics.SymbolicExpression;
using MathNet.Symbolics;

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

            Vector3D e_b = ParticleKinematics.SolveForBiNormalUnitVector(e_ba, e_n);

            Assert.AreEqual(e_b, new Vector3D(-8.326672684688674E-17, 0.8733693623277108, -0.4870584738481488));

            double externalTangentialForces = ParticleKinematics.SolveForExternalTangentialForces(e_ba, 5, v_dot);

            Assert.AreEqual(externalTangentialForces, 291.06869244870018);

            double externalNormalForces = ParticleKinematics.SolveForExternalNormalForces(e_n, 5, 500, rho);

            Assert.AreEqual(externalNormalForces, 339.91574254246422);

            double externalBiNormalForces = ParticleKinematics.SolveForExternalBiNormalForces(e_b, 5);

            Assert.AreEqual(externalBiNormalForces, -23.861029201124598);
        }

        /// <summary>
        ///  Ginsberg, Jerry.Engineering Dynamics(p. 41). Cambridge University Press. Kindle Edition.
        /// </summary>
        [Test]
        public void Test_Ex_2_3()
        {
            var x = Expr.Parse("0.2*B*cos(B)");
            var xPrime = x.Differentiate("B");
            var xDoublePrime = xPrime.Differentiate("B");

            var y = Expr.Parse("0.2*B*sin(B)");
            var yPrime = y.Differentiate("B");
            var yDoublePrime = yPrime.Differentiate("B");

            var z = Expr.Parse("0.1*B^2");
            var zPrime = z.Differentiate("B");
            var zDoublePrime = zPrime.Differentiate("B");

            var s = Expr.Parse("10*t^2");
            var symbols = new Dictionary<string, FloatingPoint> { { "t", 0.5 } };
            var sAtTime = s.Evaluate(symbols);
            var sPrime = ParticleKinematics.SolveForRBarPrimeFromFirstDerivativeComponents(xPrime, yPrime, zPrime);
            var sPrimeSimplified = sPrime.TrigonometricSimplify().ExponentialSimplify();

            Assert.AreEqual(sPrimeSimplified.ToString(), "(0.04000000000000001 + 0.08000000000000002*B^2)^0.5");

            // Math.symbolics cannot integrate yet, and Numerics must be in Func<double, double> form
            // We have to manually generate the function found above sPrimeSimplified.
            static double function_to_integrate(double x) => Math.Pow(0.04 + 0.08 * (x * x), 0.5);

            double betaPrime = ParticleKinematics.RootFindParametric(sAtTime.RealValue, 0, 10, function_to_integrate, 0.005);

            Assert.AreEqual(Math.Round(betaPrime, 2), 4.03);

            var symbols2 = new Dictionary<string, FloatingPoint> { { "B", betaPrime } };
            var rPrimeAtTime = new Vector3D(xPrime.Evaluate(symbols2).RealValue, yPrime.Evaluate(symbols2).RealValue, zPrime.Evaluate(symbols2).RealValue);
            var rDoublePrimeAtTime = new Vector3D(xDoublePrime.Evaluate(symbols2).RealValue, yDoublePrime.Evaluate(symbols2).RealValue, zDoublePrime.Evaluate(symbols2).RealValue);
            var sPrimeAtTime = sPrime.Evaluate(symbols2);

            Assert.AreEqual(rPrimeAtTime, new Vector3D(0.49938130485460674, -0.6635166255352589, 0.8059999999999917));
            Assert.AreEqual(rDoublePrimeAtTime, new Vector3D(0.8187302909529202, 0.37325153807604516, 0.2));
            Assert.AreEqual(sPrimeAtTime.RealValue, 1.1572691994518707d);

            var e_t = ParticleKinematics.GetTangentUnitVector(rPrimeAtTime, sPrimeAtTime.RealValue);
            Assert.AreEqual(e_t, new Vector3D(0.43151697555861146, -0.5733468287668306, 0.696467166309918));

            var e_n = ParticleKinematics.GetNormalUnitVector(rPrimeAtTime, rDoublePrimeAtTime, sPrimeAtTime.RealValue);
            Assert.AreEqual(e_n, new Vector3D(0.7949872407742137, 0.6065880554709596, 0.0067983796718306625));

            var rho = ParticleKinematics.GetRadiusOfCurvatureParametric(rPrimeAtTime, rDoublePrimeAtTime, sPrimeAtTime.RealValue);
            Assert.AreEqual(rho, 1.5242387788870211d);

            var v = s.Differentiate("t");
            var vAtTime = v.Evaluate(symbols);
            var vDot = v.Differentiate("t");
            var vDotAtTime = vDot.Evaluate(symbols);

            var v_bar = vAtTime.RealValue * e_t;
            Assert.AreEqual(v_bar, new Vector3D(4.315169755586115, -5.733468287668306, 6.96467166309918));
            var a_bar = vDotAtTime.RealValue * e_t + vAtTime.RealValue * vAtTime.RealValue / rho * e_n;
            Assert.AreEqual(a_bar, new Vector3D(60.786684815200175, 28.329194048887643, 14.375361349489616));
        }

        /// <summary>
        ///  Ginsberg, Jerry.Engineering Dynamics(p. 47). Cambridge University Press. Kindle Edition.
        /// </summary>
        [Test]
        public void Test_Ex_2_4()
        {
            var resultant_force = new VectorExpr3D(Expr.Zero, Expr.Parse("(1.6-4*y)*10^(-3)"), Expr.Parse("" + (10 * Constants.GRAVITATIONAL_ACCELERATION_ON_EARTH * 1e-6)));
            var a_bar = resultant_force / (10 * 1e-6);
            a_bar.Y = a_bar.Y.ExponentialSimplify();
            Assert.AreEqual(a_bar.X.ToString(), Expr.Zero.ToString());
            Assert.AreEqual(a_bar.Y.ToString(), Expr.Parse("160.00000000000003 - 400.00000000000006*y").ToString());
            Assert.AreEqual(a_bar.Z.ToString(), Expr.Parse("" + Constants.GRAVITATIONAL_ACCELERATION_ON_EARTH).ToString());
            var v_bar_0 = new Vector3D(20 * Math.Cos(45 * Math.PI / 180) * Math.Cos(20 * Math.PI / 180), -20 * Math.Cos(45 * Math.PI / 180) * Math.Sin(20 * Math.PI / 180), 20 * Math.Sin(45 * Math.PI / 180));
            Assert.AreEqual(v_bar_0, new Vector3D(13.289260487773495, -4.836895252959505, 14.142135623730951));

            ////////////////////////////////////////////////// Symbolics doesn't integrate yet
            // integral of x_double_dot=0 is x_dot = c_1
            // Double integral of x_double_dot=0 is x=c_1*t+c_2
            // 13.289 = c_1
            // 0 = c_1*0+c_2 means c_2 = 0
            var x = Expr.Parse("13.289260487773495*t");

            // double integral of y_double_dot is y=c_5*cos(20*t)+c_6*sin(20*t)+0.4
            // derivative of this is y_dot = -20*c_5*sin(20*t)+20*c_6*cos(20*t)
            // c_5 = -0.4
            // -4.836895252959505 = 20*c_6
            var y = Expr.Parse("-0.4*cos(20*t)-.241*sin(20*t)+0.4");

            // integral of z_double_dot=-9.807 is z_dot = -9.807*t + c3
            // double integral of z_double_dot is z = -9.807t^2/2+c3*t+c4
            // 14.142135623730951 = c3
            // 0 = c4
            var z = Expr.Parse("-4.9035*t^2+14.1421*t");

            var t_f = Util.SolveForVariable(0, 10, "t", 0.4, .01, y);
            Assert.AreEqual(t_f, 0.10500000000000008);

            var symbols = new Dictionary<string, FloatingPoint> { { "t", t_f } };

            var xAns = x.Evaluate(symbols);
            Assert.AreEqual(xAns.RealValue, 1.3953723512162179);

            var yAns = y.Evaluate(symbols);
            Assert.AreEqual(yAns.RealValue, 0.39390498447756506);

            var zAns = z.Evaluate(symbols);
            Assert.AreEqual(zAns.RealValue, 1.4308594125000009);

            var xDotAns = x.Differentiate("t").Evaluate(symbols);
            Assert.AreEqual(xDotAns.RealValue, 13.289260487773495);

            var yDotAns = y.Differentiate("t").Evaluate(symbols);
            Assert.AreEqual(yDotAns.RealValue, 9.3390331573623033);

            var zDotAns = z.Differentiate("t").Evaluate(symbols);
            Assert.AreEqual(zDotAns.RealValue, 13.112364999999999);
        }
    }
}
