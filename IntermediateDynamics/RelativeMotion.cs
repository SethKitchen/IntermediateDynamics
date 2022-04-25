using MathNet.Numerics.Integration;
using MathNet.Spatial.Euclidean;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Expr = MathNet.Symbolics.SymbolicExpression;

namespace IntermediateDynamics
{
    public class RelativeMotion
    {
        public Vector3D YTransformation(double angleInRadiansBetweenOldXAndNewyAxis, double angleInRadiansBetweenOldZAndNewyAxis)
        {
            return new Vector3D(Math.Cos(angleInRadiansBetweenOldXAndNewyAxis), 0, Math.Sin(angleInRadiansBetweenOldZAndNewyAxis));
        }
    }
}
