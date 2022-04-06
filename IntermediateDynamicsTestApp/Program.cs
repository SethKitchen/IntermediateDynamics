using MathNet.Spatial.Euclidean;
using System;
using IntermediateDynamics;

namespace IntermediateDynamicsTestApp
{
    class Program
    {
        static void Main(string[] args)
        {
            String ans = (new Vector3D(2, 4, 5)).ComponentRepresentation();
            Console.WriteLine(ans);
        }
    }
}
