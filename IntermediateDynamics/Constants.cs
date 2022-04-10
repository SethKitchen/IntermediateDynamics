using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace IntermediateDynamics
{
    public class Constants
    {
        /// <summary>
        /// m^3/kg*s^2
        /// Ginsberg, Jerry.Engineering Dynamics(pp. 16). Cambridge University Press. Kindle Edition.
        /// </summary>
        public const double UNIVERSAL_GRAVITATIONAL_CONSTANT_G = 6.67408e-11;

        /// <summary>
        /// kg
        /// Ginsberg, Jerry.Engineering Dynamics(pp. 16). Cambridge University Press. Kindle Edition.
        /// </summary>
        public const double MASS_OF_EARTH = 5.9722e24;

        /// <summary>
        /// m
        /// Ginsberg, Jerry.Engineering Dynamics(pp. 16). Cambridge University Press. Kindle Edition.
        /// </summary>
        public const double RADIUS_OF_EARTH = 6378137;

        /// <summary>
        /// m/s^2
        /// Ginsberg, Jerry.Engineering Dynamics(pp. 16). Cambridge University Press. Kindle Edition.
        /// </summary>
        public const double GRAVITATIONAL_ACCELERATION_ON_EARTH = UNIVERSAL_GRAVITATIONAL_CONSTANT_G * MASS_OF_EARTH / (RADIUS_OF_EARTH * RADIUS_OF_EARTH);
    }
}
