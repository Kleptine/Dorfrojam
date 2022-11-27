using System;
using System.Collections.Generic;

namespace Utilities
{
    /// <summary>A collection of utilities for working with numbers, integers, modulos, etc.</summary>
    public static class NumUtils
    {
        /// <summary>Returns the nearest float to value that is a multiplier of factor.</summary>
        public static int NearestMultiple(int value, int factor)
        {
            return (int)Math.Round(value / (double)factor, MidpointRounding.AwayFromZero) * factor;
        }

        /// <summary>Returns the nearest float to value that is a multiplier of factor.</summary>
        public static float NearestMultiple(float value, float factor)
        {
            return (float)Math.Round(value / (double)factor, MidpointRounding.AwayFromZero) * factor;
        }

        /// <summary>Returns the first multiple of factor greater than value.</summary>
        public static int NextHighestMultiple(int value, int factor)
        {
            return (int)Math.Ceiling((double)value / factor) * factor;
        }

        /// <summary>Returns the first multiple of factor greater than value.</summary>
        public static float NextHighestMultiple(float value, float factor)
        {
            return (float)NextHighestMultiple(value, (double)factor);
        }

        /// <summary>Returns the first multiple of factor greater than value.</summary>
        public static double NextHighestMultiple(double value, double factor)
        {
            return Math.Ceiling(value / factor) * factor;
        }

        /// <summary>Clamps a value to be between the given boundaries, inclusive.</summary>
        /// <param name="value">Some value (-inf, +inf).</param>
        /// <param name="min">(-inf, +inf)</param>
        /// <param name="max">(-inf, +inf)</param>
        /// <returns>A value [min, max].</returns>
        public static float Clamp(float value, float min, float max)
        {
            return Math.Min(max, Math.Max(min, value));
        }

        /// <summary>Returns the distance between two values in modulo space.</summary>
        public static float DistanceInModulo(float value, float target, float modulo)
        {
            float diff = Math.Abs(value - target);
            return Math.Min(diff, modulo - diff);
        }

        /// <summary>Linearly maps a given value [0,1] to the range start..end.</summary>
        /// <param name="clamp">
        ///     Whether to force the final value to be within start/end or to allow the linear interpolation to
        ///     extend outside (if value > 1, for instance).
        /// </param>
        /// <returns></returns>
        public static float MapUnitToRange(float unitValue, float start, float end, bool clamp = false)
        {
            if (clamp)
            {
                unitValue = Clamp(unitValue, 0, 1);
            }

            return start + (end - start) * unitValue;
        }

        /// <summary>Takes a number (-inf..+inf) and two bounds and maps to 0..1 inside a given bounds.</summary>
        /// <param name="clamp">
        ///     Whether to force the final value to be within start/end or to allow the linear interpolation to
        ///     extend outside (if value > 1, for instance).
        /// </param>
        /// <returns></returns>
        public static float MapValueToUnit(float fullValue, float start, float end, bool clamp = false)
        {
            float result = (fullValue - start) / (end - start);
            if (clamp)
            {
                result = Clamp(result, 0, 1);
            }

            return result;
        }

        /// <summary>Maps a value linearly from one range to another. Example: 5, (0, 10), (4, 8) Result: 6</summary>
        /// <param name="clamp">Whether to clamp the values between the ranges.</param>
        public static float MapBetweenRanges(float value, float sourceRangeStart, float sourceRangeEnd,
                                             float destinationRangeStart, float destinationRangeEnd, bool clamp = true)
        {
            float unitValue = MapValueToUnit(value, sourceRangeStart, sourceRangeEnd, clamp);
            return MapUnitToRange(unitValue, destinationRangeStart, destinationRangeEnd, clamp);
        }

        /// <summary>Whether the given number is a power of 2.</summary>
        public static bool IsPowerOfTwo(int x)
        {
            return (x & (x - 1)) == 0;
        }

        /// <summary>Performs a canonical Modulus operation, where the output is on the range [0, b).</summary>
        public static int Mod(int value, int modulo)
        {
            int c = value % modulo;
            if ((c < 0 && modulo > 0) || (c > 0 && modulo < 0))
            {
                c += modulo;
            }

            return c;
        }

        /// <summary>
        ///     Returns the nearest value X to TargetValue, such that X = Value + M*Increment, for some number M. Equivalent
        ///     to adding/subtracting multiples of Increment from Value, until as close as possible to TargetValue.
        /// </summary>
        public static float NearestValueByIncrement(float value, float targetValue, float increment)
        {
            // Slightly modified version of this equation: https://stackoverflow.com/questions/29557459/round-to-nearest-multiple-of-a-number
            float number = targetValue - value;
            float normalized = (float)Math.Floor((number + increment / 2) / increment) * increment;
            float result = normalized + value;
            return result;
        }

        private static readonly List<float> cacheList = new List<float>();

        /// <summary>A simple, dumb median for small lists. Sorts the list and takes the middle.</summary>
        public static float Median(List<float> list)
        {
            cacheList.Clear();
            cacheList.AddRange(list);
            cacheList.Sort();

            if (cacheList.Count % 2 == 0)
            {
                // Even length array, average two middle elements.
                int lowerIdx = cacheList.Count / 2 - 1;
                int upperIdx = cacheList.Count / 2;
                return (cacheList[lowerIdx] + cacheList[upperIdx]) / 2f;
            }

            // Odd length is just the middle element.
            return cacheList[cacheList.Count / 2 - 1];
        }

        /// <summary>
        ///     Whether the difference between two floats is less than a value, epsilon. Similar to Mathf.Approximately, but
        ///     with a configurable threshold.
        /// </summary>
        public static bool Approximately(float a, float b, float epsilon)
        {
            if (epsilon <= 0)
            {
                throw new ArgumentException("Epsilon must be above zero");
            }

            return Math.Abs(a - b) <= epsilon;
        }
    }
}
