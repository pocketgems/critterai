﻿/*
 * Copyright (c) 2010 Stephen A. Pratt
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
using System;

namespace org.critterai.math
{
    /// <summary>
    /// Provides various math related utility operations.
    /// </summary>
    /// <remarks>
    /// <para>This class is optimized for speed.  To support this priority, no argument validation is
    /// performed.  E.g. No null checks, no divide by zero checks, etc.</para>
    /// <para>Static operations are thread safe.</para>
    /// </remarks>
    public static class MathUtil 
    {

        /// <summary>
        /// A standard epsilon value.  (Minimum positive value greater than zero.)
        /// </summary>
        public const float EPSILON_STD = 0.00001f;

        /// <summary>
        /// A standard tolerance value.
        /// </summary>
        public const float TOLERANCE_STD = 0.0001f;
        
        /// <summary>
        /// Determines whether the values are within the specified tolerance
        /// of each other. 
        /// </summary>
        /// <param name="a">The a-value to compare the b-value against.</param>
        /// <param name="b">The b-value to compare against the a-value.</param>
        /// <param name="tolerence">The tolerance to use for the comparison.</param>
        /// <returns>TRUE if the values are within the specified tolerance
        /// of each other.  Otherwise FALSE.</returns>
        public static Boolean SloppyEquals(float a, float b, float tolerence)
        {
            return !(b < a - tolerence || b > a + tolerence);
        }
        
        /// <summary>
        /// Clamps the value to a positive non-zero value.
        /// </summary>
        /// <param name="value">The value to clamp.</param>
        /// <returns>The value clamped to a minimum of the smallest possible positive value greater 
        /// than zero.</returns>
        public static float ClampToPositiveNonZero(float value)
        {
            return Math.Max(float.Epsilon, value);
        }
        
        /// <summary>
        /// Clamps the value to the specified range.  The clamp is inclusive
        /// of the minimum and maximum.
        /// </summary>
        /// <param name="value">The value to clamp.</param>
        /// <param name="minimum">The minimum allowed value.</param>
        /// <param name="maximum">The maximum allowed value.</param>
        /// <returns>A value clamped to the specified range.</returns>
        public static int Clamp(int value, int minimum, int maximum)
        {
            return (value < minimum ? minimum : (value > maximum ? maximum : value));
        }
        
        /// <summary>
        /// Returns the maximum value in the list of values.
        /// </summary>
        /// <param name="values">The values to search.</param>
        /// <returns>The maximum value in the list of values.</returns>
        public static float Max(params float[] values)
        {
            float result = values[0];
            for (int i = 1; i < values.Length; i++)
                result = Math.Max(result, values[i]);
            return result;
        }
        
        /// <summary>
        /// Returns the minimum value in the list of values.
        /// </summary>
        /// <param name="values">The values to search.</param>
        /// <returns>The minimum value in the list of values.</returns>
        public static float Min(params float[] values)
        {
            float result = values[0];
            for (int i = 1; i < values.Length; i++)
                result = Math.Min(result, values[i]);
            return result;
        }
    }
}