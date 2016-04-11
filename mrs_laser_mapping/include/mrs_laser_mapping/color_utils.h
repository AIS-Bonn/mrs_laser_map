/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Computer Science Institute VI, University of Bonn
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of University of Bonn, Computer Science Institute
 *     VI nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _COLOR_UTILS_H_
#define _COLOR_UTILS_H_

namespace mrs_laser_mapping
{
class ColorMapJet
{
public:
  static inline double red(double gray)
  {
    return base(gray - 0.5);
  }
  static inline double green(double gray)
  {
    return base(gray);
  }
  static inline double blue(double gray)
  {
    return base(gray + 0.5);
  }

  static inline double base(double val)
  {
    if (val <= -0.75)
      return 0;
    else if (val <= -0.25)
      return interpolate(val, 0.0, -0.75, 1.0, -0.25);
    else if (val <= 0.25)
      return 1.0;
    else if (val <= 0.75)
      return interpolate(val, 1.0, 0.25, 0.0, 0.75);
    else
      return 0.0;
  }

  static void getRainbowColor(float value, float& r, float& g, float& b)
  {
    // this is HSV color palette with hue values going only from 0.0 to 0.833333.
    value = std::min(value, 1.0f);
    value = std::max(value, 0.0f);

    float color[3];

    float h = value * 5.0f + 1.0f;
    int i = floor(h);
    float f = h - i;
    if (!(i & 1))
      f = 1 - f;  // if i is even
    float n = 1 - f;

    if (i <= 1)
      color[0] = n, color[1] = 0, color[2] = 1;
    else if (i == 2)
      color[0] = 0, color[1] = n, color[2] = 1;
    else if (i == 3)
      color[0] = 0, color[1] = 1, color[2] = n;
    else if (i == 4)
      color[0] = n, color[1] = 1, color[2] = 0;
    else if (i >= 5)
      color[0] = 1, color[1] = n, color[2] = 0;

    r = color[0];
    g = color[1];
    b = color[2];
  }

private:
  static inline double interpolate(double val, double y0, double x0, double y1, double x1)
  {
    return (val - x0) * (y1 - y0) / (x1 - x0) + y0;
  }
};
}

#endif
