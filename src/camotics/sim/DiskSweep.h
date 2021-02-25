/******************************************************************************\

  CAMotics is an Open-Source simulation and CAM software.
  Copyright (C) 2011-2019 Joseph Coffland <joseph@cauldrondevelopment.com>
  Copyright (C) 2021 Hans Robeers <https://github.com/hrobeers>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

\******************************************************************************/

#pragma once


#include "Sweep.h"

namespace CAMotics {
  class DiskSweep : public Sweep {
    const double l;  // Length
    const double r;  // Radius
    const double r2; // Radius squared

  public:
    DiskSweep(double radius, double length);

    // From Sweep
    void getBBoxes(const cb::Vector3D &start, const cb::Vector3D &end,
                   std::vector<cb::Rectangle3D> &bboxes,
                   double tolerance) const;
    double depth(const cb::Vector3D &start, const cb::Vector3D &end,
               const cb::Vector3D &p) const;
  };
}
