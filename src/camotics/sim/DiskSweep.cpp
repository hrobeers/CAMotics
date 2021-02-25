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

#include "DiskSweep.h"

#include <cbang/log/Logger.h>

#include <limits>

using namespace std;
using namespace cb;
using namespace CAMotics;


namespace {
  inline double sqr(double x) {return x * x;}
}


DiskSweep::DiskSweep(double radius, double length) :
  l(length), r(radius), r2(sqr(radius)) {
}


void DiskSweep::getBBoxes(const Vector3D &start, const Vector3D &end,
                           vector<Rectangle3D> &bboxes,
                           double tolerance) const {

  const unsigned maxLen = l*8;
  double len = start.distance(end);
  unsigned steps = (len <= maxLen) ? 1 : (len / maxLen);
  double stride = 1.0 / steps;
  Vector3D p1 = start;
  Vector3D p2;

  for (unsigned i = 0; i < steps; i++) {
    for (unsigned j = 0; j < 3; j++)
      p2[j] = start[j] + (end[j] - start[j]) * stride * (i + 1);

    double minX = std::min(p1.x(), p2.x()) - (l/2) - tolerance;
    double minY = std::min(p1.y(), p2.y()) - r - tolerance;
    double minZ = std::min(p1.z(), p2.z()) - r - tolerance;
    double maxX = std::max(p1.x(), p2.x()) + (l/2) + tolerance;
    double maxY = std::max(p1.y(), p2.y()) + r + tolerance;
    double maxZ = std::max(p1.z(), p2.z()) + r + tolerance;

    bboxes.push_back
      (Rectangle3D(Vector3D(minX, minY, minZ),
                       Vector3D(maxX, maxY, maxZ)));

    p1 = p2;
  }
}


double DiskSweep::depth(const Vector3D &A, const Vector3D &B,
                         const Vector3D &P) const {
// A = startpoint of move to eval
// B = endpoint of move to eval
// P = point to eval

  const double Ax = A.x(), Ay = A.y(), Az = A.z();
  const double Bx = B.x(), By = B.y(), Bz = B.z();
  const double Px = P.x(), Py = P.y(), Pz = P.z();

  // Check x-axis (disk rotates around X)
  if (Px < min(Ax, Bx) - l/2 || max(Ax, Bx) + l/2 < Px) return -1;

  // Distance factor A to B
  auto d = P-A;
  double fd = (B-A).dot(d)/A.distanceSquared(B);

  // Disk center to eval
  auto Pd = A + (B-A)*fd;

  // Check if distance to Pd is smaller than radius
  if (Pd.distanceSquared(P) < r2)
    return 1;

  return -1;
}
