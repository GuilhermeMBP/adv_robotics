// Copyright (C) 2026 Maik Knof
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef YASMIN_PCL__COMMON__PCL_COMPAT_HPP_
#define YASMIN_PCL__COMMON__PCL_COMPAT_HPP_

// pcl/types.h (providing pcl::Indices, pcl::IndicesPtr, etc.) was introduced
// in PCL 1.12.  Older releases that ship with ROS 2 Galactic / Foxy only
// have PCL 1.10 where these types live in pcl/pcl_base.h instead.

#include <pcl/pcl_config.h>

#if PCL_MAJOR_VERSION > 1 || (PCL_MAJOR_VERSION == 1 && PCL_MINOR_VERSION >= 12)
#include <pcl/types.h>
#else
#include <pcl/pcl_base.h>
#endif

#endif // YASMIN_PCL__COMMON__PCL_COMPAT_HPP_
