// Copyright 2022 Fraunhofer FKIE - All Rights Reserved
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ABC_STRUCTS_H_
#define ABC_STRUCTS_H_

#include <algorithm>
#include <deque>
#include <iostream>
#include <map>
#include <string>
#include <unordered_map>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

typedef std::string vertex_t;  // represent the unique hash string id of a vertex in the graph

typedef double weight_t;  // represent the cost of moving on a search graph planner

const double inf_double = std::numeric_limits<double>::max();
const float inf_float = std::numeric_limits<float>::max();
const int inf_int = std::numeric_limits<int>::max();
const weight_t max_weight = std::numeric_limits<double>::infinity();

/**
 * @brief Storage 3D indexes (x, y, z)
 */
struct IndexGrid
{
  int x = inf_int;
  int y = inf_int;
  int z = inf_int;

  IndexGrid() : x(inf_int), y(inf_int), z(inf_int){};
  IndexGrid(int x, int y, int z) : x(x), y(y), z(z){};

  bool operator==(const IndexGrid& other) const
  {
    return (x == other.x && y == other.y && z == other.z);
  }

  bool operator!=(const IndexGrid& other) const
  {
    return !(x == other.x && y == other.y && z == other.z);
  }

  IndexGrid operator+(const IndexGrid& other) const
  {
    IndexGrid sum;
    sum.x = x + other.x;
    sum.y = y + other.y;
    sum.z = z + other.z;
    return sum;
  }

  IndexGrid operator-(const IndexGrid& other) const
  {
    IndexGrid sum;
    sum.x = x - other.x;
    sum.y = y - other.y;
    sum.z = z - other.z;
    return sum;
  }

  std::string toString() const
  {
    return std::string("x: " + std::to_string(x) + " y: " + std::to_string(y) + " z: " + std::to_string(z));
  }
};

/**
 * @brief Hasher for [IndexGrid] in 3D
 */
struct IndexGridHasher
{
  std::size_t operator()(const IndexGrid& k) const
  {
    return ((std::hash<double>()(k.x) ^ (std::hash<double>()(k.y) << 1)) >> 1) ^ (std::hash<double>()(k.z) << 1);
  }
};

/**
 * @brief Storage 3D points (x, y, z)
 */
struct PositionGrid
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  PositionGrid() : x(inf_double), y(inf_double), z(0.0){};
  PositionGrid(double x, double y) : x(x), y(y), z(0.0){};
  PositionGrid(double x, double y, double z) : x(x), y(y), z(z){};

  bool operator==(const PositionGrid& other) const
  {
    return (x == other.x && y == other.y && z == other.z);
  }

  bool operator!=(const PositionGrid& other) const
  {
    return !(x == other.x && y == other.y && z == other.z);
  }

  bool operator<(const PositionGrid& other) const
  {
    return (x < other.x) && (y < other.y) && (z < other.z);
  }

  PositionGrid operator+(const PositionGrid& other) const
  {
    PositionGrid sum;
    sum.x = x + other.x;
    sum.y = y + other.y;
    sum.z = z + other.z;
    return sum;
  }

  PositionGrid operator-(const PositionGrid& other) const
  {
    PositionGrid sum;
    sum.x = x - other.x;
    sum.y = y - other.y;
    sum.z = z - other.z;
    return sum;
  }

  std::string toString() const
  {
    return std::string("x: " + std::to_string(x) + " y: " + std::to_string(y) + " z: " + std::to_string(z));
  }

  geometry_msgs::PoseStamped toPoseStamped(const std::string frame_id) const
  {
    geometry_msgs::PoseStamped p;
    p.header.stamp = ros::Time::now();
    p.header.frame_id = frame_id;
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.position.z = z;
    p.pose.orientation.w = 1.0;
    return p;
  }
};

/**
 * @brief Hasher for [PositionGrid] in 2D
 */
struct PositionGridHasher
{
  std::size_t operator()(const PositionGrid& k) const
  {
    return ((std::hash<double>()(k.x) ^ (std::hash<double>()(k.y) << 1)) >> 1) ^ (std::hash<double>()(k.z) << 1);
  }
};

typedef std::string ValueType;
typedef std::string ValueSource;

struct ValueData
{
  ValueType type;
  ValueSource source;
  double value;
};

struct MeasurementData
{
  PositionGrid position;
  ros::Time stamp;
  std::vector<ValueData> values;
};

struct MeasurementStatistics
{
  double max = -inf_double;
  double min = inf_double;
};

struct Measurements
{
  std::vector<MeasurementData> data;
  std::unordered_map<ValueType, MeasurementStatistics> stats;
};

#endif /* ABC_STRUCTS_H_ */