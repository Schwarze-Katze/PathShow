#pragma once

#include <boost/functional/hash.hpp>
#include <boost/numeric/ublas/matrix.hpp>

//###################################################
// 类                        State
//###################################################
struct State
{
  State(double x, double y, double z, int time = 0)
      : time(time), x(x), y(y), z(z){}

  State() = default;
  bool operator==(const State &s) const
  {
    return std::tie(time, x, y, z) == std::tie(s.time, s.x, s.y, s.z);
  }

  bool except_time(const State &s)
  {
    return std::tie(x, y, z) == std::tie(s.x, s.y, s.z);
  }

  friend std::ostream &operator<<(std::ostream &os, const State &s)
  {
    return os << "(" << s.x << "," << s.y << ":" << s.z << ")" << ")@" << s.time<< std::endl ;
  }
  
  int time;
  double x;
  double y;
  double z;

  // Location location;
  // boost::numeric::ublas::matrix<double> rot;
};

namespace std
{
  template <>
  struct hash<State>
  {
    size_t operator()(const State &s) const
    {
      size_t seed = 0;
      boost::hash_combine(seed, s.time);
      boost::hash_combine(seed, s.x);
      boost::hash_combine(seed, s.y);
      boost::hash_combine(seed, s.z);
      return seed;
    }
  };
} // namespace std

namespace geometry_msgs
{
  std::size_t hash_value(Pose const& pos) {
    std::size_t seed = 0;
    boost::hash_combine(seed, pos.position.x);
    boost::hash_combine(seed, pos.position.y);
    boost::hash_combine(seed, pos.position.z);
    return seed;
  }
} // namespace geometry_msgs
