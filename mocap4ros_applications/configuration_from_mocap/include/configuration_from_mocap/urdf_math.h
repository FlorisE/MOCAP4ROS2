// Copyright 2020 National Institute of Advanced Industrial Science and Technology, Japan
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
//
// Author: Floris Erich <floris.erich@aist.go.jp>

#ifndef URDF_MATH_H
#define URDF_MATH_H

#include <urdf/model.h>

/** Cross product of two urdf vectors (left x right)
 * @param left Left vector to be crossed
 * @param right Right vector to be crossed
 */
urdf::Vector3 cross( const urdf::Vector3& left, const urdf::Vector3& right )
{
  urdf::Vector3 result;
  result.x = left.y * right.z - left.z * right.y;
  result.y = left.z * right.x - left.x * right.z;
  result.z = left.x * right.y - left.y * right.x;
  return result;
}

/** The norm (length) of a vector
 * @param vec The vector to get the norm of
 */
double norm( const urdf::Vector3& vec )
{
  return sqrt( vec.x * vec.x + vec.y * vec.y + vec.z * vec.z );
}

/** Normalize the vector 
 * @param vec The vector to be normalized
 */
urdf::Vector3 normalize( const urdf::Vector3& vec)
{
  double vecNorm = norm( vec );
  return urdf::Vector3( vec.x / vecNorm, vec.y / vecNorm, vec.z / vecNorm );
}

/** The dot product of two vectors
 * @param left Left vector to be dotted
 * @param right Right vector to be dotted
 */
double dot( const urdf::Vector3& left, const urdf::Vector3& right )
{
  return left.x * right.x + left.y * right.y + left.z * right.z;
}

/** The dot product of two rotations (quaternions) gives the rotation angle between them
 * @param left Left rotation to be dotted
 * @param right Right rotation to be dotted
 */
double dot( const urdf::Rotation& left, const urdf::Rotation& right )
{
  return left.x * right.x + left.y * right.y + left.z * right.z + left.w * right.w;
}

/** Scaling a rotation (d * rot)
 * @param d Scaling factor
 * @param rot Rotation to be scaled
 */
urdf::Rotation operator*( double d, urdf::Rotation rot )
{
  rot.x = d * rot.x;
  rot.y = d * rot.y;
  rot.z = d * rot.z;
  rot.w = d * rot.w;
  return rot;
}

/** Scaling a rotation (d * rot)
 * @param rot Rotation to be scaled
 * @param d Scaling factor
 */
urdf::Rotation operator*( urdf::Rotation rot, double d )
{
  rot.x = d * rot.x;
  rot.y = d * rot.y;
  rot.z = d * rot.z;
  rot.w = d * rot.w;
  return rot;
}

/** Scaling a rotation (rot = d * rot)
 * @param rot Rotation to be scaled
 * @param d Scaling factor
 */
void operator*=( urdf::Rotation& rot, double d )
{
  rot = d * rot;
} 

/** Scaling a rotation (rot = d * rot)
 * @param d Scaling factor
 * @param rot Rotation to be scaled
 */
void operator*=( double d, urdf::Rotation& rot )
{
  rot = rot * d;
}

/** Spherical linear interpolation of two rotation quaternions based on https://en.wikipedia.org/wiki/Slerp
 * @param rot1 First rotation quaternion
 * @param rot2 Second rotation quaternion
 * @param t The weight of interpolation
 */
urdf::Rotation slerp( urdf::Rotation rot1, urdf::Rotation rot2, double t )
{
  rot1.normalize();
  rot2.normalize();

  double dotted = dot( rot1, rot2 );

  if ( dotted < 0.0f )
  {
    rot1 = rot1.GetInverse();
    dotted = -dotted;
  }

  const double ROT_THRESHOLD = 0.9995;
  if ( dotted > ROT_THRESHOLD )
  {
    urdf::Rotation result = rot1 * (t * (rot2 * rot1.GetInverse()));
    result.normalize();
    return result;
  }

  double theta_0 = acos( dotted );
  double theta = theta_0 * t;
  double sin_theta = sin( theta );
  double sin_theta_0 = sin( theta_0 );

  double s0 = cos( theta ) - dotted * sin_theta / sin_theta_0;
  double s1 = sin_theta / sin_theta_0;

  return (s0 * rot1) * (s1 * rot2);
}

/** "Blend" a set of rotation quaternions.
 * Average of two quaternions, but with more than two this does not provide a mathematical correct result
 * @param rotations Rotations to be "blended"
 * @return New rotation that is the blended version of the inputs
 */
urdf::Rotation blend( const std::vector<std::shared_ptr<urdf::Rotation>>& rotations )
{
  if (rotations.size() == 0)
  {
    return urdf::Rotation();
  }
  else if (rotations.size() == 1)
  {
    return *(rotations[0]);
  }
  else
  {
    urdf::Rotation res;
    double roll_avg, pitch_avg, yaw_avg;
    double roll_tmp, pitch_tmp, yaw_tmp;
    int n = rotations.size();
    for (int i = 0; i < n; ++i)
    {
      (*rotations[i]).getRPY(roll_tmp, pitch_tmp, yaw_tmp);
      roll_avg += roll_tmp / n;
      pitch_avg += pitch_tmp / n;
      yaw_avg += yaw_tmp / n;
    }
    res.setFromRPY(roll_avg, pitch_avg, yaw_avg);
    return res;
  }
}

/** Scale the vector
 * @param v Vector to be scaled
 * @param s Number to be scaled by (inverse of)
 * @return New vector that is the scaled vector
 */
urdf::Vector3 operator/( const urdf::Vector3& v, double s )
{
  return urdf::Vector3( v.x / s, v.y / s, v.z / s );
}

/** Substract right from left vector (left - right)
 * @param left Vector to be substracted from
 * @param right Subtraction vector
 * @return New vector that is the result of the subtraction
 */
urdf::Vector3 operator-( const urdf::Vector3& left, const urdf::Vector3& right )
{
  return urdf::Vector3( left.x - right.x, left.y - right.y, left.z - right.z );
}

/** Vector sum of left and right vectors
 * Required because the default + operator on urdf::Vector3 doesn't take const arguments
 * @param left Left part of the sum
 * @param right Right part of the sum
 * @return A new vector that is the sum of the left and right vector
 */
urdf::Vector3 operator+( const urdf::Vector3& left, const urdf::Vector3& right )
{
  return urdf::Vector3( left.x + right.x, left.y + right.y, left.z + right.z );
}

urdf::Vector3 operator+=( urdf::Vector3& left, const urdf::Vector3& right )
{
  left.x += right.x;
  left.y += right.y;
  left.z += right.z;
  return left;
}

geometry_msgs::msg::Vector3 urdf_to_geometry_msg( const urdf::Vector3& v )
{
  geometry_msgs::msg::Vector3 msg;
  msg.x = v.x;
  msg.y = v.y;
  msg.z = v.z;
  return msg;
}

urdf::Vector3 geometry_msgs_translation_to_urdf( const geometry_msgs::msg::Vector3& v )
{
  return urdf::Vector3( v.x, v.y, v.z );
}

geometry_msgs::msg::Quaternion urdf_to_geometry_msg( const urdf::Rotation& r )
{
  geometry_msgs::msg::Quaternion msg;
  msg.w = r.w;
  msg.x = r.x;
  msg.y = r.y;
  msg.z = r.z;
  return msg;
}

urdf::Rotation geometry_msgs_quaternion_to_urdf( const geometry_msgs::msg::Quaternion q )
{
  urdf::Rotation r;
  r.w = q.w;
  r.x = q.x;
  r.y = q.y;
  r.z = q.z;
  return r;
}

#endif
