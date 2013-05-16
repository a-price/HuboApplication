/**
 * \file tf_eigen.cpp
 * \brief Converts between ROS TF and Eigen data structures for position and orientation.
 * Note: taken from http://www.ros.org/wiki/tf_conversions.
 * This functionality will be included in a later version of the core ROS.
 */

/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */



#include "hubo_vision/tf_eigen.h"

namespace tf 
{

  void VectorTFToEigen(const tf::Vector3& t, Eigen::Vector3d& k)
  {
	k(0) = t[0];
	k(1) = t[1];
	k(2) = t[2];
  };

void VectorEigenToTF(const Eigen::Vector3d& k, tf::Vector3& t)
{
  t[0] = k(0);
  t[1] = k(1);
  t[2] = k(2);
}

void RotationTFToEigen(const tf::Quaternion& t, Eigen::Quaterniond& k)
{
	Eigen::Quaterniond m(t[3],t[0],t[1],t[2]);
	k = m;
};

void RotationEigenToTF(const Eigen::Quaterniond& k, tf::Quaternion& t)
{
  t[0] = k.x();
  t[1] = k.y();
  t[2] = k.z();
  t[3] = k.w();
}

void TransformTFToEigen(const tf::Transform &t, Eigen::Affine3d &k)
{
	for(int i=0; i<3; i++)
	{
		k.matrix()(i,3) = t.getOrigin()[i];
		for(int j=0; j<3; j++)
		{
			k.matrix()(i,j) = t.getBasis()[i][j];
		}
	}
	// Fill in identity in last row
	for (int col = 0 ; col < 3; col ++)
		k.matrix()(3, col) = 0;
	k.matrix()(3,3) = 1;

};

void TransformTFToEigen(const tf::Transform &t, Eigen::Isometry3d &k)
{
	Eigen::Affine3d affine;
	TransformTFToEigen(t, affine);

	//k.translation() = affine.translation();
	//k.linear() = affine.rotation();
	k.matrix() = affine.matrix();
}

void TransformEigenToTF(const Eigen::Affine3d &k, tf::Transform &t)
{
	t.setOrigin(tf::Vector3(k.matrix()(0,3), k.matrix()(1,3), k.matrix()(2,3)));
	t.setBasis(tf::Matrix3x3(k.matrix()(0,0), k.matrix()(0,1),k.matrix()(0,2),k.matrix()(1,0), k.matrix()(1,1),k.matrix()(1,2),k.matrix()(2,0), k.matrix()(2,1),k.matrix()(2,2)));
};

void TransformEigenToTF(const Eigen::Isometry3d &k, tf::Transform &t)
{
	t.setOrigin(tf::Vector3(k.matrix()(0,3), k.matrix()(1,3), k.matrix()(2,3)));
	t.setBasis(tf::Matrix3x3(k.matrix()(0,0), k.matrix()(0,1),k.matrix()(0,2),k.matrix()(1,0), k.matrix()(1,1),k.matrix()(1,2),k.matrix()(2,0), k.matrix()(2,1),k.matrix()(2,2)));
};
}
