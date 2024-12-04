// SPDX-FileCopyrightText: 2024 MakotoYoshigoe 
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCOMARKERLOCALIZATION__POSE_HPP_
#define ARUCOMARKERLOCALIZATION__POSE_HPP_

namespace ArucoMarkerLocalization{
class Pose{
	public:
	Pose();
	Pose(double x, double y, double t);
	~Pose();
	void angleNomalization();
	void setPose(double x, double y, double t);
	//Pose operator=(const Pose & p);
	Pose operator-(const Pose & p) const;
	Pose operator+(const Pose & p) const;

	double x_;
	double y_;
	double t_;
};
}

#endif
