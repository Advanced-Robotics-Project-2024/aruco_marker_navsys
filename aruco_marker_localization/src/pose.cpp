// SPDX-FileCopyrightText: 2024 MakotoYoshigoe 
// SPDX-License-Identifier: Apache-2.0

#include<aruco_marker_localization/pose.hpp>
#include<cmath>

namespace ArucoMarkerLocalization{
	Pose::Pose(){}
	Pose::Pose(double x, double y, double t)
	{
		setPose(x, y, t);
	}

	Pose::~Pose(){}
	
    void Pose::angleNomalization()
    {
        while(t_>M_PI) t_ -= 2*M_PI;
        while(t_<M_PI) t_ += 2*M_PI;
    }

	void Pose::setPose(double x, double y, double t)
	{
		x_ = x;
		y_ = y;
		t_ = t;
	}

	Pose Pose::operator-(const Pose & p) const
 	{
 	    Pose ans{x_-p.x_, y_-p.y_, t_-p.t_};
 	    ans.angleNomalization();
 	    return ans;
 	}

	Pose Pose::operator+(const Pose & p) const
 	{
 	    Pose ans{x_+p.x_, y_+p.y_, t_+p.t_};
 	    ans.angleNomalization();
 	    return ans;
 	}
}

