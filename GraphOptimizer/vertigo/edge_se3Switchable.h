/*
* edge_se3Switchable.h
*
*  Created on: 17.10.2011
*      Author: niko
*
*  Updated on: 14.01.2013
*      Author: Christian Kerl <christian.kerl@in.tum.de>
*/

//#ifndef EDGE_SE3SWITCHABLE_H
//#define EDGE_SE3SWITCHABLE_H
#pragma once
#include <g2o/types/slam3d/isometry3d_gradients.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/hyper_graph_action.h>
class EdgeSE3Switchable : public g2o::BaseMultiEdge<6, Eigen::Isometry3d>
{
public:
	EdgeSE3Switchable();

	bool read(std::istream& is);
	bool write(std::ostream& os) const;
	void computeError();
	void linearizeOplus();

	void setMeasurement(const Eigen::Isometry3d& m){
		_measurement = m;
		_inverseMeasurement = m.inverse();
	}

protected:
	Eigen::Isometry3d _inverseMeasurement;
};


//#endif /* EDGE_SE3SWITCHABLE_H_ */
