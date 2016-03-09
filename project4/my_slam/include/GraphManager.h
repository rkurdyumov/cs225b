#pragma once
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include "karto_scan_matcher.h"
#include "../../../include/util.h"
#include <map>
#include <algorithm>
#include <iostream>

// g2o includes
#include "vertex_se2.h"
#include "vertex_point_xy.h"
#include "edge_se2.h"
#include "edge_se2_pointxy.h"
#include "se2.h"
#include "g2o/core/graph_optimizer_sparse.h"
#include "g2o/core/block_solver.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

using namespace std;
using namespace karto_scan_matcher;
using namespace g2o;
using namespace tutorial;

#define LOCAL_THRESHOLD_DIST 0.5 // in meters
#define DEFAULT_K 4 // Number of nodes to compare with in sequential scan matching
#define DEFAULT_N 2 // Number of nodes on either side for local registration

struct ParentChildPair
{
	int parent;
	int child;
};

class GraphManager
{
public:
	GraphManager(int K = DEFAULT_K, int N = DEFAULT_N)
	{
		// Number of last neighbors to use in sequential scan matching
		this->setK(K);
		this->setN(N);

		// Optimization initialization
		typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
		typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
		SlamLinearSolver* linearSolver = new SlamLinearSolver();
		linearSolver->setBlockOrdering(false);
		SlamBlockSolver* solver = new SlamBlockSolver(&this->optimizer, linearSolver);
		this->optimizer.setSolver(solver);
	}

	// Number of last poses to be used in sequential scan matching
	void setK(int K)
	{
		this->K = K;
	}

	// The number of poses on each side to consider during local registration scan matching
	void setN(int N)
	{
		this->N = N;
	}

	// All poses should be in the global frame
	void addNode(ScanWithPose& scanWithPose)
	{
		this->scans.push_back(scanWithPose.scan);
		this->barycenters.push_back(this->computeBarycenter(scanWithPose.pose,scanWithPose.scan));
		this->poses.push_back(scanWithPose.pose);

		SE2* pose_SE2_p = new SE2(scanWithPose.pose.x,scanWithPose.pose.y,scanWithPose.pose.theta);

		VertexSE2* vert =  new VertexSE2;
		vert->setId((int)this->poses.size()-1);
		vert->setEstimate(*pose_SE2_p);
		this->optimizer.addVertex(vert);
	}

	//  Adds an edge to the graph given the index of the parent's pose and covConstraint is in the global frame
	void addConstraint(int parentIndex, int childIndex, const geometry_msgs::Pose2D childPose, const Eigen::Matrix3d& covConstraint)
	{
		// Grab the vertices in the graph that correspond to the parentIndex and childIndex
		EdgeSE2* edge = new EdgeSE2;
		edge->vertices()[0] = this->optimizer.vertex(parentIndex);
		edge->vertices()[1] = this->optimizer.vertex(childIndex);

		// Convert the parent's index to its pose, which can be interpreted as a transformation.  transformation is position
		//     of the child in the parent's frame
		geometry_msgs::Pose2D parentPose = this->poses[parentIndex];
		const SE2 parent_pose_SE2(parentPose.x,parentPose.y,parentPose.theta);
		const SE2 child_pose_SE2(childPose.x,childPose.y,childPose.theta);
		SE2 childMeanInParentsFrame = parent_pose_SE2.inverse() * child_pose_SE2;

		// Set the position of the child in the parent's frame
		edge->setMeasurement(childMeanInParentsFrame);
		edge->setInverseMeasurement(childMeanInParentsFrame.inverse());

		// Convert information matrix from global frame to parent's frame
		Eigen::Matrix3d information_in_global_frame = covConstraint.inverse();
		Eigen::Matrix3d rotation_g_to_p;
		float c = cos(-parentPose.theta);
		float s = sin(-parentPose.theta);
		rotation_g_to_p << c,-s,0, s,c,0, 0,0,1;
		Eigen::Matrix3d information_in_parent_frame = rotation_g_to_p*information_in_global_frame*rotation_g_to_p.inverse();

		// Set the inverse of the covariance matrix for the child's pose in the parent's frame
		edge->setInformation(information_in_parent_frame);
		this->optimizer.addEdge(edge);

		//cout << "Parent_frame: mean=(" << childMeanInParentsFrame[0] << "," << childMeanInParentsFrame[1] << "," << childMeanInParentsFrame[2] << ") info=" << endl << information_in_parent_frame << endl << endl;
	}

	int getLatestScanWithPoseIndex()
	{
		return (int)this->poses.size()-1;
	}

	ScanWithPose& getCurrentScanWithPose()
	{
		ScanWithPose* currScanWithPose = new ScanWithPose(this->scans.back(),this->poses.back());
		return *currScanWithPose;
	}

	ScanWithPose getScanWithPoseForIndex(int index)
	{
		ScanWithPose* sWP = new ScanWithPose();
		sWP->scan = this->scans[index];
		sWP->pose = this->poses[index];
		return *sWP;
	}

	vector<ScanWithPose>& getScansWithPosesForIndices(vector<int>& indices)
	{
		vector<ScanWithPose>* scansWithPoses = new vector<ScanWithPose>();
		for(int i = 0; i < (int)indices.size(); i++)
		{
			ScanWithPose pose(this->scans[indices[i]],this->poses[indices[i]]);
			scansWithPoses->push_back(pose);
		}
		return *scansWithPoses;
	}

	vector<int>& getLastKNodes()
	{
		vector<int>* lastKNodes = new vector<int>();
		int i = (int)this->poses.size();
		while(i > ((int)this->poses.size()-this->K) && i > 0)
		{
			lastKNodes->push_back(i-1);
			i--;
		}
		return *lastKNodes;
	}

	vector<int>& findClosestNeighborsAroundIndex(int index)
	{
		vector<int>* neighborIndices = new vector<int>();
		for(int n = index-this->N; n <= index+this->N; n++)
		{
			if(n < 0 || n > (int)this->poses.size())
				continue;
			neighborIndices->push_back(n);
		}
		return *neighborIndices;
	}

	// Find map containing keys with pose indices and values with their corresponding barycenter
	//     distances to childIndex's barycenter
	#define MAX_BARYCENTER_DIST 1.0 // in meters
	map<int,float> getNeighborIndicesAndBarycenterDistances(int childIndex)
	{
		map<int,float> indicesAndBarycenterDists;
		geometry_msgs::Pose2D childBarycenter = this->barycenters[childIndex];
		float dist;

		// Don't find distance to the child or the nearest K poses to the child
		int i;
		for(i = 0; i < (int)this->barycenters.size()-K-1; i++)
		{
			dist = util::Utility::euclideanDistance(childBarycenter.x,childBarycenter.y,this->barycenters[i].x,this->barycenters[i].y);

			// Case this pose's barycenter is within a threshold distance from the child's barycenter
			if(dist < MAX_BARYCENTER_DIST)
				indicesAndBarycenterDists[i] = dist;
		}
		return indicesAndBarycenterDists;
	}

	bool value_comparer(map<int,float>::value_type& i1, map<int,float>::value_type& i2)
	{
		return i1.second<i2.second;
	}

	// Return the key corresponding to the max element in the map
	int findKeyCorrespondingToMaxValueInMap(map<int,float>& myMap)
	{
		map<int,float>::iterator iter = max_element(myMap.begin(),myMap.end());//,value_comparer);
		return iter->first;
	}

	#define PRINT_AMOUNT 200
	// Optimize the graph and relocate the poses and their barycenters
	void optimizeGraph()
	{
		// Fix the location of the first robot pose
		VertexSE2* firstPose = dynamic_cast<VertexSE2*>(this->optimizer.vertex(0));
		firstPose->setFixed(true);
		this->optimizer.initializeOptimization();
		this->optimizer.optimize(10);

		// Update all poses except the first
		for(int i = 1; i < (int)this->poses.size(); i++)
		{
			VertexSE2* vertex = dynamic_cast<VertexSE2*>(this->optimizer.vertex(i));
			double* estimate = new double[3];
			if(vertex->getEstimateData(estimate))
			{
				// Store the old pose before shifting
				geometry_msgs::Pose2D oldPose = this->poses[i];

				// Extract the (x,y,theta) from the vertex estimate
				this->poses[i].x = estimate[0];
				this->poses[i].y = estimate[1];
				this->poses[i].theta = estimate[2];

				// Update this pose's barycenter
				Eigen::Vector3d old_barycenter;
				old_barycenter << this->barycenters[i].x, this->barycenters[i].y, 1.0;

				Eigen::Matrix3d rotation_old_to_new;
				float c = cos(oldPose.theta-this->poses[i].theta);
				float s = sin(oldPose.theta-this->poses[i].theta);
				rotation_old_to_new << c,-s,0, s,c,0, 0,0,1;

				Eigen::Matrix3d translation;
				float dx = this->poses[i].x-oldPose.x;
				float dy = this->poses[i].y-oldPose.y;
				translation << 1,0,dx, 0,1,dy, 0,0,1;

				Eigen::Vector3d new_barycenter;
				new_barycenter = translation*rotation_old_to_new*old_barycenter;

				this->barycenters[i].x = new_barycenter(0);
				this->barycenters[i].y = new_barycenter(1);
			}
		}
	}

	geometry_msgs::Pose2D computeBarycenter(geometry_msgs::Pose2D& pose, sensor_msgs::LaserScan& scan)
	{
		// Loop through each point in a laser scan to find barycenter in laser frame
		const int SKIP_AMOUNT = 10;
		int index = 0, count = 0;
		float sumX=0, sumY=0;
		for(double theta = scan.angle_min; theta <= scan.angle_max; theta += scan.angle_increment*SKIP_AMOUNT)
		{
			float distance = scan.ranges[index];
			if(distance != scan.range_max)
			{
				sumX += distance*cos(theta);
				sumY += distance*sin(theta);
				count++;
			}
			index += SKIP_AMOUNT;
		}

		// Barycenter in laser frame
		btVector3 barycenter_in_laser(sumX/count,sumY/count,0);

		// Convert from the laser frame to the base frame
		btVector3 barycenter_in_base = btTransform(tf::createQuaternionFromYaw(0), btVector3(0.12, 0, 0))*barycenter_in_laser;

		// Transform to the global frame
		btVector3 barycenter_in_global = btTransform(tf::createQuaternionFromYaw(pose.theta),
				btVector3(pose.x, pose.y, 0))*barycenter_in_base;

		// Store barycenter in global frame
		geometry_msgs::Pose2D barycenter;
		barycenter.x = float(barycenter_in_global.x());
		barycenter.y = float(barycenter_in_global.y());
		barycenter.theta = float(0);
		return barycenter;
	}

	void visualizeGraph()
	{
		// Visualize nodes of graph (PoseArray) and their laser scans (Marker)
		geometry_msgs::PoseArray graph_nodes;
		visualization_msgs::Marker graph_scans;
		visualization_msgs::Marker graph_barycenters;
		geometry_msgs::PoseArray graph_odoms;
		visualization_msgs::Marker graph_edges;
		for(int i = 0; i < (int)this->poses.size(); i++)
		{
			// For graph nodes
			geometry_msgs::Pose2D pose = this->poses[i];
			geometry_msgs::Pose poseVis;
			poseVis.position.x = pose.x;
			poseVis.position.y = pose.y;
			poseVis.orientation = tf::createQuaternionMsgFromYaw(pose.theta);
			graph_nodes.poses.push_back(poseVis);

			// For graph odoms
			geometry_msgs::Pose2D odom = this->odoms[i];
			poseVis.position.x = odom.x;
			poseVis.position.y = odom.y;
			poseVis.orientation = tf::createQuaternionMsgFromYaw(odom.theta);
			graph_odoms.poses.push_back(poseVis);

			// For graph barycenters
			geometry_msgs::Pose2D barycenter = this->barycenters[i];
			geometry_msgs::Point point;
			point.x = barycenter.x;
			point.y = barycenter.y;
			graph_barycenters.points.push_back(point);

			const int SKIP_AMOUNT = 10;
			const float eps = 0.0001;
			sensor_msgs::LaserScan scan = this->scans[i];
			int index = 0;
			for(double theta = scan.angle_min; theta <= scan.angle_max; theta += scan.angle_increment*SKIP_AMOUNT)
			{
				float distance = scan.ranges[index];

				if(distance >= scan.range_max-eps)
				{
					index += SKIP_AMOUNT;
					continue;
				}

				// Point in laser frame
				btVector3 point_in_laser(distance*cos(theta), distance*sin(theta), 0);
				// Convert from the laser frame to the base frame
				btVector3 point_in_base = btTransform(tf::createQuaternionFromYaw(0), btVector3(0.12, 0, 0))*point_in_laser;

				// Transform to the odom frame
				btVector3 point_in_odom = btTransform(tf::createQuaternionFromYaw(pose.theta),
						btVector3(pose.x, pose.y, 0))*point_in_base;
				geometry_msgs::Point point;
				point.x = point_in_odom.x();
				point.y = point_in_odom.y();
				graph_scans.points.push_back(point);

				index += SKIP_AMOUNT;
			}
		}

		// For graph edges
		geometry_msgs::Point p1, p2;
		for(int i = 0; i < (int)this->edges.size(); i++)
		{
			p1.x = this->poses[edges[i].parent].x; p1.y = this->poses[edges[i].parent].y;
			p2.x = this->poses[edges[i].child].x; p2.y = this->poses[edges[i].child].y;
			graph_edges.points.push_back(p1);graph_edges.points.push_back(p2);
		}

		graph_nodes.header.frame_id = "odom";
		graph_nodes.header.stamp = ros::Time::now();

		graph_odoms.header.frame_id = "odom";
		graph_odoms.header.stamp = ros::Time::now();

		graph_scans.action = visualization_msgs::Marker::ADD;
		graph_scans.id = 0;
		graph_scans.type = visualization_msgs::Marker::POINTS;
		graph_scans.scale.x = 0.1;
		graph_scans.scale.y = 0.1;
		graph_scans.pose.orientation.w = 1.0;
		graph_scans.color.g = 1.0;
		graph_scans.color.a = 1.0;
		graph_scans.ns = "graph_scans";
		graph_scans.header.frame_id = "odom";
		graph_scans.header.stamp = ros::Time::now();

		graph_edges.action = visualization_msgs::Marker::ADD;
		graph_edges.id = 0;
		graph_edges.type = visualization_msgs::Marker::LINE_LIST;
		graph_edges.scale.x = 0.01;
		graph_edges.pose.orientation.w = 1.0;
		graph_edges.color.r = 1.0;
		graph_edges.color.g = 1.0;
		graph_edges.color.b = 1.0;
		graph_edges.color.a = 1.0;
		graph_edges.ns = "graph_edges";
		graph_edges.header.frame_id = "odom";
		graph_edges.header.stamp = ros::Time::now();

		graph_barycenters.action = visualization_msgs::Marker::ADD;
		graph_barycenters.id = 0;
		graph_barycenters.type = visualization_msgs::Marker::POINTS;
		graph_barycenters.scale.x = 0.1;
		graph_barycenters.scale.y = 0.1;
		graph_barycenters.pose.orientation.w = 1.0;
		graph_barycenters.color.r = 0.0;
		graph_barycenters.color.g = 0.0;
		graph_barycenters.color.b = 1.0;
		graph_barycenters.color.a = 1.0;
		graph_barycenters.ns = "graph_barycenters";
		graph_barycenters.header.frame_id = "odom";
		graph_barycenters.header.stamp = ros::Time::now();

		this->graph_nodes_pub.publish(graph_nodes);
		this->graph_scans_pub.publish(graph_scans);
		this->graph_barycenters_pub.publish(graph_barycenters);
		this->graph_odoms_pub.publish(graph_odoms);
		this->graph_edges_pub.publish(graph_edges);
	}

	ros::Publisher graph_nodes_pub;
	ros::Publisher graph_scans_pub;
	ros::Publisher graph_barycenters_pub;
	ros::Publisher graph_odoms_pub;
	ros::Publisher graph_edges_pub;

	vector<geometry_msgs::Pose2D> odoms;
	vector<ParentChildPair> edges;

	int numPoses()
	{
		return this->poses.size();
	}

private:
	int K;
	int N;
	vector<geometry_msgs::Pose2D> poses;
	vector<geometry_msgs::Pose2D> barycenters;
	vector<sensor_msgs::LaserScan> scans;
	SparseOptimizer optimizer;
};
