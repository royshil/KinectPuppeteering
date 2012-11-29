/*
 *  KinectSkeleton.h
 *  AmitParametricBowl
 *
 *  Created by Roy Shilkrot on 11/11/12.
 *  Copyright 2012 MIT. All rights reserved.
 *
 */
#pragma once

#include "Pinocchio/pinocchioApi.h"

typedef Eigen::Vector3f EigV3f;
typedef Eigen::Matrix3f EigM3f;
typedef Eigen::Quaternionf EigQuat;
typedef Eigen::Transform<float,3,Eigen::Affine> EigT3f;
inline Vector3 Eig2PinocV3f(const Eigen::Vector3f& eigv) { return Vector3(eigv[0],eigv[1],eigv[2]); }
inline Eigen::Vector3f Pinoc2EigV3f(const Vector3& pinocv) { return EigV3f(pinocv[0],pinocv[1],pinocv[2]); }

class KinectSkeleton : public Skeleton
{
private:
	string m_skel_file;
	void getStartEnd(ifstream& ifs, double& startx, double& starty, double& startz, double& endx, double& endy, double& endz);
	void getXMLStartEnd(ifstream& ifs, double& startx, double& starty, double& startz, double& endx, double& endy, double& endz);
	EigV3f convertKinectDmapTo3D(const EigV3f& v_);
public:
	map<string,EigV3f> jointV;
	map<string,EigM3f> referenceFrames;
	map<int,string> jointIdxToName;
	EigM3f m_back_frame;
	double m_skel_scale;
	EigV3f m_back_offset;
	map<int,int> forwardJoint;
	
	Eigen::RowVectorXf getAsVector(const EigT3f& global = EigT3f::Identity());
	
	void glDraw(const Eigen::Transform<float,3,Eigen::Affine>& global = Eigen::Affine3f::Identity(), bool draw_frames = false);
	
	void makeFramesForJoints();
	
	EigM3f makeFrameForJoint(const std::string& joint, const std::string& prev_joint);
	
	void populateSkelFromFile(const std::string& skelFile);
	void populateSkelFromXMLFile(const std::string& skelFile);
	void populateSkelFromVector(const Eigen::RowVectorXf& from_vector);
	
	void transformJoints(const EigM3f& back_frame, double skel_scale, const EigV3f& back_offset);	
	void FindExtremeJoints(vector<string>& extremes);
	
	void makePinnocSkel();
	
	string getSkelFile() { return m_skel_file; } 
	
	KinectSkeleton() {};
	KinectSkeleton(const Eigen::RowVectorXf& from_vector);
    KinectSkeleton(const std::string& skelFile);
	KinectSkeleton(const std::string& skelFile, const EigM3f& back_frame, double skel_scale, const EigV3f& back_offset);
	
	typedef boost::shared_ptr<KinectSkeleton> Ptr;
};