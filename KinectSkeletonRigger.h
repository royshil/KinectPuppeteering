/*
 *  KinectSkeletonRigger.h
 *  KinectPuppeteering
 *
 *  Created by Roy Shilkrot on 11/25/12.
 *  Copyright 2012 MIT. All rights reserved.
 *
 */

#include "KinectSkeleton.h"
#include "Pinocchio/pinocchioApi.h"


class KinectSkeletonRigger {
private:
	//------- skeleton
	string m_skeletonfile;
	KinectSkeleton::Ptr m_skel,m_pose;
	void LoadPose(const std::string& skelFile, KinectSkeleton::Ptr& _pose) {
		if (skelFile != "") _pose.reset(new KinectSkeleton(skelFile));
	}
public:
	void LoadSkeletonFromText(const std::string& skelFile) {
		m_skeletonfile = skelFile;
		LoadPose(skelFile,m_skel);
	}
	void LoadPoseFromText(const std::string& skelFile) {
		LoadPose(skelFile,m_pose);
		ComputeBoneTransforms();
	}
	void drawSkeleton() { drawSkeleton(m_skel, true); drawSkeleton(m_pose); drawEmbeddingSkel(); }
	void drawSkeleton(const KinectSkeleton::Ptr& _skel, bool draw_frames = false);
	void drawEmbeddingSkel();
	
private:	
	//------- mesh
	string m_meshfile;
	Mesh m_m, m_display_mesh;
public:
	void LoadMesh(const std::string& meshFile) {  
		if(meshFile == "") return;
		m_meshfile = meshFile;
		m_m = Mesh(meshFile);
		if(m_m.vertices.size() == 0) {
			cout << "Error reading file.  Aborting." << endl;
			return;
		}
		m_m.normalizeBoundingBox();
		m_m.computeVertexNormals();	
	}
	void drawMesh(bool show_original = false);
	void drawMesh(const Mesh &m, bool flatShading, Vector3 trans);

	
private:
	//------- rig
	vector<Vector<double, -1> > my_weights;
	PinocchioOutput m_po;
	map<string,float> embedding_scale;
	vector<Eigen::Affine3f> Ts;
public:
	void AutoRig();
	void ComputeBoneTransforms(const EigT3f& globalT = EigT3f::Identity());
	void SaveRig(const std::string& f);
	void LoadRig(const std::string& f);
	
	typedef boost::shared_ptr<KinectSkeletonRigger> Ptr;
};