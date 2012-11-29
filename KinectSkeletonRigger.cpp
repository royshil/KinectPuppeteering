/*
 *  KinectSkeletonRigger.cpp
 *  KinectPuppeteering
 *
 *  Created by Roy Shilkrot on 11/25/12.
 *  Copyright 2012 MIT. All rights reserved.
 *
 */

#include "KinectSkeletonRigger.h"

void KinectSkeletonRigger::AutoRig() {
	if (m_m.vertices.size() <= 0 || m_skel.get() == NULL) {
		return;
	}
	cout << "autorigging...";
	Debugging::setOutStream(cout);
	KinectSkeleton tmp_skel = *m_skel;
	m_po = autorig(tmp_skel, m_m);
	cout << "DONE" << endl;
	
	if(m_po.attachment != NULL) {
		//		m_display_mesh = m_po.attachment->deform(m_m,m_transforms);	
		
		my_weights.resize(m_m.vertices.size());
		for (int i=0; i<m_m.vertices.size(); i++) {
			my_weights[i] = m_po.attachment->getWeights(i);
		}
	}		
}

void KinectSkeletonRigger::ComputeBoneTransforms(const EigT3f& globalT) {
	if(my_weights.size() > 0) {
//		float minpos = m_pose->jointV[extremes[1]].y();
		
//		float foot_tras = (minpos - 0.1) * embedding_scale["lankle"];
		
		Ts.resize(m_skel->fGraph().verts.size(),Eigen::Affine3f::Identity());
		for (int v_idx = 0; v_idx < m_skel->fGraph().verts.size(); v_idx++) 
		{
			int prev_idx = m_skel->fPrev()[v_idx];
			assert(m_pose->jointIdxToName[v_idx] == m_skel->jointIdxToName[v_idx]);
			string joint_name = m_skel->jointIdxToName[v_idx];
			if (joint_name == "shoulders") {
				prev_idx = m_skel->getJointForName("back");
			}
			int prev_prev_idx = m_skel->fPrev()[prev_idx];
			string prev_joint_name = m_skel->jointIdxToName[prev_idx];
			if (prev_joint_name == "shoulders") {
				prev_prev_idx = m_skel->getJointForName("back");
			}
			string prev_prev_joint_name = m_skel->jointIdxToName[prev_prev_idx];
			
			cout << prev_prev_joint_name << " -> " << prev_joint_name << " -> " << joint_name << endl;
			
			EigV3f pose_prevprev_to_prev = (m_pose->jointV[prev_joint_name]-m_pose->jointV[prev_prev_joint_name]);
			EigV3f skel_prevprev_to_prev = (m_skel->jointV[prev_joint_name]-m_skel->jointV[prev_prev_joint_name]);
			EigV3f trns_skel_to_pose = (pose_prevprev_to_prev - skel_prevprev_to_prev) * embedding_scale[prev_joint_name];

			EigQuat q_skel_to_pose = EigQuat::Identity().setFromTwoVectors(m_skel->referenceFrames[joint_name].row(0),
															  m_pose->referenceFrames[joint_name].row(0));
			
			EigV3f embed = Pinoc2EigV3f(m_po.embedding[v_idx]);
			EigV3f embed_prev = Pinoc2EigV3f(m_po.embedding[prev_idx]);
			EigV3f embed_prev_prev = Pinoc2EigV3f(m_po.embedding[prev_prev_idx]);

			EigQuat q_embed_to_skel = EigQuat::Identity().setFromTwoVectors((embed_prev - embed).normalized(),
																		   m_skel->referenceFrames[joint_name].row(0));
			
//			double embed_bone = (bone_offset-prev_bone_offset).norm();
			EigV3f embed_prevprev_to_prev = embed_prev - embed_prev_prev;
			EigV3f trns_embed_to_skel = (skel_prevprev_to_prev * embedding_scale[prev_joint_name] - embed_prevprev_to_prev);
			
			Ts[v_idx] = globalT * 
//					Eigen::Translation3f(EigV3f(0,-foot_tras,0)) *  // make sit on lowest joint
//				Eigen::Translation3f(-Pinoc2EigV3f(m_po.embedding[m_pose->getJointForName("back")])) * 
				Eigen::Translation3f(trns_skel_to_pose) *
				Eigen::Translation3f(trns_embed_to_skel) *
				Eigen::Translation3f(embed_prev) *
				q_skel_to_pose *
				q_embed_to_skel *
				Eigen::Translation3f(-embed_prev);
			
			//			cout << "transform " << joint_name << ": " << endl << Ts[v_idx].affine() << endl;
		}
		
		//apply trasforms
		//TODO: parallelize / GPU
		m_display_mesh = m_m;
		for (int vert_i=0; vert_i < m_display_mesh.vertices.size(); vert_i++) {
			EigV3f vert_pos = Pinoc2EigV3f(m_display_mesh.vertices[vert_i].pos);
			EigV3f new_vert_pos = EigV3f::Zero();
			Vector<double, -1> weights = my_weights[vert_i];
			assert(weights.size() == Ts.size() - 1);
			
			float w_sum = 0.0f;
			for (int bone_i=1; bone_i < Ts.size(); bone_i++) {
				float weight = weights[bone_i-1];
				if (weight > 1.) weight = 1.0;
				//				if(weight < 1e-8) continue;
				
				new_vert_pos += (Ts[bone_i] * vert_pos) * weight; //Linear blend skinning
			}
			
			m_display_mesh.vertices[vert_i].pos = Eig2PinocV3f(new_vert_pos);
		}
	}
}

void KinectSkeletonRigger::drawSkeleton(const KinectSkeleton::Ptr& _skel, bool draw_frames) {
	if (_skel.get() != NULL) {
		_skel->glDraw(/*globalTransform*/EigT3f::Identity(),draw_frames);
	}
}

void KinectSkeletonRigger::drawEmbeddingSkel() {
	if (m_skel.get() != NULL) {
		glDisable(GL_DEPTH_TEST);
		glDisable(GL_LIGHTING);
		for (int i=0; i<m_skel->fGraph().verts.size(); i++) {
			string jointName = m_skel->jointIdxToName[i];
			string prevJointName = m_skel->jointIdxToName[m_skel->fPrev()[i]];
			EigV3f v = Pinoc2EigV3f(m_po.embedding[i]);
			
			
			glPushMatrix();
			glTranslated(v[0], v[1], v[2]);
			glColor4f(1.0, 0, 1.0, 1.0);
			glutSolidSphere(0.01, 10, 10);
			glPopMatrix();
			
			//bone
			if (jointName != "shoulders") {
				EigV3f prev_v = Pinoc2EigV3f(m_po.embedding[m_skel->fPrev()[i]]);
				glBegin(GL_LINES);
				glColor4f(1.0, 1.0, 0.0, 1.0);
				glVertex3fv(&(v[0]));
				glVertex3fv(&(prev_v[0]));
				glEnd();
			}
		}	
		glEnable(GL_LIGHTING);
		glEnable(GL_DEPTH_TEST);
	}
}

void KinectSkeletonRigger::drawMesh(bool show_original) { 
	if (m_display_mesh.vertices.size() > 0) {
		glColor4f(0.55, 0.25, 0.25, 1.0);
		drawMesh(m_display_mesh, true, Vector3(0,0,0)); 
	}
	if(show_original && m_m.vertices.size() > 0) {
		glColor4f(0.55, 0.55, 0.55, 0.25);
		drawMesh(m_m, true, Vector3(0,0,0)); 
	}
}

void KinectSkeletonRigger::drawMesh(const Mesh &m, bool flatShading, Vector3 trans)
{
	int i;
	Vector3 normal;
	
	glBegin(GL_TRIANGLES);
	for(i = 0; i < (int)m.edges.size(); ++i) {
		int v = m.edges[i].vertex;
		const Vector3 &p = m.vertices[v].pos;
		
		if(!flatShading) {
			normal = m.vertices[v].normal;
			glNormal3d(normal[0], normal[1], normal[2]);
		}
		else if(i % 3 == 0) {
			const Vector3 &p2 = m.vertices[m.edges[i + 1].vertex].pos;
			const Vector3 &p3 = m.vertices[m.edges[i + 2].vertex].pos;
			
			normal = ((p2 - p) % (p3 - p)).normalize();
			glNormal3d(normal[0], normal[1], normal[2]);
		}
		
		glVertex3d(p[0] + trans[0], p[1] + trans[1], p[2] + trans[2]);
	}
	glEnd();
}

void KinectSkeletonRigger::LoadRig(const std::string& f) {
	if(f=="") return;
	ifstream ifs(f.c_str());
	string s_; 
	ifs >> m_meshfile;
	LoadMesh(m_meshfile);
	if(m_m.vertices.size() == 0) {
		cerr << "can't load mesh"<<endl; return;
	}
	ifs >> m_skeletonfile;
	LoadSkeletonFromText(m_skeletonfile);
	if(m_skel.get() == NULL || m_skel->fGraph().verts.size() == 0) {
		cerr << "can't load skeleton"<<endl; return;
	}
	int embedding_size,tmp;
	ifs >> embedding_size;
	m_po.embedding.resize(embedding_size);
	for (int i=0; i<embedding_size; i++) {
		ifs >> tmp >> m_po.embedding[i][0] >> m_po.embedding[i][1] >> m_po.embedding[i][2] >> tmp;
	}
	int attachment_size;
	ifs >> attachment_size;
	if(m_po.attachment == NULL) m_po.attachment = new Attachment;
	my_weights.resize(attachment_size);
	for (int i=0; i<attachment_size; i++) {
		//		my_weights[i].resize(embedding_size-1);
		for (int v=0; v<embedding_size-1; v++) {
			ifs >> my_weights[i][v];
		}
		//		cout << my_weights[i];
	}
	
	for (int v_idx = 0; v_idx < m_skel->fGraph().verts.size(); v_idx++) 
	{
		int prev_idx = m_skel->fPrev()[v_idx];
		string joint_name = m_skel->jointIdxToName[v_idx];
		string prev_joint_name = m_skel->jointIdxToName[prev_idx];
		if (joint_name == "shoulders") {
			prev_joint_name = "back";
			prev_idx = m_skel->getJointForName("back");
		}
		EigV3f skel_offset = m_skel->jointV[joint_name] - m_skel->jointV[prev_joint_name];
		EigV3f embed_offest = Pinoc2EigV3f(m_po.embedding[v_idx]) - Pinoc2EigV3f(m_po.embedding[prev_idx]);
		embedding_scale[joint_name] = embed_offest.norm() / skel_offset.norm(); // / ;
		
		cout << "embedding_scale " << joint_name << ": " << embedding_scale[joint_name] << endl;		
	}
}

void KinectSkeletonRigger::SaveRig(const std::string& f) {
	if (f == "") {
		return;
	}
	if (m_meshfile == "" || m_skeletonfile == "") {
		cerr << "no mesh or skeleton loaded" << endl;
		return;
	}
	if (m_po.attachment == NULL) {
		cerr << "no rigging" << endl;
		return;
	}
	ofstream ofs(f.c_str());
	ofs << m_meshfile << endl << m_skeletonfile << endl;
	
	//output embedding
	ofs << m_po.embedding.size() << endl;
	for(int i = 0; i < (int)m_po.embedding.size(); ++i) {
        ofs << i << " " << m_po.embedding[i][0] << " " << m_po.embedding[i][1] <<
		" " << m_po.embedding[i][2] << " " << m_skel->fPrev()[i] << endl;
    }
	
    //output attachment
	ofs << m_m.vertices.size() << endl;
    for(int i = 0; i < (int)m_m.vertices.size(); ++i) {
        Vector<double, -1> v = my_weights[i];
        for(int j = 0; j < v.size(); ++j) {
            double d = floor(0.5 + v[j] * 10000.) / 10000.;
            ofs << d << " ";
        }
    }
	ofs.close();
}
