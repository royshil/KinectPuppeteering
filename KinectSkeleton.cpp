/*
 *  KinectSkeleton.cpp
 *  AmitParametricBowl
 *
 *  Created by Roy Shilkrot on 11/11/12.
 *  Copyright 2012 MIT. All rights reserved.
 *
 */

#include "KinectSkeleton.h"
 
void KinectSkeleton::glDraw(const Eigen::Transform<float,3,Eigen::Affine>& global, bool draw_frames) {
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glPushMatrix();
	glMultMatrixf(global.data());

	for (int i=0; i<this->fGraph().verts.size(); i++) {
		string jointName = this->jointIdxToName[i];
		string prevJointName = this->jointIdxToName[this->fPrev()[i]];
		EigV3f v = /*global */ this->jointV[jointName];
		//		if (m_po.embedding.size() > i) {
		//			v = (v - m_skel->fGraph().verts[i]) * (1.0/embedding_scale) + m_po.embedding[i];
		//		} 
		
		glPushMatrix();
		glTranslated(v[0], v[1], v[2]);
		glColor4f(1.0, 0, 1.0, 1.0);
		glutSolidSphere(0.01, 10, 10);
		glPopMatrix();
		//		if (m_po.embedding.size() > _skel->fPrev()[i]) {
		//			prev_v = (prev_v - m_skel->fGraph().verts[_skel->fPrev()[i]]) * (1.0/embedding_scale) + m_po.embedding[_skel->fPrev()[i]];
		//		}
		
		//bone
		if (jointName != "shoulders") {
			EigV3f prev_v = /*global */ this->jointV[prevJointName];
			glBegin(GL_LINES);
			glColor4f(1.0, 1.0, 0.0, 1.0);
			glVertex3fv(&(v[0]));
			glVertex3fv(&(prev_v[0]));
			glEnd();
		}
		
		if(draw_frames) {
			//frame
			EigM3f frame = this->referenceFrames[jointName];
			GLfloat linew; glGetFloatv(GL_LINE_WIDTH, &linew);
			glLineWidth(3);
			glEnable(GL_LINE_SMOOTH);
			glBegin(GL_LINES);
			glColor4f(1, 0, 0, 1);
			glVertex3fv(&(v[0]));
			EigV3f dir = this->jointV[jointName].transpose() + frame.row(0) * 0.05; //right-red
			glVertex3fv(&(dir[0]));
			glColor4f(0, 1, 0, 1);
			glVertex3fv(&(v[0]));
			dir = this->jointV[jointName].transpose() + frame.row(1) * 0.05;		//up-green
			glVertex3fv(&(dir[0]));
			glColor4f(0, 0, 1, 1);
			glVertex3fv(&(v[0]));
			dir = this->jointV[jointName].transpose() + frame.row(2) * 0.05;		//forward-blue
			glVertex3fv(&(dir[0]));
			glEnd();
			glLineWidth(linew);
		}
	}
	
	glPopMatrix();

	glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);
}

void KinectSkeleton::getStartEnd(ifstream& ifs, double& startx, double& starty, double& startz, double& endx, double& endy, double& endz) {
	char cbuf[512]; string s_; char comma;
	ifs.getline(cbuf, 512);
	stringstream ss(cbuf);
	ss >> s_ >> startx >> comma >> starty >> comma >> startz;
	ifs.getline(cbuf, 512);
	stringstream ss1 (cbuf);
	ss1 >> s_ >> endx >> comma >> endy >> comma >> endz;
}

EigV3f KinectSkeleton::convertKinectDmapTo3D(const EigV3f& v_) {
	/*
	 based on http://labs.manctl.com/rgbdemo/index.php/Documentation/KinectCalibrationTheory
	 */
	double fx_d = 5.9421434211923247e+02;
	double fy_d = 5.9104053696870778e+02;
	double cx_d = 3.3930780975300314e+02;
	double cy_d = 2.4273913761751615e+02;
	
	EigV3f v;
	v[0] = (v_[0] - cx_d) * v_[2] / fx_d;
	v[1] = (v_[1] - cy_d) * v_[2] / fy_d;
	v[2] = v_[2];
	
	return v;
}

void KinectSkeleton::makeFramesForJoints() {
	referenceFrames["shoulders"] = makeFrameForJoint("shoulders", "back");
	referenceFrames["head"] = makeFrameForJoint("head", "shoulders");
	referenceFrames["back"] = makeFrameForJoint("back", "shoulders");
	referenceFrames["lshoulder"] = makeFrameForJoint("lshoulder", "shoulders");
	referenceFrames["rshoulder"] = makeFrameForJoint("rshoulder", "shoulders");
	referenceFrames["lthigh"] = makeFrameForJoint("lthigh", "back");
	referenceFrames["rthigh"] = makeFrameForJoint("rthigh", "back");
	referenceFrames["lknee"] = makeFrameForJoint("lknee", "lthigh");
	referenceFrames["lankle"] = makeFrameForJoint("lankle", "lknee");
	referenceFrames["rknee"] = makeFrameForJoint("rknee", "rthigh");
	referenceFrames["rankle"] = makeFrameForJoint("rankle", "rknee");
	referenceFrames["lelbow"] = makeFrameForJoint("lelbow", "lshoulder");
	referenceFrames["lhand"] = makeFrameForJoint("lhand", "lelbow");
	referenceFrames["relbow"] = makeFrameForJoint("relbow", "rshoulder");
	referenceFrames["rhand"] = makeFrameForJoint("rhand", "relbow");		
}

EigM3f KinectSkeleton::makeFrameForJoint(const std::string& joint, const std::string& prev_joint) {
	EigV3f rightV = (jointV[prev_joint] - jointV[joint]); rightV.normalize();
	
	EigV3f upV;
	if(forwardJoint.count(getJointForName(joint))>0) {
			//a forward joint exists
		EigV3f fv = (jointV[jointIdxToName[forwardJoint[getJointForName(joint)]]] - jointV[joint]).normalized();
		upV = rightV.cross(fv);
	} else {
		upV = rightV.cross(EigV3f::UnitZ());  
	}
	upV.normalize();
	if(upV[1] < 0) upV = -upV;
	
	EigV3f forwardV = rightV.cross(upV); forwardV.normalize();
	if(forwardV[2] < 0) forwardV = -forwardV;
	EigM3f frame; frame << rightV , upV , forwardV; frame.transposeInPlace();
	return frame;
}		

inline void getNumFromXML(const string& s__, double& num) {
	string s_(s__);
	boost::trim(s_);
	int begin_num = s_.find('>')+1;
	int end_num = s_.rfind('<');
	string num_s = s_.substr(begin_num, end_num-begin_num);
	num = atof(num_s.c_str());
}

void KinectSkeleton::getXMLStartEnd(ifstream& ifs, double& startx, double& starty, double& startz, double& endx, double& endy, double& endz) {
	char cbuf[512]; string s_;
	ifs.getline(cbuf, 512); s_ = string(cbuf);
	getNumFromXML(s_, startx);
	ifs.getline(cbuf, 512); s_ = string(cbuf);
	getNumFromXML(s_, starty);
	ifs.getline(cbuf, 512); s_ = string(cbuf);
	getNumFromXML(s_, startz);
	ifs.getline(cbuf, 512); s_ = string(cbuf);
	getNumFromXML(s_, endx);
	ifs.getline(cbuf, 512); s_ = string(cbuf);
	getNumFromXML(s_, endy);
	ifs.getline(cbuf, 512); s_ = string(cbuf);
	getNumFromXML(s_, endz);
}


void KinectSkeleton::populateSkelFromXMLFile(const std::string& skelFile) {
	ifstream ifs(skelFile.c_str(),ifstream::in);
	double startx, starty, startz, endx, endy, endz;
	
	char cbuf[512];
	while (!ifs.eof()) {
		ifs.getline(cbuf, 512);
		stringstream ss(cbuf);
		string s_; ss >> s_; 
		boost::trim(s_);
		if (s_ == "<neck>") {
			getXMLStartEnd(ifs, startx, starty, startz, endx, endy, endz);
			jointV["head"] = convertKinectDmapTo3D(EigV3f(startx, starty, startz));
			jointV["shoulders"] = convertKinectDmapTo3D(EigV3f(endx, endy, endz));
		}
		if (s_ == "<left_shoulder>") {
			getXMLStartEnd(ifs, startx, starty, startz, endx, endy, endz);
			jointV["lshoulder"] = convertKinectDmapTo3D(EigV3f(endx, endy, endz));
		}
		if (s_ == "<left_upper_arm>") {
			getXMLStartEnd(ifs, startx, starty, startz, endx, endy, endz);
			jointV["lelbow"] = convertKinectDmapTo3D(EigV3f(endx, endy, endz));
		}
		if (s_ == "<left_lower_arm>") {
			getXMLStartEnd(ifs, startx, starty, startz, endx, endy, endz);
			jointV["lhand"] = convertKinectDmapTo3D(EigV3f(endx, endy, endz));
		}
		if (s_ == "<right_shoulder>") {
			getXMLStartEnd(ifs, startx, starty, startz, endx, endy, endz);
			jointV["rshoulder"] = convertKinectDmapTo3D(EigV3f(endx, endy, endz));
		}
		if (s_ == "<right_upper_arm>") {
			getXMLStartEnd(ifs, startx, starty, startz, endx, endy, endz);
			jointV["relbow"] = convertKinectDmapTo3D(EigV3f(endx, endy, endz));
		}
		if (s_ == "<right_lower_arm>") {
			getXMLStartEnd(ifs, startx, starty, startz, endx, endy, endz);
			jointV["rhand"] = convertKinectDmapTo3D(EigV3f(endx, endy, endz));
		}
		if (s_ == "<left_upper_torso>") {
			getXMLStartEnd(ifs, startx, starty, startz, endx, endy, endz);
			jointV["back"] = convertKinectDmapTo3D(EigV3f(endx, endy, endz));
		}
		if (s_ == "<left_lower_torso>") {
			getXMLStartEnd(ifs, startx, starty, startz, endx, endy, endz);
			jointV["lthigh"] = convertKinectDmapTo3D(EigV3f(endx, endy, endz));
		}
		if (s_ == "<left_upper_leg>") {
			getXMLStartEnd(ifs, startx, starty, startz, endx, endy, endz);
			jointV["lknee"] = convertKinectDmapTo3D(EigV3f(endx, endy, endz));
		}
		if (s_ == "<left_lower_leg>") {
			getXMLStartEnd(ifs, startx, starty, startz, endx, endy, endz);
			jointV["lankle"] = convertKinectDmapTo3D(EigV3f(endx, endy, endz));
		}
		if (s_ == "<right_lower_torso>") {
			getXMLStartEnd(ifs, startx, starty, startz, endx, endy, endz);
			jointV["rthigh"] = convertKinectDmapTo3D(EigV3f(endx, endy, endz));
		}
		if (s_ == "<right_upper_leg>") {
			getXMLStartEnd(ifs, startx, starty, startz, endx, endy, endz);
			jointV["rknee"] = convertKinectDmapTo3D(EigV3f(endx, endy, endz));
		}
		if (s_ == "<right_lower_leg>") {
			getXMLStartEnd(ifs, startx, starty, startz, endx, endy, endz);
			jointV["rankle"] = convertKinectDmapTo3D(EigV3f(endx, endy, endz));
		}
	}	
}

void KinectSkeleton::populateSkelFromFile(const std::string& skelFile) {
	string subst = skelFile.substr(skelFile.length()-4,4);
	if(subst == ".xml") {
		populateSkelFromXMLFile(skelFile);
		return;
	}
	
	ifstream ifs(skelFile.c_str(),ifstream::in);
	double startx, starty, startz, endx, endy, endz;
	
	char cbuf[512];
	while (!ifs.eof()) {
		ifs.getline(cbuf, 512);
		stringstream ss(cbuf);
		string s_; ss >> s_;
		//			cout << s_ << endl;
		if (s_ == "neck:") {
			getStartEnd(ifs, startx, starty, startz, endx, endy, endz);
			jointV["head"] = convertKinectDmapTo3D(EigV3f(startx, starty, startz));
			jointV["shoulders"] = convertKinectDmapTo3D(EigV3f(endx, endy, endz));
		}
		if (s_ == "left_shoulder:") {
			getStartEnd(ifs, startx, starty, startz, endx, endy, endz);
			jointV["lshoulder"] = convertKinectDmapTo3D(EigV3f(endx, endy, endz));
		}
		if (s_ == "left_upper_arm:") {
			getStartEnd(ifs, startx, starty, startz, endx, endy, endz);
			jointV["lelbow"] = convertKinectDmapTo3D(EigV3f(endx, endy, endz));
		}
		if (s_ == "left_lower_arm:") {
			getStartEnd(ifs, startx, starty, startz, endx, endy, endz);
			jointV["lhand"] = convertKinectDmapTo3D(EigV3f(endx, endy, endz));
		}
		if (s_ == "right_shoulder:") {
			getStartEnd(ifs, startx, starty, startz, endx, endy, endz);
			jointV["rshoulder"] = convertKinectDmapTo3D(EigV3f(endx, endy, endz));
		}
		if (s_ == "right_upper_arm:") {
			getStartEnd(ifs, startx, starty, startz, endx, endy, endz);
			jointV["relbow"] = convertKinectDmapTo3D(EigV3f(endx, endy, endz));
		}
		if (s_ == "right_lower_arm:") {
			getStartEnd(ifs, startx, starty, startz, endx, endy, endz);
			jointV["rhand"] = convertKinectDmapTo3D(EigV3f(endx, endy, endz));
		}
		if (s_ == "left_upper_torso:") {
			getStartEnd(ifs, startx, starty, startz, endx, endy, endz);
			jointV["back"] = convertKinectDmapTo3D(EigV3f(endx, endy, endz));
		}
		if (s_ == "left_lower_torso:") {
			getStartEnd(ifs, startx, starty, startz, endx, endy, endz);
			jointV["lthigh"] = convertKinectDmapTo3D(EigV3f(endx, endy, endz));
		}
		if (s_ == "left_upper_leg:") {
			getStartEnd(ifs, startx, starty, startz, endx, endy, endz);
			jointV["lknee"] = convertKinectDmapTo3D(EigV3f(endx, endy, endz));
		}
		if (s_ == "left_lower_leg:") {
			getStartEnd(ifs, startx, starty, startz, endx, endy, endz);
			jointV["lankle"] = convertKinectDmapTo3D(EigV3f(endx, endy, endz));
		}
		if (s_ == "right_lower_torso:") {
			getStartEnd(ifs, startx, starty, startz, endx, endy, endz);
			jointV["rthigh"] = convertKinectDmapTo3D(EigV3f(endx, endy, endz));
		}
		if (s_ == "right_upper_leg:") {
			getStartEnd(ifs, startx, starty, startz, endx, endy, endz);
			jointV["rknee"] = convertKinectDmapTo3D(EigV3f(endx, endy, endz));
		}
		if (s_ == "right_lower_leg:") {
			getStartEnd(ifs, startx, starty, startz, endx, endy, endz);
			jointV["rankle"] = convertKinectDmapTo3D(EigV3f(endx, endy, endz));
		}
	}
}

void KinectSkeleton::transformJoints(const EigM3f& back_frame, double skel_scale, const EigV3f& back_offset) {
	Eigen::Affine3f T = Eigen::UniformScaling<float>(skel_scale) *
						back_frame *
						Eigen::Translation3f(-back_offset);
	
	for(map<string,EigV3f>::iterator itr = jointV.begin(); itr != jointV.end(); ++itr) { (*itr).second = T * (*itr).second; }
}		

void KinectSkeleton::makePinnocSkel() {
	makeJoint("shoulders",  Eig2PinocV3f(jointV["shoulders"]));					//0
	makeJoint("head",       Eig2PinocV3f(jointV["head"]),		"shoulders");	//1
	makeJoint("back",       Eig2PinocV3f(jointV["back"]),		"shoulders");	//2
	
	makeJoint("lthigh",     Eig2PinocV3f(jointV["lthigh"]),		"back");		//3
	makeJoint("lknee",      Eig2PinocV3f(jointV["lknee"]),		"lthigh");		//4
	makeJoint("lankle",     Eig2PinocV3f(jointV["lankle"]),		"lknee");		//5
	
	makeJoint("rthigh",     Eig2PinocV3f(jointV["rthigh"]),		"back");		//6
	makeJoint("rknee",      Eig2PinocV3f(jointV["rknee"]),		"rthigh");		//7
	makeJoint("rankle",     Eig2PinocV3f(jointV["rankle"]),		"rknee");		//8
	
	makeJoint("lshoulder",  Eig2PinocV3f(jointV["lshoulder"]),	"shoulders");	//9
	makeJoint("lelbow",     Eig2PinocV3f(jointV["lelbow"]),		"lshoulder");	//10
	makeJoint("lhand",      Eig2PinocV3f(jointV["lhand"]),		"lelbow");		//11
	
	makeJoint("rshoulder",  Eig2PinocV3f(jointV["rshoulder"]),	"shoulders");	//12
	makeJoint("relbow",     Eig2PinocV3f(jointV["relbow"]),		"rshoulder");	//13
	makeJoint("rhand",      Eig2PinocV3f(jointV["rhand"]),		"relbow");		//14
	
	//symmetry
	makeSymmetric("lthigh", "rthigh");
	makeSymmetric("lknee", "rknee");
	makeSymmetric("lankle", "rankle");
	//		makeSymmetric("lfoot", "rfoot");
	
	makeSymmetric("lshoulder", "rshoulder");
	makeSymmetric("lelbow", "relbow");
	makeSymmetric("lhand", "rhand");
	
	initCompressed();
	
	setFoot("lankle");
	setFoot("rankle");
	
	setFat("shoulders");
	setFat("head");
	setFat("back");
	
	jointIdxToName[getJointForName("shoulders")] = "shoulders";
	jointIdxToName[getJointForName("head")] = "head";
	jointIdxToName[getJointForName("back")] = "back";
	jointIdxToName[getJointForName("lthigh")] = "lthigh";
	jointIdxToName[getJointForName("lknee")] = "lknee";
	jointIdxToName[getJointForName("lankle")] = "lankle";
	jointIdxToName[getJointForName("rthigh")] = "rthigh";
	jointIdxToName[getJointForName("rknee")] = "rknee";
	jointIdxToName[getJointForName("rankle")] = "rankle";
	jointIdxToName[getJointForName("lshoulder")] = "lshoulder";
	jointIdxToName[getJointForName("lelbow")] = "lelbow";
	jointIdxToName[getJointForName("lhand")] = "lhand";
	jointIdxToName[getJointForName("rshoulder")] = "rshoulder";
	jointIdxToName[getJointForName("relbow")] = "relbow";
	jointIdxToName[getJointForName("rhand")] = "rhand";

//	forwardJoint[fPrev()[getJointForName("shoulders")]] = getJointForName("shoulders");
//	forwardJoint[fPrev()[getJointForName("head")]] = getJointForName("head");
//	forwardJoint[fPrev()[getJointForName("back")]] = getJointForName("back");
	forwardJoint[fPrev()[getJointForName("lthigh")]] = getJointForName("lthigh");
	forwardJoint[fPrev()[getJointForName("lknee")]] = getJointForName("lknee");
	forwardJoint[fPrev()[getJointForName("lankle")]] = getJointForName("lankle");
	forwardJoint[fPrev()[getJointForName("rthigh")]] = getJointForName("rthigh");
	forwardJoint[fPrev()[getJointForName("rknee")]] = getJointForName("rknee");
	forwardJoint[fPrev()[getJointForName("rankle")]] = getJointForName("rankle");
	forwardJoint[fPrev()[getJointForName("lshoulder")]] = getJointForName("lshoulder");
	forwardJoint[fPrev()[getJointForName("lelbow")]] = getJointForName("lelbow");
	forwardJoint[fPrev()[getJointForName("lhand")]] = getJointForName("lhand");
	forwardJoint[fPrev()[getJointForName("rshoulder")]] = getJointForName("rshoulder");
	forwardJoint[fPrev()[getJointForName("relbow")]] = getJointForName("relbow");
	forwardJoint[fPrev()[getJointForName("rhand")]] = getJointForName("rhand");
	
}

Eigen::RowVectorXf KinectSkeleton::getAsVector(const EigT3f& global) {
	Eigen::RowVectorXf v(45);
	v.segment(0,3) = jointV["back"];
	v.segment(3,3) = jointV["head"];
	v.segment(6,3) = jointV["lankle"];
	v.segment(9,3) = jointV["lelbow"];
	v.segment(12,3) = jointV["lhand"];
	v.segment(15,3) = jointV["lknee"];
	v.segment(18,3) = jointV["lshoulder"];
	v.segment(21,3) = jointV["lthigh"];
	v.segment(24,3) = jointV["rankle"];
	v.segment(27,3) = jointV["relbow"];
	v.segment(30,3) = jointV["rhand"];
	v.segment(33,3) = jointV["rknee"];
	v.segment(36,3) = jointV["rshoulder"];
	v.segment(39,3) = jointV["rthigh"];
	v.segment(42,3) = jointV["shoulders"];
	return v;
}

void KinectSkeleton::populateSkelFromVector(const Eigen::RowVectorXf& from_vector) {
	jointV["back"] = from_vector.segment(0,3);
	jointV["head"] = from_vector.segment(3,3);
	jointV["lankle"] = from_vector.segment(6,3);
	jointV["lelbow"] = from_vector.segment(9,3);
	jointV["lhand"] = from_vector.segment(12,3);
	jointV["lknee"] = from_vector.segment(15,3);
	jointV["lshoulder"] = from_vector.segment(18,3);
	jointV["lthigh"] = from_vector.segment(21,3);
	jointV["rankle"] = from_vector.segment(24,3);
	jointV["relbow"] = from_vector.segment(27,3);
	jointV["rhand"] = from_vector.segment(30,3);
	jointV["rknee"] = from_vector.segment(33,3);
	jointV["rshoulder"] = from_vector.segment(36,3);
	jointV["rthigh"] = from_vector.segment(39,3);
	jointV["shoulders"] = from_vector.segment(42,3);
}

KinectSkeleton::KinectSkeleton(const Eigen::RowVectorXf& from_vector) {
	populateSkelFromVector(from_vector);

	makePinnocSkel();
	
	makeFramesForJoints();	
}

void KinectSkeleton::FindExtremeJoints(vector<string>& extremes) {
	float	highest_val = std::numeric_limits<float>::min(), 
			lowest_val = 1.0, 
			rightmost_val = std::numeric_limits<float>::min(),
			leftmost_val = 1.0;
	extremes.resize(4);
	
	map<string,EigV3f>::iterator itr = jointV.begin();
	for(;itr != jointV.end();++itr) {
		if ((*itr).second.y() > highest_val) {
			highest_val = (*itr).second.y();
			extremes[0] = (*itr).first;
		}
		if ((*itr).second.y() < lowest_val) {
			lowest_val = (*itr).second.y();
			extremes[1] = (*itr).first;
		}
		if ((*itr).second.x() > rightmost_val) {
			rightmost_val = (*itr).second.x();
			extremes[2] = (*itr).first;
		}
		if ((*itr).second.x() < leftmost_val) {
			leftmost_val = (*itr).second.x();
			extremes[3] = (*itr).first;
		}
	}
}


KinectSkeleton::KinectSkeleton(const std::string& skelFile):m_skel_file(skelFile) {
	populateSkelFromFile(skelFile);
	
	//transform all to local frame of back-shoulders
	EigV3f upV = jointV["shoulders"] - jointV["back"]; upV.normalize();
	EigV3f forwardV = upV.cross(jointV["lshoulder"] - jointV["shoulders"]); forwardV.normalize();
	EigV3f rightV = upV.cross(forwardV);  rightV.normalize();
	m_back_frame << rightV , upV , forwardV; m_back_frame.transposeInPlace();
	
	m_skel_scale = 1.0 / (jointV["head"] - jointV["lankle"]).norm();
	
	m_back_offset = jointV["back"];
	transformJoints(m_back_frame,m_skel_scale,m_back_offset);
	
	makePinnocSkel();
	
	makeFramesForJoints();
}

KinectSkeleton::KinectSkeleton(const std::string& skelFile, const EigM3f& back_frame, double skel_scale, const EigV3f& back_offset):m_skel_file(skelFile) {
	populateSkelFromFile(skelFile);
	
	m_back_frame = back_frame;
	m_skel_scale = skel_scale;
	m_back_offset = jointV["back"];// back_offset;
	transformJoints(m_back_frame,m_skel_scale,m_back_offset);
	
	makePinnocSkel();
	
	makeFramesForJoints();
}