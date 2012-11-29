/*
 *  KinectPuppeteeringViewer.h
 *  KinectPuppeteering
 *
 *  Created by Roy Shilkrot on 11/24/12.
 *  Copyright 2012 MIT. All rights reserved.
 *
 */

#include <QFileDialog>
#include <QGLViewer/qglviewer.h>

#include "KinectSkeletonRigger.h"

class KinectPuppeteeringViewer : public QGLViewer
{
    Q_OBJECT

	KinectSkeletonRigger::Ptr rigger;
	
public:
    KinectPuppeteeringViewer(QWidget *parent = 0);
	
    protected :
    virtual void draw();
    virtual void init();
    virtual QString helpString() const;
	
    public slots:
	void LoadMesh() { rigger->LoadMesh(QFileDialog::getOpenFileName(this,tr("Open Mesh"), "", "*.obj").toStdString()); }
	void LoadSkeleton() { rigger->LoadSkeletonFromText(QFileDialog::getOpenFileName(this,tr("Open Skeleton"), "", "").toStdString()); }
	void LoadPose() { rigger->LoadPoseFromText(QFileDialog::getOpenFileName(this,tr("Open Skeleton"), "", "").toStdString()); }
	void AutoRig() { rigger->AutoRig(); }
	void SaveRig() { rigger->SaveRig(QFileDialog::getSaveFileName(this,tr("Save Rigging"), "rigging.txt", "*.txt").toStdString()); }
	void LoadRig() { rigger->LoadRig(QFileDialog::getOpenFileName(this,tr("Open Rigging"), "rigging.txt", "*.txt").toStdString()); }
signals:
	
};