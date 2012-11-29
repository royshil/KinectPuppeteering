/*
 *  KinectPuppeteeringViewer.cpp
 *  KinectPuppeteering
 *
 *  Created by Roy Shilkrot on 11/24/12.
 *  Copyright 2012 MIT. All rights reserved.
 *
 */

#include "KinectPuppeteeringViewer.h"

void KinectPuppeteeringViewer::draw()
{   
	//some 3D
	
	rigger->drawMesh(false);
	rigger->drawSkeleton();
	
//    glColor4f(1.0, 0.0, 0.0, 1.0);
//    drawText(20, 20, QString("here will be 3D!"), QFont("Helvetica [Cronyx]", 20));
}

void KinectPuppeteeringViewer::init()
{
	rigger.reset(new KinectSkeletonRigger);
	
    // Restore previous viewer state.
    restoreStateFromFile(); 
	
    setAxisIsDrawn(true);
	
    // Opens help window
 //   help();
}

QString KinectPuppeteeringViewer::helpString() const
{
    QString text("<h2>My QGLViewer</h2>");
	//.... more text?
    return text;
}

// Constructor must call the base class constructor.
KinectPuppeteeringViewer::KinectPuppeteeringViewer(QWidget *parent) : QGLViewer(parent)
{
    restoreStateFromFile();
//    help();
}