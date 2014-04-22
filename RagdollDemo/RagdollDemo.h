/*
Bullet Continuous Collision Detection and Physics Library
RagdollDemo
Copyright (c) 2007 Starbreeze Studios

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Written by: Marten Svanfeldt
*/

#ifndef RAGDOLLDEMO_H
#define RAGDOLLDEMO_H

#include "GlutDemoApplication.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "GLDebugDrawer.h"

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;
class btHingeConstraint;

class RagdollDemo : public GlutDemoApplication
{

	btAlignedObjectArray<class RagDoll*> m_ragdolls;

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;
    // Assignment 5 ****************
    btRigidBody*    body[15];
    btCollisionShape*   geom[15];
    bool pause;
    // ************************
    
    // assignment 6 ****************
    btHingeConstraint* joints[13];
    bool oneStep = false;
    // *****************************
    
    // assignment 8 ****************
    
    int IDs[15];
    int timeStepGenerations;
    int bodyRotationFitness;
    
public:
    int touches[15];
    //btVector3 touchPoints[15];
    double weights[4][8]; // assignment 9
    long timeStep; // assignment 9
    
    
    // *****************************

public:
	void initPhysics();

	void exitPhysics();

	virtual ~RagdollDemo()
	{
		exitPhysics();
	}

	void spawnRagdoll(const btVector3& startOffset);

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	virtual void keyboardCallback(unsigned char key, int x, int y);

	static DemoApplication* Create()
	{
		RagdollDemo* demo = new RagdollDemo();
		demo->myinit();
		demo->initPhysics();
		return demo;
	}
    
    // Assignment 5 ****************
    void CreateBox( int index, double x, double y, double z, double length,
                   double width, double height);
    
    void CreateCylinder( int index, double x, double y, double z, double radius,
                        double length, double eulerX, double eulerY, double eulerZ);
    
    void DeleteObject(int index);
	// ***********************
    
    // Assignment 6 ***************
    
    btVector3 PointWorldToLocal( int bodyIndex, btVector3 point );
    
    btVector3 AxisWorldToLocal( int bodyIndex, btVector3 point );
    
    void CreateHinge( int jointIndex, int bodyAIndex, int bodyBIndex, const btVector3& pivotInA,
                     const btVector3& pivotInB, const btVector3& axisInA, const btVector3& axisInB );
    
    void DestroyHinge( int index );
    
    // ****************************
    
    // Assignment 7 ***************
    
    void actuateJoint(int jointIndex, double desiredAngle, double jointOffset, double timeStep);
    void actuateJoint2(int jointIndex, double desiredAngle, double jointOffset, double timeStep);
    
    // ****************************
    
    // Assignment 8 ***************
    
    virtual void renderme();
    
    // ****************************
    
    // Assignment 9 ***************
    
    void saveFitness(btRigidBody *body, btRigidBody *leftFrontLeg, btRigidBody *rightFrontLeg);
    
    // ****************************
    
    void calcRotationAngle(btRigidBody *body, btRigidBody *arm);
    
    btScalar calcDistance(btScalar x1Position, btScalar z1Position, btScalar x2Position, btScalar z2Position);

};


#endif
