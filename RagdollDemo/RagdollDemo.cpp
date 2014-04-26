/*
 Bullet Continuous Collision Detection and Physics Library
 Ragdoll Demo
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

#define CONSTRAINT_DEBUG_SIZE 0.2f


// include iostream to all for printing to the console
#include <iostream>
using namespace std;
#include <fstream>
#include "btBulletDynamicsCommon.h"
#include "GlutStuff.h"
#include "GL_ShapeDrawer.h"

#include "LinearMath/btIDebugDraw.h"

#include "GLDebugDrawer.h"
#include "RagdollDemo.h"

// assignment 9 ***********
#include <stdlib.h>
#include <time.h>
#include <stdio.h>
#include <cmath>
// ************************


// Enrico: Shouldn't these three variables be real constants and not defines?

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4     0.785398163397448309616
#endif

class RagDoll
{
	enum
	{
		BODYPART_PELVIS = 0,
		BODYPART_SPINE,
		BODYPART_HEAD,
        
		BODYPART_LEFT_UPPER_LEG,
		BODYPART_LEFT_LOWER_LEG,
        
		BODYPART_RIGHT_UPPER_LEG,
		BODYPART_RIGHT_LOWER_LEG,
        
		BODYPART_LEFT_UPPER_ARM,
		BODYPART_LEFT_LOWER_ARM,
        
		BODYPART_RIGHT_UPPER_ARM,
		BODYPART_RIGHT_LOWER_ARM,
        
		BODYPART_COUNT
	};
    
	enum
	{
		JOINT_PELVIS_SPINE = 0,
		JOINT_SPINE_HEAD,
        
		JOINT_LEFT_HIP,
		JOINT_LEFT_KNEE,
        
		JOINT_RIGHT_HIP,
		JOINT_RIGHT_KNEE,
        
		JOINT_LEFT_SHOULDER,
		JOINT_LEFT_ELBOW,
        
		JOINT_RIGHT_SHOULDER,
		JOINT_RIGHT_ELBOW,
        
		JOINT_COUNT
	};
    
	btDynamicsWorld* m_ownerWorld;
	btCollisionShape* m_shapes[BODYPART_COUNT];
	btRigidBody* m_bodies[BODYPART_COUNT];
	btTypedConstraint* m_joints[JOINT_COUNT];
    
	btRigidBody* localCreateRigidBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
	{
		bool isDynamic = (mass != 0.f);
        
		btVector3 localInertia(0,0,0);
		if (isDynamic)
			shape->calculateLocalInertia(mass,localInertia);
        
		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
		
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);
        
		m_ownerWorld->addRigidBody(body);
        
		return body;
	}
    
public:
	RagDoll (btDynamicsWorld* ownerWorld, const btVector3& positionOffset)
    : m_ownerWorld (ownerWorld)
	{
		// Setup the geometry
		m_shapes[BODYPART_PELVIS] = new btCapsuleShape(btScalar(0.15), btScalar(0.20));
		m_shapes[BODYPART_SPINE] = new btCapsuleShape(btScalar(0.15), btScalar(0.28));
		m_shapes[BODYPART_HEAD] = new btCapsuleShape(btScalar(4*0.10), btScalar(4*0.05));
		m_shapes[BODYPART_LEFT_UPPER_LEG] = new btCapsuleShape(btScalar(0.07), btScalar(0.45));
		m_shapes[BODYPART_LEFT_LOWER_LEG] = new btCapsuleShape(btScalar(0.05), btScalar(0.37));
		m_shapes[BODYPART_RIGHT_UPPER_LEG] = new btCapsuleShape(btScalar(0.07), btScalar(0.45));
		m_shapes[BODYPART_RIGHT_LOWER_LEG] = new btCapsuleShape(btScalar(0.05), btScalar(0.37));
		m_shapes[BODYPART_LEFT_UPPER_ARM] = new btCapsuleShape(btScalar(0.05), btScalar(0.33));
		m_shapes[BODYPART_LEFT_LOWER_ARM] = new btCapsuleShape(btScalar(0.04), btScalar(0.25));
		m_shapes[BODYPART_RIGHT_UPPER_ARM] = new btCapsuleShape(btScalar(0.05), btScalar(0.33));
		m_shapes[BODYPART_RIGHT_LOWER_ARM] = new btCapsuleShape(btScalar(0.04), btScalar(0.25));
        
		// Setup all the rigid bodies
		btTransform offset; offset.setIdentity();
		offset.setOrigin(positionOffset);
        
		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.), btScalar(1.), btScalar(0.)));
		m_bodies[BODYPART_PELVIS] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_PELVIS]);
        
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.), btScalar(1.2), btScalar(0.)));
		m_bodies[BODYPART_SPINE] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_SPINE]);
        
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.), btScalar(1.6), btScalar(0.)));
		m_bodies[BODYPART_HEAD] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_HEAD]);
        
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(-0.18), btScalar(0.65), btScalar(0.)));
		m_bodies[BODYPART_LEFT_UPPER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_UPPER_LEG]);
        
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(-0.18), btScalar(0.2), btScalar(0.)));
		m_bodies[BODYPART_LEFT_LOWER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_LOWER_LEG]);
        
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.18), btScalar(0.65), btScalar(0.)));
		m_bodies[BODYPART_RIGHT_UPPER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_UPPER_LEG]);
        
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.18), btScalar(0.2), btScalar(0.)));
		m_bodies[BODYPART_RIGHT_LOWER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_LOWER_LEG]);
        
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(-0.35), btScalar(1.45), btScalar(0.)));
		transform.getBasis().setEulerZYX(0,0,M_PI_2);
		m_bodies[BODYPART_LEFT_UPPER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_UPPER_ARM]);
        
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(-0.7), btScalar(1.45), btScalar(0.)));
		transform.getBasis().setEulerZYX(0,0,M_PI_2);
		m_bodies[BODYPART_LEFT_LOWER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_LOWER_ARM]);
        
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.35), btScalar(1.45), btScalar(0.)));
		transform.getBasis().setEulerZYX(0,0,-M_PI_2);
		m_bodies[BODYPART_RIGHT_UPPER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_UPPER_ARM]);
        
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.7), btScalar(1.45), btScalar(0.)));
		transform.getBasis().setEulerZYX(0,0,-M_PI_2);
		m_bodies[BODYPART_RIGHT_LOWER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_LOWER_ARM]);
        
		// Setup some damping on the m_bodies
		for (int i = 0; i < BODYPART_COUNT; ++i)
		{
			m_bodies[i]->setDamping(0.05, 0.85);
			m_bodies[i]->setDeactivationTime(0.8);
			m_bodies[i]->setSleepingThresholds(1.6, 2.5);
		}
        
		// Now setup the constraints
		btHingeConstraint* hingeC;
		btConeTwistConstraint* coneC;
        
		btTransform localA, localB;
        
		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.15), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.15), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_SPINE], localA, localB);
		hingeC->setLimit(btScalar(-M_PI_4), btScalar(M_PI_2));
		m_joints[JOINT_PELVIS_SPINE] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
        
		m_ownerWorld->addConstraint(m_joints[JOINT_PELVIS_SPINE], true);
        
        
		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,M_PI_2); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.30), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,M_PI_2); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_HEAD], localA, localB);
		coneC->setLimit(M_PI_4, M_PI_4, M_PI_2);
		m_joints[JOINT_SPINE_HEAD] = coneC;
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
        
		m_ownerWorld->addConstraint(m_joints[JOINT_SPINE_HEAD], true);
        
        
		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,-M_PI_4*5); localA.setOrigin(btVector3(btScalar(-0.18), btScalar(-0.10), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,-M_PI_4*5); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.225), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_LEFT_UPPER_LEG], localA, localB);
		coneC->setLimit(M_PI_4, M_PI_4, 0);
		m_joints[JOINT_LEFT_HIP] = coneC;
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
        
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_HIP], true);
        
		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(-0.225), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.185), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[BODYPART_LEFT_UPPER_LEG], *m_bodies[BODYPART_LEFT_LOWER_LEG], localA, localB);
		hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
		m_joints[JOINT_LEFT_KNEE] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
        
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_KNEE], true);
        
        
		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,M_PI_4); localA.setOrigin(btVector3(btScalar(0.18), btScalar(-0.10), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,M_PI_4); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.225), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_RIGHT_UPPER_LEG], localA, localB);
		coneC->setLimit(M_PI_4, M_PI_4, 0);
		m_joints[JOINT_RIGHT_HIP] = coneC;
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
        
		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_HIP], true);
        
		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(-0.225), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.185), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[BODYPART_RIGHT_UPPER_LEG], *m_bodies[BODYPART_RIGHT_LOWER_LEG], localA, localB);
		hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
		m_joints[JOINT_RIGHT_KNEE] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
        
		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_KNEE], true);
        
        
		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,M_PI); localA.setOrigin(btVector3(btScalar(-0.2), btScalar(0.15), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,M_PI_2); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.18), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_LEFT_UPPER_ARM], localA, localB);
		coneC->setLimit(M_PI_2, M_PI_2, 0);
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
        
		m_joints[JOINT_LEFT_SHOULDER] = coneC;
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_SHOULDER], true);
        
		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.18), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[BODYPART_LEFT_UPPER_ARM], *m_bodies[BODYPART_LEFT_LOWER_ARM], localA, localB);
        //		hingeC->setLimit(btScalar(-M_PI_2), btScalar(0));
		hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
		m_joints[JOINT_LEFT_ELBOW] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
        
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_ELBOW], true);
        
        
        
		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,0); localA.setOrigin(btVector3(btScalar(0.2), btScalar(0.15), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,M_PI_2); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.18), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_RIGHT_UPPER_ARM], localA, localB);
		coneC->setLimit(M_PI_2, M_PI_2, 0);
		m_joints[JOINT_RIGHT_SHOULDER] = coneC;
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
        
		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_SHOULDER], true);
        
		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.18), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[BODYPART_RIGHT_UPPER_ARM], *m_bodies[BODYPART_RIGHT_LOWER_ARM], localA, localB);
        //		hingeC->setLimit(btScalar(-M_PI_2), btScalar(0));
		hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
		m_joints[JOINT_RIGHT_ELBOW] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
        
		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_ELBOW], true);
	}
    
	virtual	~RagDoll ()
	{
		int i;
        
		// Remove all constraints
		for ( i = 0; i < JOINT_COUNT; ++i)
		{
			m_ownerWorld->removeConstraint(m_joints[i]);
			delete m_joints[i]; m_joints[i] = 0;
		}
        
		// Remove all bodies and shapes
		for ( i = 0; i < BODYPART_COUNT; ++i)
		{
			m_ownerWorld->removeRigidBody(m_bodies[i]);
			
			delete m_bodies[i]->getMotionState();
            
			delete m_bodies[i]; m_bodies[i] = 0;
			delete m_shapes[i]; m_shapes[i] = 0;
		}
	}
};

// assignment 8 *********************************

static RagdollDemo* ragdollDemo;

bool myContactProcessedCallback(btManifoldPoint& cp, void* body0, void* body1)
{
    int *ID1, *ID2;
    btCollisionObject* o1 = static_cast<btCollisionObject*>(body0);
    btCollisionObject* o2 = static_cast<btCollisionObject*>(body1);
    
    ID1 = static_cast<int*>(o1->getUserPointer());
    ID2 = static_cast<int*>(o2->getUserPointer());
    
    //printf("ID1 = %d, ID2 = %d\n", *ID1, *ID2);
    
    ragdollDemo->touches[*ID1] = 1;
    ragdollDemo->touches[*ID2] = 1;
    /*
    ragdollDemo->touchPoints[*ID1] = cp.m_positionWorldOnB;
    ragdollDemo->touchPoints[*ID2] = cp.m_positionWorldOnB;
*/
    return false;
}

// ***********************************************


void RagdollDemo::initPhysics()
{
    printf("hello1\n");
    
    /*
    weights = new double*[4];
    
    for(int i = 0 ; i < 4 ; i++)
    {
        double* SubArray = NULL;
        SubArray = new double[9];
        
        for(int j = 0 ; j < 9 ; j++)
        {
            SubArray[j] = (float)j;
        }
        
        weights[i] = SubArray;
    }
    */

    
    
    /*
    weights = (double **) malloc(4*sizeof(double *));
    for(int i = 0; i < 4; i++)
    {
        weights[i]=(double *) malloc(9*sizeof(double));
    }
    */
    
    
    timeStep = 0;
    timeStepGenerations = 0;
    bodyRotationFitness = 0;
    
    // read weights from file and store in matrix ****************
    ifstream synapseFile;
    
    synapseFile.open("/Users/Ryan1/UVM/cs206/Final_Project/weights.dat");
    //double w;
    int count = 1;
    if (synapseFile.is_open())
    {
        for (int i=0; i<4; i++)
        {
            for (int j=0; j<9; j++)
            {
                synapseFile >> weights[i][j];
                //cout << count << ": " << weights[i][j] << "\n";
                printf("%d: %f\n", count, weights[i][j]);
                count++;
            }
        }
        synapseFile.close();
        //char ch = getchar();
    }
    else
    {
        cout << "unable to open file\n";
        exit(0);
    }
    
    // assignment 8 ***************
    
    ragdollDemo = this;
    gContactProcessedCallback = myContactProcessedCallback;
    
    // ****************************
    
    // Setup the basic world
    
	setTexturing(true);
	setShadows(true);
    
	setCameraDistance(btScalar(7.5));
    
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
    
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
    
	btVector3 worldAabbMin(-10000,-10000,-10000);
	btVector3 worldAabbMax(10000,10000,10000);
	m_broadphase = new btAxisSweep3 (worldAabbMin, worldAabbMax);
    
	m_solver = new btSequentialImpulseConstraintSolver;
    
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	//m_dynamicsWorld->getDispatchInfo().m_useConvexConservativeDistanceUtil = true;
	//m_dynamicsWorld->getDispatchInfo().m_convexConservativeDistanceThreshold = 0.01f;
    
	// Setup a big ground box
	{
		btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(200.),btScalar(10.),btScalar(200.)));
		m_collisionShapes.push_back(groundShape);
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,-10,0));
        
#define CREATE_GROUND_COLLISION_OBJECT 1
#ifdef CREATE_GROUND_COLLISION_OBJECT
		btCollisionObject* fixedGround = new btCollisionObject();
		fixedGround->setCollisionShape(groundShape);
		fixedGround->setWorldTransform(groundTransform);
        fixedGround->setUserPointer(&IDs[15]); // assignment 8
		m_dynamicsWorld->addCollisionObject(fixedGround);
#else
		localCreateRigidBody(btScalar(0.),groundTransform,groundShape);
#endif //CREATE_GROUND_COLLISION_OBJECT
        
	}
    
	// Spawn one ragdoll
	btVector3 startOffset(1,0.5,0);
	//spawnRagdoll(startOffset);
	startOffset.setValue(-1,0.5,0);
	//spawnRagdoll(startOffset);
    
    // &%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&%&
    printf("hello2\n");
    // Assignment 8 ****************
    
    for ( int i=0; i < 16; i++)
    {
        IDs[i] = i;
        //cout << i << "\n";
    }
    printf("hello3\n");
    // ASSIGNMENT 5 CODE ***********************************
    
    CreateBox(0, 0., 1., 0., 2., 0.15, 1.); // main body
    CreateCylinder(1, 1.5, 1., 1.5, 0.15, 1., M_PI_2, 0., 0.); //left back upper leg
    CreateCylinder(2, 1.5, 1., -1.5, 0.15, 1., M_PI_2, 0., 0.); //right back upper leg
    CreateCylinder(3, -1., 1., 1.5, 0.15, 1., M_PI_2, 0., 0.); //left front upper leg
    CreateCylinder(4, -1., 1., -1.5, 0.15, 1., M_PI_2, 0., 0.); // right front upper leg
    CreateCylinder(5, 1.5, 0.5, 2., 0.15, 1., 0., 0., 0.); //left back lower leg
    CreateCylinder(6, 1.5, 0.5, -2., 0.15, 1., 0., 0., 0.); //right back lower leg
    CreateCylinder(7, -1., 0.5, 2., 0.15, 1., 0., 0., 0.); //left front lower leg
    CreateCylinder(8, -1., 0.5, -2., 0.15, 1., 0., 0., 0.); //right front lower leg
    CreateCylinder(9, -2.25, 1., 0., 0.15, 0.5, 0., 0., M_PI_2); // front arm
    CreateCylinder(10, -2.625, 1., -0.075, 0.075, 0.25, 0., 0., M_PI_2); // right lower claw
    CreateCylinder(11, -2.625, 1., 0.075, 0.075, 0.25, 0., 0., M_PI_2); // left lower claw
    CreateCylinder(12, -2.875, 1., -0.075, 0.075, 0.25, 0., 0., M_PI_2); // right upper claw
    CreateCylinder(13, -2.875, 1., 0.075, 0.075, 0.25, 0., 0., M_PI_2); // left upper claw
    CreateCylinder(14, -6., 1., 0., 0.15, 1.5, 0., 0., 0.); // free standing object
    
    printf("hello4\n");
    
    //create the hinges
    //
    //back upper right leg to back lower right leg
    CreateHinge(0, 2, 6, PointWorldToLocal(2, btVector3(1.5, 1., -2.)),
                PointWorldToLocal(6, btVector3(1.5, 1., -2.)),
                AxisWorldToLocal(2, btVector3(-1., 0., 0.)),
                AxisWorldToLocal(6, btVector3(-1., 0., 0.)));
    //
    //back upper left leg to back lower left leg
    CreateHinge(1, 1, 5, PointWorldToLocal(1, btVector3(1.5, 1., 2.)),
                PointWorldToLocal(5, btVector3(1.5, 1., 2.)),
                AxisWorldToLocal(1, btVector3(-1., 0., 0.)),
                AxisWorldToLocal(5, btVector3(-1., 0., 0.)));
    
    //left upper front leg to left lower front leg
    CreateHinge(2, 3, 7, PointWorldToLocal(3, btVector3(-1., 1., 2.)),
                PointWorldToLocal(7, btVector3(-1., 1., 2.)),
                AxisWorldToLocal(3, btVector3(-1., 0., 0.)),
                AxisWorldToLocal(7, btVector3(-1., 0., 0.)));
    
    //right upper front leg to right lower front leg
    CreateHinge(3, 4, 8, PointWorldToLocal(4, btVector3(-1.5, 1., -2.)),
                PointWorldToLocal(8, btVector3(-1.5, 1., -2.)),
                AxisWorldToLocal(4, btVector3(-1., 0., 0.)),
                AxisWorldToLocal(8, btVector3(-1., 0., 0.)));
    //
    //back upper right leg to main body
    CreateHinge(4, 0, 2, PointWorldToLocal(0, btVector3(1.5, 1., -1.)),
                PointWorldToLocal(2, btVector3(1.5, 1., -1.)),
                AxisWorldToLocal(0, btVector3(0., 1., 0.)),
                AxisWorldToLocal(2, btVector3(0., 1., 0.)));
    //
    //back upper left leg to main body
    CreateHinge(5, 0, 1, PointWorldToLocal(0, btVector3(1.5, 1., 1.)),
                PointWorldToLocal(1, btVector3(1.5, 1., 1.)),
                AxisWorldToLocal(0, btVector3(0., 1., 0.)),
                AxisWorldToLocal(1, btVector3(0., 1., 0.)));
    
    //left upper front leg to main body
    CreateHinge(6, 0, 3, PointWorldToLocal(0, btVector3(-1., 1., 1.)),
                PointWorldToLocal(3, btVector3(-1., 1., 1.)),
                AxisWorldToLocal(0, btVector3(0., 1., 0)),
                AxisWorldToLocal(3, btVector3(0., 1., 0)));
    
    //right upper front leg to main body
    CreateHinge(7, 0, 4, PointWorldToLocal(0, btVector3(-1., 1., -1.)),
                PointWorldToLocal(4, btVector3(-1., 1., -1.)),
                AxisWorldToLocal(0, btVector3(0., 1., 0)),
                AxisWorldToLocal(4, btVector3(0., 1., 0.)));
    
    //front arm to main body
    CreateHinge(8, 0, 9, PointWorldToLocal(0, btVector3(-2., 1., 0.)),
                PointWorldToLocal(9, btVector3(-2., 1., 0.)),
                AxisWorldToLocal(0, btVector3(0., 0., 1.)),
                AxisWorldToLocal(9, btVector3(0., 0., 1.)));
    
    //right lower claw to arm
    CreateHinge(9, 9, 10, PointWorldToLocal(9, btVector3(-2.5, 1., -0.075)),
                PointWorldToLocal(10, btVector3(-2.5, 1., -0.075)),
                AxisWorldToLocal(9, btVector3(0., 1., 0.)),
                AxisWorldToLocal(10, btVector3(0., 1., 0.)));
    
    //left lower claw to arm
    CreateHinge(10, 9, 11, PointWorldToLocal(9, btVector3(-2.5, 1., 0.075)),
                PointWorldToLocal(11, btVector3(-2.5, 1., 0.075)),
                AxisWorldToLocal(9, btVector3(0., 1., 0.)),
                AxisWorldToLocal(11, btVector3(0., 1., 0.)));
    
    //right lower claw to right upper claw
    CreateHinge(11, 10, 12, PointWorldToLocal(10, btVector3(-2.75, 1., -0.075)),
                PointWorldToLocal(12, btVector3(-2.75, 1., -0.075)),
                AxisWorldToLocal(10, btVector3(0., 1., 0.)),
                AxisWorldToLocal(12, btVector3(0., 1., 0.)));
    
    //right lower claw to right upper claw
    CreateHinge(12, 11, 13, PointWorldToLocal(11, btVector3(-2.75, 1., 0.075)),
                PointWorldToLocal(13, btVector3(-2.75, 1., 0.075)),
                AxisWorldToLocal(11, btVector3(0., 1., 0.)),
                AxisWorldToLocal(13, btVector3(0., 1., 0.)));
    
    // *****************************************************
    
	clientResetScene();
    printf("hello5\n");
}

void RagdollDemo::CreateBox(int index, double x, double y, double z, double length, double width, double height)
{
    btTransform t;
    t.setIdentity();
    t.setOrigin(btVector3(btScalar(x), btScalar(y), btScalar(z)));
    
    geom[index] = new btBoxShape(btVector3(btScalar(length), btScalar(width), btScalar(height)));
    body[index] = localCreateRigidBody(btScalar(1.0), t, geom[index]);
    
    body[index]->setUserPointer(&IDs[index]); //assignment 8
}

void RagdollDemo::CreateCylinder(int index, double x, double y, double z, double radius, double length, double eulerX,
                                 double eulerY, double eulerZ)
{
    btTransform t;
    t.setIdentity();
    t.setOrigin(btVector3(btScalar(x), btScalar(y), btScalar(z)));
    t.getBasis().setEulerZYX(eulerX, eulerY, eulerZ);
    
    geom[index] = new btCylinderShape(btVector3(btScalar(radius), btScalar(length/2), btScalar(radius)));
    body[index] = localCreateRigidBody(btScalar(1.0), t, geom[index]);
    
    body[index]->setUserPointer(&IDs[index]); //assignment 8
    
}

void RagdollDemo::DeleteObject(int index)
{
    delete body[index];
    delete geom[index];
}

// Assignment 6 *************************

btVector3 RagdollDemo::PointWorldToLocal(int index, btVector3 point)
{
    btTransform t = body[index]->getCenterOfMassTransform().inverse();
    
    btVector3 local = t*point;
    return local;
}

btVector3 RagdollDemo::AxisWorldToLocal(int index, btVector3 axis)
{
    btTransform t = body[index]->getCenterOfMassTransform().inverse();
    
    btVector3 zero(0,0,0);
    t.setOrigin(zero);
    
    btVector3 local = t * axis;
    
    return local;
}

void RagdollDemo::CreateHinge(int jointIndex, int bodyAIndex, int bodyBIndex, const btVector3 &pivotInA,
                              const btVector3 &pivotInB, const btVector3 &axisInA, const btVector3 &axisInB)
{
    btHingeConstraint* hinge = new btHingeConstraint(*body[bodyAIndex], *body[bodyBIndex], pivotInA,
                                                     pivotInB, axisInA, axisInB);
    
    
    // ***************
    // knee joints
    if (jointIndex >= 0 && jointIndex <= 3)
    {
        hinge->setLimit(-M_PI, 0);
    }
    // hip joints
    else if (jointIndex >= 4 && jointIndex <= 7)
    {
        hinge->setLimit(-M_PI_2, M_PI_2);
    }
    // shoulder joint
    else if (jointIndex == 8)
    {
        hinge->setLimit(-M_PI_2, 0);
    }
    else if (jointIndex == 9)
    {
        hinge->setLimit(M_PI_4, M_PI_4);
    }
    else if (jointIndex == 10)
    {
        hinge->setLimit(-M_PI_4, -M_PI_4);
    }
    else if (jointIndex == 11)
    {
        hinge->setLimit(-M_PI_2, 0);
    }
    else if (jointIndex == 12)
    {
        hinge->setLimit(0, M_PI_2);
    }
    
    
    // ****************************

    joints[jointIndex] = hinge;
    
    m_dynamicsWorld->addConstraint(hinge, true);
}

// ****************************

void RagdollDemo::DestroyHinge(int index)
{
    delete joints[index];
}

// ****************************

void RagdollDemo::actuateJoint2(int jointIndex, double desiredAngle, double jointOffset, double timeStep)
{
    
    desiredAngle = desiredAngle + jointOffset;
    
    btScalar currentAngle = joints[jointIndex]->getHingeAngle();
    //cout << "current angle " << jointIndex << ": " << currentAngle << "\n";
    //cout << "desired angle " << jointIndex << ": " << desiredAngle << "\n";
    
    btScalar diff = desiredAngle - currentAngle;
    diff = 6*diff;
    double maxImpulse = abs(diff*10);
    
    //cout << "diff: " << diff << "\nmaxImpulse: " << maxImpulse << "\n\n";
    
    joints[jointIndex]->enableAngularMotor(true, diff, maxImpulse);
}

// Assignment 8 *************************

void RagdollDemo::renderme()
{
    
    extern GLDebugDrawer gDebugDrawer;
    
    // call parent method
    GlutDemoApplication::renderme();
    
    for ( int i=0; i<15; i++)
    {
        if ( touches[i] == 1 )
        {
            //btVector3 position = touchPoints[i];
            
            //gDebugDrawer.drawSphere(position, 0.2, btVector3(1.,0.,0.));
        }
    }
    
    //gDebugDrawer.drawSphere(btVector3(0.,0.,0.), 0.9, btVector3(1.,0.,0.));
    
}

// **************************************

// assignment 10 *************************

void RagdollDemo::saveFitness(btRigidBody *body, btRigidBody *arm, btRigidBody *object)
{
    btVector3 bodyPosition = body->getCenterOfMassPosition();
    btVector3 armPosition = arm->getCenterOfMassPosition();
    btVector3 objectPosition = object->getCenterOfMassPosition();
    
    btScalar distance = calcDistance(armPosition.getX(), armPosition.getZ(), objectPosition.getX(), objectPosition.getZ());
    //minimize distance between arm and object
    cout << "calculated distance: " << distance << "\n";
    distance = 1 / (1 + distance);

    //btScalar bodyZPos = abs(bodyPosition.getZ());
    //cout << "body's z position: " << bodyZPos << "\n";
    //bodyZPos = 1 / (1 + bodyZPos);
    //cout << "body's minimized z position: " << bodyZPos << "\n";
    
    btScalar fitness = distance;
    cout << "fitness: " << fitness << "\n";
    
    ofstream fitsFile;
    fitsFile.open("/Users/Ryan1/UVM/cs206/Final_Project/fits.dat");
    if (fitsFile.is_open())
    {
        fitsFile << fitness << "\n";
        fitsFile.close();
    }
    else
    {
        cout << "unable to open fits file\n";
        exit(0);
    }
    
}

// ***************************************

btScalar RagdollDemo::calcDistance(btScalar x1Position, btScalar z1Position, btScalar x2Position, btScalar z2Position)
{
    //cout << "1: ( " << x1Position << ", " << z1Position << " )\n";
    //cout << "2: ( " << x2Position << ", " << z2Position << " )\n";
    
    btScalar distance = (x2Position - x1Position) * (x2Position - x1Position);
    distance = distance + ( (z2Position - z1Position) * (z2Position - z1Position) );
    distance = sqrt(distance);
    
    return distance;
}

// ***************************************

void RagdollDemo::spawnRagdoll(const btVector3& startOffset)
{
	RagDoll* ragDoll = new RagDoll (m_dynamicsWorld, startOffset);
	m_ragdolls.push_back(ragDoll);
}

void RagdollDemo::clientMoveAndDisplay()
{
    printf("hello6\n");
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();
    
	float minFPS = 100000.f/60.f;
	if (ms > minFPS)
		ms = minFPS;
    
    // assignment 8 ******************
    
    for (int i=0; i<15; i++)
    {
        touches[i] = 0;
    }
    printf("hello7\n");
    // *******************************
    
	//if (m_dynamicsWorld)
    if (!pause || (pause && oneStep))
	{
        m_dynamicsWorld->stepSimulation(ms / 100000.f);
        //optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
        
        oneStep = false;
        printf("hello8\n");
        if (timeStep % 10 == 0)
        {
            for (int i=0; i<9; i++)
            {
                double motorCommand = 0.0;
                
                for (int j=0; j<4; j++)
                {
                    //cout << weights[j][i] << " err\n";
                    
                    motorCommand = motorCommand + (touches[j+5] * weights[j][i]);
                    cout << "touchSensor " << j+5 << ": " << touches[j+5] << "\n";
                }
 
                motorCommand = tanh(motorCommand);
                motorCommand = motorCommand * M_PI_4;
                
                //cout << "motorCommand " << i << ": " << motorCommand << "\n";
                
                if (i >= 4 && i <= 7)
                {
                    actuateJoint2(i, motorCommand, 0, ms / 100000.f);
                }
                else if ( i == 8 )
                {
                    actuateJoint2(i, motorCommand, -M_PI_4, ms / 100000.f);
                }
                else
                {
                    actuateJoint2(i, motorCommand, -M_PI_2, ms / 100000.f);
                }
                
                printf("hello9\n");
            }
        }
        
        timeStep++;
        //printf("%d\n", timeStep);

	}
    printf("hello10 \n");
	renderme();
    
	glFlush();
    
	glutSwapBuffers();
    
     if (timeStepGenerations == 1000)
     {
         saveFitness(body[0], body[3], body[4]);
         exit(0);
     }
    
    
    //assignment 9 ******
    timeStepGenerations++;
    //cout << "timestepGeneration: " << timeStepGenerations << "\n";
    
    // ********
}

void RagdollDemo::displayCallback()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
	renderme();
    
	//optional but useful: debug drawing
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();
    
	glFlush();
	glutSwapBuffers();
}

void RagdollDemo::keyboardCallback(unsigned char key, int x, int y)
{
	switch (key)
	{
        case 'e':
		{
            btVector3 startOffset(0,2,0);
            spawnRagdoll(startOffset);
            break;
		}
            // Assignment 5 ************
        case 'p':
        {
            pause = !pause;
        }
            // **********************
            // Assignment 6 ***********
        case 'n':
        {
            oneStep = true;
        }
            // ************************
        default:
            DemoApplication::keyboardCallback(key, x, y);
	}
    
	
}



void RagdollDemo::exitPhysics()
{
    
    for (int i=0; i<14; i++)
    {
        DestroyHinge(i);
    }
    
    for (int i=0; i<15; i++)
    {
        DeleteObject(i);
    }
    
	int i;
    
	for (i=0;i<m_ragdolls.size();i++)
	{
		RagDoll* doll = m_ragdolls[i];
		delete doll;
	}
    
	//cleanup in the reverse order of creation/initialization
    
	//remove the rigidbodies from the dynamics world and delete them
	
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}
    
	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}
    
	//delete dynamics world
	delete m_dynamicsWorld;
    
	//delete solver
	delete m_solver;
    
	//delete broadphase
	delete m_broadphase;
    
	//delete dispatcher
	delete m_dispatcher;
    
	delete m_collisionConfiguration;
    
	
}
