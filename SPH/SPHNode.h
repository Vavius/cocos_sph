//
//  SPHNode.h
//  SPH
//
//  Created by Vasiliy Yanushevich on 11/2/12.
//  Copyright 2012 Vasiliy Yanushevich. All rights reserved.
//

#import <Foundation/Foundation.h>
#import "cocos2d.h"
#import "Box2d.h"
#import "FluidHashList.h"
#import "GLES-Render.h"

// SPH node for simulating Smoothed-Particle hydrodynamics
// Individual particles are stored as struct, to keep things as simple as possible

// Everything is just a copy-paste of the ElectroDruid code from this post: http://www.box2d.org/forum/viewtopic.php?f=3&t=574&sid=0f208bac89ee07a05d5a524ef3b652cc&start=70

#define VERLET_INTEGRATION

#define nParticles		(500)
#define hashWidth		(40)
#define hashHeight		(40)

const int nominalNeighbourListLength = 256;

struct sParticle
{
	sParticle() : mPosition(0,0), mOldPosition(0,0), mVelocity(0,0),
    mAcceleration(0,0),	mForce(0,0), mMass(1.0f), mRestitution(1.0f), mFriction(0.0f) {}
	~sParticle() {}
    
	b2Vec2 mPosition;
	b2Vec2 mOldPosition;
	b2Vec2 mVelocity;
	b2Vec2 mAcceleration;
	b2Vec2 mForce;
    CCSprite *sp;
    
	// electrodruid TODO - these can probably be moved out as global values to save storing them
	float mMass;
	float mRestitution;
	float mFriction;
};

@interface SPHNode : CCNode {
    CCSpriteBatchNode *particle_sprites;
    
    b2World *m_world;
    
	GLESDebugDraw *m_debugDraw;		// strong ref
    
    // MAGIC NUMBERS
	float totalMass;
    
	float boxWidth;
	float boxHeight;
        
	float rad;
	float visc;
	float idealRad;
    
	//////////////////////////////////////////////////////////////////////////
    
	sParticle liquid[nParticles];
    
	// This buffer almost certainly doesn't need to be this big, but
	// better safe than sorry for now. Works much quicker as a statically-defined re-usable array
	// than as a std::vector though. It's a small memory hit but a reasonably big performance boost.
	float vlenBuffer[nParticles];
    
	// Speedy new custom linked-list thing
	cFluidHashList hashGridList[hashWidth][hashHeight];
    
	b2Body* bod;
    
	// This buffer almost certainly doesn't need to be this big, but
	// better safe than sorry for now
	b2Shape *mNeighboursBuffer[nominalNeighbourListLength];
}

@end
