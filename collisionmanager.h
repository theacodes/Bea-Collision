#ifndef __PONG_COLLISIONMANAGER_H__
#define __PONG_COLLISIONMANAGER_H__

#include "phoenix.h"

// forward decl for CollisionObject.
class CollisionObject;

//! A structure for storing the result of a collision.
struct CollisionResult{
	bool colliding;
	boost::intrusive_ptr<CollisionObject> a;
	boost::intrusive_ptr<CollisionObject> b;

	//! The minimun translation vector
	phoenix::Vector2d separation;
};


//! Polygon Collision Manager.
/*
	Manages collision objects and tests collisions between polygons.
*/
class CollisionManager
	: public phoenix::ResourceManager
{
public:

	CollisionManager()
		: ResourceManager(), autoupdate( true )
	{
		window_event_connection = phoenix::WindowManager::Instance()->listen( boost::bind( &CollisionManager::onWindowEvent, this, _1 ) );
	}

	virtual ~CollisionManager(){
		window_event_connection.disconnect();
	}

	virtual void onWindowEvent( const phoenix::WindowEvent& e ){
		if( autoupdate && e.type == phoenix::WET_UPDATE ){
			update();
		}
	}

	//! If set to true, it will automatically check for collisions when WET_UPDATE is signaled.
	void setAutoUpdate( const bool a ){ autoupdate = a; }

	//! Tests all objects against all others.
	void update();

	//! Tests two collision objects & fires signals.
	void test( boost::intrusive_ptr<CollisionObject> a, boost::intrusive_ptr<CollisionObject> b );

	//! Checks to see if two polygons are overlapping.
	/*!
		Also calculates the minimum translation vector.
		\param v The relative velocity of the two polygons.
	*/
	const CollisionResult test( const phoenix::Polygon& a, const phoenix::Polygon& b, const phoenix::Vector2d& v = phoenix::Vector2d(0,0) );

private:

	//! Projects a polygon onto an axis.
	void projectPolygon( const phoenix::Vector2d& axis, const phoenix::Polygon& poly, float& min, float& max );

	//! Calculates the distance between two intervals.
	float intervalDistance(float minA, float maxA, float minB, float maxB);

	//! Connection to window manager.
	boost::signals2::connection window_event_connection;

	//! If set to true, it will automatically check for collisions when WET_UPDATE is signaled.
	bool autoupdate;

};

#include "collisionobject.h"

#endif //__PONG_COLLISIONMANAGER_H__