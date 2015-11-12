#ifndef BEACON_H
#define BEACON_H

#include "ns3/nstime.h"
#include "iostream"

class Beacon {
public:

	Beacon(){};
	virtual ~Beacon(){};

	void SetId(int32_t id) { this->id=id; };
	int32 GetId(void) const { return id; };
	void SetX(double x) { this->x=x; };
	double GetX(void) const { return x; };
	void SetY(double y) { this->y=y; };
	double GetY(void) const { return y; };
	void SetSpeed(double speed) { this->speed=speed; };
	double GetSpeed(void) const { return speed; };
	void SetAccel(double accel) { this->accel=accel; };
	double GetAccel(void) const { return accel; };
	void SetHeading(double heading) { this->heading=heading; };
	double GetHeading(void) const { return heading; };
	void SetTimestamp(ns3::Time timestamp) { this->timestamp=timestamp; };
	ns3::Time GetTimestamp(void) const { return timestamp; };


private:
	int32_t id;
	double x;
	double y;
	double speed;
	double accel;
	double heading;
	ns3::Time timestamp;
};

#endif
