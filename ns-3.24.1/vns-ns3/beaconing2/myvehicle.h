#ifndef MYVEHICLE_H_
#define MYVEHICLE_H_

#include "vns/vns.h"

class Connection {
public:
	Connection(int32 id, const vns::Vec& pos, double time) : id(id), pos(pos), time(time) {};
	virtual ~Connection() {};

	inline int32 getID() const { return id; }
	inline vns::Vec getPosition() const { return pos; }
	inline double getTime() const { return time; }

private:
	int32 id;
	vns::Vec pos;
	double time;
};

class MyVehicle: public vns::Vehicle {
public:
	MyVehicle(uint64 id,vns::VehicleType type, vns::VehicleModel model, const vns::Path::Iterator& path, float width, float length, vns::DriverModel* driver) : vns::Vehicle(id,type, model, path, width, length, driver) {}
	virtual ~MyVehicle() {}


	inline void addConnection(int32 id, const vns::Vec& pos, double time){ connections.insert(id, new Connection(id, pos, time)); };
	inline const vns::HashTable<int32, Connection*>& getConnections() const { return connections; };

	void simulationStep(vns::Simulator* sim) {
		vns::Vehicle::simulationStep(sim);
		double time = sim->getSimulationTime();

		vns::HashTable<int32, Connection*>::Iterator it(&connections);
		while( it.isValid() ) {
			Connection* con = it.value();
			int32 key = it.key();
			it.next();
			if(time >= con->getTime()+1.0){
				connections.remove(key);
				delete con;
			}
		}

	}

private:
	vns::HashTable<int32, Connection*> connections;
};

class MyVehicleFactory: public vns::VehicleFactory {
public:

	MyVehicleFactory() : vns::VehicleFactory(){};
	virtual ~MyVehicleFactory(){};

	MyVehicle* createVehicle(uint64 id,vns::VehicleType type, vns::VehicleModel model, const vns::Path::Iterator& path, float width, float length, vns::DriverModel* driver) {
		return new MyVehicle(id,type, model, path, width, length, driver);
	}
};

#endif
