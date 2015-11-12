#include "connectionspainter.h"
#include "myvehicle.h"

ConnectionsPainter::ConnectionsPainter() : vns::Painter() {}

ConnectionsPainter::~ConnectionsPainter() {}

void ConnectionsPainter::drawVehicle(vns::SceneViewer* w, const vns::Vehicle* vehicle, uint8 lod){	
	Painter::drawVehicle(w,vehicle,lod);

	// draw Connections
	const MyVehicle* myv = dynamic_cast<const MyVehicle*>(vehicle);
	vns::Vec p1 = vehicle->getPosition();

	setColor4f(1.0, 0.0, 0.0,0.8);
	vns::HashTable<int32, Connection*>::ConstIterator it(&myv->getConnections());
	while( it.isValid() ){
		Connection* con = it.value();
		drawLine(p1,con->getPosition());
		it.next();
	}
}
