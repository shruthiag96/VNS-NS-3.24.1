#ifndef CONNECTIONSPAINTER_H_
#define CONNECTIONSPAINTER_H_

#include "vns/vns.h"

class ConnectionsPainter : public vns::Painter {
public:
	ConnectionsPainter();
	virtual ~ConnectionsPainter();

	void drawVehicle(vns::SceneViewer* w, const vns::Vehicle* vehicle, uint8 lod);
};

#endif
