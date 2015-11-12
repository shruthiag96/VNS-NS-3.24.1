RM= rm -f
EXE = beaconing


all: $(EXE)

$(EXE): main.o connectionspainter.o mynetwork.o
	g++ -o $(EXE) main.o connectionspainter.o mynetwork.o -lvns -lvns-ns3 -lns3.24.1-core-debug -lns3.24.1-network-debug -lns3.24.1-internet-debug -lns3.24.1-mobility-debug -lns3.24.1-wifi-debug

main.o: mynetwork.h myvehicle.h connectionspainter.h
	g++ -c main.cpp

connectionspainter.o: connectionspainter.h myvehicle.h
	g++ -c connectionspainter.cpp

mynetwork.o: mynetwork.h beaconapplication.h beacon.h
	g++ -c mynetwork.cpp

clean:
	$(RM) *.o *~ $(EXE)
