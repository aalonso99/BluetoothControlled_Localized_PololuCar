all: locpf loc

loc:
	g++ -o localization localization.cpp pugixml.cpp -lsfml-graphics -lsfml-window -lsfml-system -lzmq -std=c++11 -O2
	
locpf:
	g++ -o localization_pf localization_pf.cpp pugixml.cpp -lsfml-graphics -lsfml-window -lsfml-system -lzmq -fopenmp -std=c++11 -O2
