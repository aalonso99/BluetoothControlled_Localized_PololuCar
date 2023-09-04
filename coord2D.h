#ifndef COORD2D_H
#define COORD2D_H

struct floatCoord2D {
	float x;
	float y;
	
	floatCoord2D(float xx, float yy) {
		x = xx;
		y = yy;
	}
	
};

struct coord2D {
	int x;
	int y;
	
	coord2D(int xx, int yy) {
		x = xx;
		y = yy;
	}
	
	coord2D(floatCoord2D coord) {
		x = coord.x;
		y = coord.y; 
	}
};

#endif
