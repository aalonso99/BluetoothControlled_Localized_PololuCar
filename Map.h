#ifndef MAP_H
#define MAP_H

struct Map {
	Eigen::MatrixXi matrix;
	int margin;
	int cellsPerMetre;
	
	Map() {}
	
	Map(Eigen::MatrixXi mat, int marg, int cpm): matrix(mat) {
		margin = marg;
		cellsPerMetre = cpm;
	}
	
	Map(const Map& map): matrix(map.matrix), margin(map.margin), cellsPerMetre(map.cellsPerMetre) {}
	
};

#endif
