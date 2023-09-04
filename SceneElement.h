#ifndef SCENE_ELEMENT
#define SCENE_ELEMENT

#include <vector>
#include "coord2D.h"

// Scene element, represented by a sequence of coordinates. The last coordinate is assumed to be linked to the first
class SceneElement {

	private:
		std::vector<coord2D> shape_coords = {};
		
	public:

		SceneElement() {}
		
		SceneElement(std::vector<coord2D> &coord_list) {
			shape_coords = std::move(coord_list);
		}
		
		SceneElement operator=(SceneElement scene){
			shape_coords = scene.shape_coords;
			return *this;
		}
		
		void push_back(coord2D coord){
			shape_coords.push_back(coord);
		}
		
		void pop_back(){
			return shape_coords.pop_back();
		}
		
		std::vector<coord2D> getShapeCoords(){
			return shape_coords;
		}
		
};

#endif
