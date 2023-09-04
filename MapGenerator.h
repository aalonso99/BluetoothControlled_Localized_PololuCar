// flood_fill functions adapted from https://www.geeksforgeeks.org/flood-fill-algorithm-implement-fill-paint/

#ifndef MAP_GENERATOR
#define MAP_GENERATOR

#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <climits>
#include <cassert>
#include "coord2D.h"
#include "SceneElement.h"
#include "Map.h"
#include "pugixml.hpp"

class MapGenerator {

	private:
		SceneElement scene;
		std::vector<SceneElement> scene_objects;
		std::vector<bool> drawn_objects;
		
		int margin = 0;
		int cellsPerMetre;
		
		int scene_width = 0;
		int scene_height = 0;
		int min_x = INT_MAX;
		int min_y = INT_MAX;
		int max_x = INT_MIN;
		int max_y = INT_MIN;
		
		Eigen::MatrixXi map; 
		
		void update_scene_width_height(){
			
			for (auto coord : scene.getShapeCoords()) {
				
				if(coord.x < min_x){
					min_x = coord.x;
				}
				
				if(coord.x > max_x){
					max_x = coord.x;
				}
				
				if(coord.y < min_y){
					min_y = coord.y;
				}
				
				if(coord.y > max_y){
					max_y = coord.y;
				}
			}
			
			scene_width = max_x-min_x;
			scene_height = max_y-min_y;
		}
		
		
		// A naive way of drawing line
		void naive_draw_line(const int x1, const int x2, const int y1, const int y2){
		
		 	//std::cout<<"x1:"<<x1<<" y1:"<<y1<<std::endl;
		 	//std::cout<<"x2:"<<x2<<" y2:"<<y2<<std::endl;
		 	
		 	if (x1 != x2){
		 		float m = (y2 - y1) / (x2 - x1);
		 		
		 		int sign = (x2 - x1)/abs(x2 - x1);
		 
				for (int x = x1; (x-x1)*sign <= (x2-x1)*sign; x+=sign) {
				
					int y = round(m*(x-x1) + y1);
					//std::cout<<"x:"<<x<<" y:"<<y<<std::endl;
					map(y,x) = 1;
				}
		 	}else{
		 		int sign = (y2 - y1)/abs(y2 - y1);
		 		for (int y = y1; (y-y1)*sign <= (y2-y1)*sign; y+=sign) {
		 			//std::cout<<"x:"<<x1<<" y:"<<y<<std::endl;
					map(y,x1) = 1;
				}
		 	}
			
			
			map(y2,x2) = 1;
		}
		
		// A recursive function to replace previous color 'prevC' at  '(x, y)'
		// and all surrounding pixels of (x, y) with new color 'newC' and
		void flood_fill_util(Eigen::MatrixXi &map, int x, int y, int prevC, int newC)
		{
			// Base cases
			if (x < 0 || x >= map.cols() || y < 0 || y >= map.rows())
				return;
			if (map(y,x) != prevC)
				return;
			if (map(y,x) == newC)
				return;
		 
			// Replace the color at (x, y)
			map(y,x) = newC;
		 
			// Recur for north, east, south and west
			flood_fill_util(map, x+1, y, prevC, newC);
			flood_fill_util(map, x-1, y, prevC, newC);
			flood_fill_util(map, x, y+1, prevC, newC);
			flood_fill_util(map, x, y-1, prevC, newC);
		}
		 
		// It mainly finds the previous color on (x, y) and
		// calls flood_fill_util()
		void flood_fill(Eigen::MatrixXi &map, int x, int y, int newC)
		{
			int prevC = map(y,x);
			
			if(prevC==newC) return;
			
			flood_fill_util(map, x, y, prevC, newC);
		}
		
	public:

		MapGenerator() {}
		
		// You can pass the scene contour from a list of coordinates...
		MapGenerator(std::vector<coord2D> &scene_coord_list, const int map_margin=0, const int cells_per_metre = 10): scene(scene_coord_list) {
			setMargin(map_margin);
			setCellsPerMetre(cells_per_metre);
			update_scene_width_height();
		}
		
		// ... or pass the scene and the objects/obstacles defined in an xml file.
		MapGenerator(const std::string &scene_xml, const int map_margin=0, const int cells_per_metre = 10) {
		
			// Set the margin and scale
			setMargin(map_margin);
			setCellsPerMetre(cells_per_metre);
   
			// load the XML file
			pugi::xml_document doc;
			assert(doc.load_file(scene_xml.c_str()));

			// Reads the countour of the scene
			pugi::xml_node tools = doc.child("SceneData").child("SceneContour");
			
			std::vector<coord2D> scene_coord_list({});
		   
			for (pugi::xml_node_iterator it = tools.begin(); it != tools.end(); ++it) {

				int x = (int)(std::stof( it->attribute("x").value() )*cellsPerMetre);
				int y = (int)(std::stof( it->attribute("y").value() )*cellsPerMetre);
				
				scene_coord_list.push_back(coord2D(x,y));
				
			}
			
			// TODO: Read the objects in the scene 
			
			// Set the scene parameters according to the coordinates
			setScene(scene_coord_list);
			update_scene_width_height();
			
		}
		
		void setScene(std::vector<coord2D> &scene_coord_list){
			scene = SceneElement(scene_coord_list);
			update_scene_width_height();
		}
		
		void setMargin(int map_margin){
			margin = map_margin;
		} 
		
		void setCellsPerMetre(int cells_per_metre) {
			cellsPerMetre = cells_per_metre;
		}
		
		void addObject(std::vector<coord2D> &object_coord_list){
			scene_objects.push_back(SceneElement(object_coord_list));
			drawn_objects.push_back(false);
		}
		
		// Generates the map matrix
		Map generateMap(){
			
			//std::cout<<"Height:"<<scene_height<<std::endl;
			//std::cout<<"Width:"<<scene_width<<std::endl;
			map = Eigen::MatrixXi::Zero(scene_height+2*margin, scene_width+2*margin);
			
			auto scene_coords = scene.getShapeCoords();
			
			for(int i=0; i<scene_coords.size()-1; ++i){
				// Draws line in map matrix
				naive_draw_line(margin+scene_coords[i].x-min_x, margin+scene_coords[i+1].x-min_x, 
							    margin+scene_coords[i].y-min_y, margin+scene_coords[i+1].y-min_y);
			}
			
			// Line between the last point and the first
			naive_draw_line(margin+scene_coords[scene_coords.size()-1].x-min_x, margin+scene_coords[0].x-min_x, 
					        margin+scene_coords[scene_coords.size()-1].y-min_y, margin+scene_coords[0].y-min_y);
					      
			// Fills the outside of the scene with 1s (not valid positions)
			// Might not work if margin=0 (TODO: correct it)
			flood_fill(map, 1, 1, 1);
			
			return Map(map, margin, cellsPerMetre);
		}
		
		//TODO
		// Adds the undrawn objects to the map
		Map updateMap(){
			return Map(map, margin, cellsPerMetre);
		}
		
		Map getMap(){
			return Map(map, margin, cellsPerMetre);
		}
		
};

#endif
