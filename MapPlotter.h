#ifndef MAP_PLOTTER_H
#define MAP_PLOTTER_H

#include <SFML/Graphics.hpp>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>
#include "Map.h"
#include "coord2D.h"

class MapPlotter {

	private:
		Map map; 
		sf::RenderWindow window;
		sf::Vector2u winSize;
		unsigned cellSize;
		std::vector<sf::Texture> robotTextures; // We need to keep the textures so they do not get deleted
		std::vector<sf::Sprite> robotSprites;
		
		void load_robot_images(const std::vector<std::string> &paths){
			for(auto path : paths){
				// Load the texture from a file
				sf::Texture texture;
				if (!texture.loadFromFile(path))
				{
					std::cerr << "ERROR: robot image" << path << "not found!" << std::endl;
				}else{
					// Create a sprite from the texture
					robotTextures.push_back(texture);
					
					sf::Sprite sprite(robotTextures.back());
					sprite.setScale(0.15,0.15);
					robotSprites.push_back(sprite);
				}
			}
		}

		void draw_robots(const std::vector<coord2D> &coords, const std::vector<float> &orientations){
			int size = coords.size() < robotSprites.size() ? coords.size() : robotSprites.size();

			for(int i = 0; i < size; ++i){
				
				// Set the position of the sprite centered in the window
				robotSprites[i].setPosition( (map.margin + coords[i].x + (winSize.x/cellSize-map.matrix.cols())/2) * cellSize, 
											 (map.margin + coords[i].y + (winSize.y/cellSize-map.matrix.rows())/2) * cellSize );
				//robotSprites[i].setPosition( coords[i].x, 
				//							 coords[i].y );
				
				// Set the rotation of the sprite (transforming from radians to degrees)
				// We set the origin of the sprite to make it rotate around its center
				robotSprites[i].setOrigin(robotSprites[i].getLocalBounds().width / 2.f, 
										  robotSprites[i].getLocalBounds().height / 2.f);
				robotSprites[i].setRotation(orientations[i]*360.0f/(2*M_PI));

				// Draw the sprite on top of the grid
				window.draw(robotSprites[i]);
			}
		}	
		
		void draw_cells(){
			for (int i = 0; i < map.matrix.rows(); ++i) {
				for (int j = 0; j < map.matrix.cols(); ++j) {
				    sf::RectangleShape cell(sf::Vector2f(cellSize, cellSize));
				    // Set the position of the sprite centered in the window
				    cell.setPosition( (j + (winSize.x/cellSize-map.matrix.cols())/2) * cellSize, 
				    				  (i + (winSize.y/cellSize-map.matrix.rows())/2) * cellSize );
					//cell.setPosition( j * cellSize, 
				    //				  i * cellSize );
				    if (map.matrix(i, j) == 1) {
				        cell.setFillColor(sf::Color::Black);
				    } else {
				        cell.setFillColor(sf::Color::White);
				    }
				    window.draw(cell);
				}
			}
		}
		
		void handle_events(){
			// View for resizing
        	sf::View view = window.getView();
        	
			// Event handling 
			sf::Event event;
			while (window.pollEvent(event)) {
			    if (event.type == sf::Event::Closed) {
			        window.close();
			    }else if (event.type == sf::Event::Resized) {
                            // resize my view
                            view.setSize({
                                    static_cast<float>(event.size.width),
                                    static_cast<float>(event.size.height)
                            });
                            window.setView(view);
                            winSize = window.getSize();
                }
			}
		}
		
	public:

		MapPlotter(const Map &map_matrix,
				   const int window_width, const int window_height, 
				   const unsigned cell_size,
				   const std::vector<std::string> &robot_imgs_paths = {}
				  ) : map(), window(sf::VideoMode(window_width, window_height), "Map"){
							  
			map = map_matrix;
			cellSize = cell_size;
			winSize = window.getSize();
			load_robot_images(robot_imgs_paths);
		}
		
		bool isOpen() {
			return window.isOpen();
		}
		
		sf::RenderWindow* getWindow() {
			return &window;
		}
		
		unsigned getCellSize() {
			return cellSize;
		}
		
		void drawElements(const std::vector<coord2D> &coords = {}, const std::vector<float> &orientations = {}) {
		
			handle_events();
			
			// Clear window
			window.clear(sf::Color::Black);
			
			// Draw elements
			draw_cells();
			draw_robots(coords, orientations);
			
		}
		
		void drawCircle(coord2D coord, int opacity=255, float size=5.0f, sf::Color color=sf::Color::Blue){
/*			sf::CircleShape circle(size);*/
			sf::CircleShape circle(opacity/20.0f);
			color.a = opacity;
			circle.setFillColor(color);
			circle.setPosition( (map.margin + coord.x + (winSize.x/cellSize-map.matrix.cols())/2) * cellSize,
							    (map.margin + coord.y + (winSize.y/cellSize-map.matrix.rows())/2) * cellSize );
							   
			window.draw(circle);
		}
		
		void update() {
			// Display window 
			window.display();
		}
		
		
};

#endif
