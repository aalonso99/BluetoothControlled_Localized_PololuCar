#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>

#include "MapPlotter.h"
#include "MapGenerator.h"
#include "Listener.h"

#include <chrono>
#include <thread> // For sleep_for() call

#define WINDOW_SIZE 1000
#define MAP_MARGIN 3
#define MAP_CELLS_PER_METRE 100

const float SPEED_F = 0.22f; // meters/second
const float SPEED_B = 0.2f; // meters/second
const float SPEED_R = 2.0f*M_PI; // rads/second

const float COMMAND_DURATION = 1.0f/20.0f; // aproximation of command duration

void sleep(int t){

	std::this_thread::sleep_for(std::chrono::milliseconds(t));

}


int main() {

	Listener listener;
	
	MapGenerator generator("pasillo.xml", MAP_MARGIN, MAP_CELLS_PER_METRE);
	Map map = generator.generateMap();
	      
	MapPlotter map_plotter(map, WINDOW_SIZE, WINDOW_SIZE, MAP_MARGIN, {"./robot.png"});
	
	float alpha = 0.0f; //Assuming original angle of 0
	float x, y; 
	x=y=1.0f; // Position about the center of the room
	
	
	while(map_plotter.isOpen()){
	
		//  Wait for next command from user
        std::string command = listener.receive(true);
        // Receive sensor data
        std::string lidar_sensor_data = listener.receive(true);
        
        // Print out received message
        std::cout << "Command received from client: " << command << std::endl;
        std::cout << "Lidar sensor data: " + lidar_sensor_data << std::endl;
        
        //Register movement based on command
        if (command=="0"){}
        else if (command=="1") {
        	x += SPEED_F*cos(alpha)*COMMAND_DURATION;
        	y += SPEED_F*sin(alpha)*COMMAND_DURATION;
        } else if (command=="2") {
        	x -= SPEED_B*cos(alpha)*COMMAND_DURATION;
    		y -= SPEED_B*sin(alpha)*COMMAND_DURATION;
        } else if (command=="3"){
        	alpha -= SPEED_R*COMMAND_DURATION;
        } else if (command=="4"){
        	alpha += SPEED_R*COMMAND_DURATION;
        }

        
        // Transform position to map coordinates
        int x_map = (int)(x*map.cellsPerMetre);
        int y_map = (int)(y*map.cellsPerMetre);
        
        std::cout << "x: " << x << std::endl;
        std::cout << "y: " << y << std::endl;
        std::cout << "alpha: " << alpha << std::endl;
        std::cout << "x_map: " << x_map << std::endl;
        std::cout << "y_map: " << y_map << std::endl;
        
        // Draw changes on the map
		map_plotter.drawElements( {coord2D(x_map,y_map)}, {alpha} );
		//map_plotter.drawCircle({coord2D(100-i,100-j)});
		map_plotter.update();
		
		//sleep(10);
	}
	
	return 0;
}
