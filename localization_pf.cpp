#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>

#include "MapPlotter.h"
#include "MapGenerator.h"
#include "Listener.h"
#include "ParticleFilter.h"

#include <chrono>
#include <thread> // For sleep_for() call

#define WINDOW_SIZE 1000
#define MAP_MARGIN 3
#define MAP_CELLS_PER_METRE 100


// Particle filter parameters
//const int NPART = 10000;  // Now we pass the number of particles as argument
const float SPEED_F = 0.22f; // meters/second
const float SPEED_B = 0.2f; // meters/second
const float SPEED_R = 2.0f*M_PI; // rads/second
const float COMMAND_DURATION = 1.0f/20.0f; // aproximation of command duration
const float S_X_F = 0.03f;
const float S_Y_F = 0.01f;
const float S_X_B = 0.02f;
const float S_Y_B = 0.01f;
const float S_ALPHA = 0.5f;
const float S_LIDAR = 0.2f;
const float LIDAR_MIN = 0.2f;
const float LIDAR_MAX = 2.0f;

void sleep(int t){

	std::this_thread::sleep_for(std::chrono::milliseconds(t));

}


int main( int narg, char *arg[] ) {

	if (narg < 2) {
		std::cout << "Provide the number of particles as argument." <<std::endl;
		return -1;
	}
	
	const int NPART(std::stoi(arg[1]));

	Listener listener;
	
	MapGenerator generator("pasillo.xml", MAP_MARGIN, MAP_CELLS_PER_METRE);
	Map map = generator.generateMap();
	
	MapPlotter map_plotter(map, WINDOW_SIZE, WINDOW_SIZE, MAP_MARGIN, {});
	
	ParticleFilter pf(NPART, map, SPEED_F, SPEED_B, SPEED_R, COMMAND_DURATION,
					  S_X_F, S_Y_F, S_X_B, S_Y_B, S_ALPHA, S_LIDAR, LIDAR_MIN, LIDAR_MAX);
	pf.randomize();
	
	float previous_lidar_sensor_data = 0.0f;
	
	while(map_plotter.isOpen()){
	
		//  Wait for next command from user
        std::string command = listener.receive(true);
        // Receive sensor data
        float lidar_sensor_data = std::stof(listener.receive(true))/1000.0f;
		//lidar_sensor_data = 0.0f;
        
        // Print out received message
//        std::cout << "Command received from client: " << command << std::endl;
//        std::cout << "Lidar sensor data: " << lidar_sensor_data << std::endl;
        
//        std::cout << "Command: " << command << std::endl;
//        std::cout << "Command size: " << command.size() << std::endl;
        
        //Register movement based on command
        if (command != "0") {
        
		    if (command=="1") {
		    	pf.move(GO_FORWARD);
		    } else if (command=="2") {
		    	pf.move(GO_BACK);
		    } else if (command=="3"){
		    	pf.move(TURN_LEFT);
		    } else if (command=="4"){
		    	pf.move(TURN_RIGHT);
		    }
		    
		    ////Update pf and resample
		    // If the lidar sensor gives value 0.0 or there is a huge change from the previous information
		    // there might have been an error with the read, so we will not use it
		    float lidar_data_variation = abs(lidar_sensor_data-previous_lidar_sensor_data);
		    if(lidar_sensor_data == 0.0f || lidar_data_variation > previous_lidar_sensor_data*0.2f) {
		    	std::cout << "Ignoring lidar information" << std::endl;
		    	pf.updateLikelihood();
		    } else {
		    	pf.updateLikelihood(lidar_sensor_data);
		    }

		    pf.resample();
		    previous_lidar_sensor_data = lidar_sensor_data;
		    
		    //// Draw changes on the map
		    
		    map_plotter.drawElements( {}, {} );
		    
		    // Read parameters from particles
		    std::vector<coord2D> coords;
		    //std::vector<float> orientations;
		    std::vector<int> opacities; // The opacity of a particle is proportional to its likelihood.
		    float max_likelihood = 0.0f;
		    for(auto p : pf.getParticles()) {
		    	coords.push_back(p.coord);
		    	//std::cout << "Likelihood:" << p.likelihood << std::endl;
		    	//orientations.push_back(p.alpha);
		    	opacities.push_back(p.likelihood);
		    	if(p.likelihood > max_likelihood) {
		    		max_likelihood = p.likelihood;
		    	}
		    }
		    // Normalize particle weight to be used as opacity value
		    if (max_likelihood > 0.0f) {
				for(auto it=opacities.begin(); it!=opacities.end(); ++it) {
					*it /= max_likelihood/255;
					if (*it < 100) {
						*it = 100;
					}
				}
		    }
		    
		    //Draw particles
		    for(int i=0; i<NPART; ++i) {
		    	map_plotter.drawCircle(coords[i], opacities[i]);
		    }
			
			map_plotter.update();
			
        } else {
//        	std::cout << "STOP" << std::endl;
        }
        
	}
	
	return 0;
}
