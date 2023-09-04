#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <vector>
#include <eigen3/Eigen/Dense>
#include <stdlib.h>     
#include <time.h> 
#include <random>
#include <cmath>
#include <omp.h>
#include "coord2D.h"

class RNGenerator {

	private:
		std::random_device rd;
		std::mt19937 gen;
				
		
	public:
		RNGenerator(): gen(rd()) {}
		
		int generateInt(int min, int max) {
			std::uniform_int_distribution<> dis(min, max);
			return dis(gen);
		}
		
		float generateFloat(float min, float max) {
			std::uniform_real_distribution<> dis(min, max);
			return dis(gen);
		}
		
		float generateNormal(float mean, float std) {
			std::normal_distribution<> nd(mean, std);
			return nd(gen);
		}
		
		// Returns the probability in [0,1] for a value as extreme as x on coming from a normal distribution with given parameters
		float probabilityPointNormalDistribution(float x, float mean, float std) {
			std::normal_distribution<> nd(mean, std);
			// Cumulative Density Function
			float cdf = (1.0f+std::erf((nd.mean() - x) / (nd.stddev() * std::sqrt(2.0f)))) / 2.0f;
			if (cdf < 0.5f) {
				return 2*cdf;
			} else {
				return 2*(1.0f-cdf);
			}
		}
		
		std::vector<unsigned> generateNFromDiscreteDistribution(unsigned n, const std::vector<float> &weights) {
			std::discrete_distribution<> dd(weights.begin(), weights.end());
			std::vector<unsigned> sample;
			sample.reserve(n);
			for(int i=0; i<n; ++i){
				unsigned x = dd(gen);
/*				std::cout<<"Sampled element weight: "<<weights[x]<<std::endl;*/
				sample.push_back(x);
			}
			
			return sample;
		}
};

enum Action {
	DO_NOTHING,
	GO_FORWARD,
	GO_BACK,
	TURN_LEFT,
	TURN_RIGHT
};

struct particle {
	floatCoord2D coord;
	float alpha;
	float likelihood = 1.0;
	
	particle(): coord(0.0f,0.0f) {
		alpha = 0.0f;
	}
	
	particle(float x, float y, float angle): coord(x,y) {
		alpha = angle;
	}
};

class ParticleFilter {
	
	private:
		// State variables (position and movement direction)
		std::vector<particle> particles;
		
		// Map
		Map map; 
		
		// Random number generator
		RNGenerator rng;
		
		//// MODEL PARAMETERS
		// Speed
		const float SPEED_F; //going forward
		const float SPEED_B; //going back
		const float SPEED_R; //rotating
		const float COMMAND_DURATION; // aproximation of command duration
		// Standard Deviation (sigma) going forward
		const float S_X_F; // deviation in the moving direction 
		const float S_Y_F; // deviation orthogonal to the moving direction 
		// Standard Deviation (sigma) going backward
		const float S_X_B; // deviation in the moving direction 
		const float S_Y_B; // deviation orthogonal to the moving direction 
		// Standard Deviation (sigma) while turning
		const float S_ALPHA;
		// Standard Deviation (sigma) of lidar sensor
		const float S_LIDAR;
		// Lidar min and max range
		const float LIDAR_MIN;
		const float LIDAR_MAX;
		
		void go_forward(particle& p) {
			const float DEVIATION_X = rng.generateNormal(0.0f, S_X_F);
			const float DEVIATION_Y = rng.generateNormal(0.0f, S_Y_F);
			
			const float dx = ((SPEED_F+DEVIATION_X)*cos(p.alpha) + DEVIATION_Y*cos(p.alpha+M_PI/2.0f))*COMMAND_DURATION*map.cellsPerMetre;
			const float dy = ((SPEED_F+DEVIATION_X)*sin(p.alpha) + DEVIATION_Y*sin(p.alpha+M_PI/2.0f))*COMMAND_DURATION*map.cellsPerMetre;

/*			const float dx = SPEED_F*cos(p.alpha)*COMMAND_DURATION*map.cellsPerMetre;*/
/*			const float dy = SPEED_F*sin(p.alpha)*COMMAND_DURATION*map.cellsPerMetre;*/
			
/*			std::cout << "dx:" << dx << " dy:" << dy << std::endl;*/
			
			p.coord.x += dx;
			p.coord.y += dy;
		}
		
		void go_back(particle& p) {
			const float DEVIATION_X = rng.generateNormal(0.0f, S_X_B);
			const float DEVIATION_Y = rng.generateNormal(0.0f, S_Y_B);
			
			const float dx = -((SPEED_B+DEVIATION_X)*cos(p.alpha) + DEVIATION_Y*cos(p.alpha+M_PI/2.0f))*COMMAND_DURATION*map.cellsPerMetre;
			const float dy = -((SPEED_B+DEVIATION_X)*sin(p.alpha) + DEVIATION_Y*sin(p.alpha+M_PI/2.0f))*COMMAND_DURATION*map.cellsPerMetre;

/*			const float dx = -SPEED_B*cos(p.alpha)*COMMAND_DURATION*map.cellsPerMetre;*/
/*			const float dy = -SPEED_B*sin(p.alpha)*COMMAND_DURATION*map.cellsPerMetre;*/
			
/*			std::cout << "dx:" << dx << " dy:" << dy << std::endl;*/
			
			p.coord.x += dx;
			p.coord.y += dy;
		}
		
		void turn_left(particle& p) {
			const float DEVIATION_ALPHA = rng.generateNormal(0.0f, S_ALPHA);
			const float dalpha = -(SPEED_R+DEVIATION_ALPHA)*COMMAND_DURATION;
/*			std::cout << "dalpha:" << dalpha << std::endl;*/
			p.alpha += dalpha;
		}
		
		void turn_right(particle& p) {
			const float DEVIATION_ALPHA = rng.generateNormal(0.0f, S_ALPHA);
			const float dalpha = (SPEED_R+DEVIATION_ALPHA)*COMMAND_DURATION;
/*			std::cout << "dalpha:" << dalpha << std::endl;*/
			p.alpha += dalpha;
		}
		
		bool valid_position(unsigned x, unsigned y) {
/*			std::cout << "Map value:" << map.matrix(y,x) << std::endl;*/
			return  x>0 && y>0 && x<map.matrix.cols() && y<map.matrix.rows() && map.matrix(y,x) == 0;
		}
		
		// Distance in the map from the particle to the closest wall in the moving direction 
		int calculate_simulation_distance(particle p, unsigned horizon_length) {
			unsigned x = (unsigned) p.coord.x;
			unsigned y = (unsigned) p.coord.y;
			const float dx = cos(p.alpha);
			const float dy = sin(p.alpha); 
			for (int i=2; (i*i*dx*dx+i*i*dy*dy)<(horizon_length*horizon_length); i+=2) {
				unsigned x_i = (unsigned)(x+i*dx);
				unsigned y_i = (unsigned)(y+i*dy);
				if(!valid_position(x_i, y_i)) {
					//std::cout << "Distance in map:" << (int)sqrt(i*i*dx*dx+i*i*dy*dy) << std::endl;
					return (int)sqrt(i*i*dx*dx+i*i*dy*dy);
				} 
			}
			
			return -1;
		}
		
		// Assigns a likelihood of 0 to every point further to an object than the minimum of the lidar range (in the moving direction)
		void remove_particles_far_from_object() {
			unsigned horizon_length = LIDAR_MIN*map.cellsPerMetre;
			#pragma omp parallel for num_threads(5) 
			for (auto& p : particles) {
				unsigned x = (unsigned) p.coord.x;
				unsigned y = (unsigned) p.coord.y;
				if (!valid_position(x,y)) {
					p.likelihood = 0.0f;
				} else {
					// If the particle can find a wall in the horizon, then likelihood is 1. It is 0 otherwise.
					p.likelihood = calculate_simulation_distance(p, horizon_length) != -1 ? p.likelihood : 0.0;
				}
			}
		}
		
		// Assigns a likelihood of 0 to every point closer to an object than the maximum of the lidar range (in the moving direction)
		void remove_particles_close_to_object() {
			unsigned horizon_length = LIDAR_MAX*map.cellsPerMetre;
			#pragma omp parallel for num_threads(5) 
			for (auto& p : particles) {
				unsigned x = (unsigned) p.coord.x;
				unsigned y = (unsigned) p.coord.y;
				if (!valid_position(x,y)) {
					p.likelihood = 0.0f;
				} else {
					// If the particle cannot find a wall in the horizon, then likelihood is 1. It is 0 otherwise.
					p.likelihood = calculate_simulation_distance(p, horizon_length) == -1 ? p.likelihood : 0.0;
				}
			}
		}
		
	public:
		
		ParticleFilter(unsigned npart, const Map &user_map, 
					   const float speed_f, const float speed_b, const float speed_r, const float cmd_duration,
					   const float s_x1, const float s_y1, 
					   const float s_x2, const float s_y2,
					   const float s_alpha, 
					   const float s_lidar, const float lidar_min, const float lidar_max):
					    
					   particles(npart),
					   map(user_map),
					   SPEED_F(speed_f), SPEED_B(speed_b), SPEED_R(speed_r), COMMAND_DURATION(cmd_duration),
					   S_X_F(s_x1), S_Y_F(s_y1), 
					   S_X_B(s_x2), S_Y_B(s_y2), 
					   S_ALPHA(s_alpha),
					   S_LIDAR(s_lidar), LIDAR_MIN(lidar_min), LIDAR_MAX(lidar_max) {}

		void randomize() {
		
			int width = map.matrix.cols();
			int height = map.matrix.rows();
			
			for(auto it=particles.begin(); it!=particles.end(); it++) {
				*it = particle(rng.generateFloat(0, width),
							   rng.generateFloat(0, height),
							   rng.generateFloat(0, 2.0f*M_PI));
			}
			
		}
		
		void move(Action action) {
			switch(action){
				case GO_FORWARD:
					for (auto& p : particles){
						go_forward(p);
					}
					break;
				case GO_BACK:
					for (auto& p : particles){
						go_back(p);
					}
					break;
				case TURN_LEFT:
					for (auto& p : particles){
						turn_left(p);
					}
					break;
				case TURN_RIGHT:
					for (auto& p : particles){
						turn_right(p);
					}
					break;
			}
		}
		
		void updateLikelihood(float lidar_read = NULL) {
			
			if (!lidar_read) {
				for (auto& p : particles) {
					unsigned x = (unsigned) p.coord.x;
					unsigned y = (unsigned) p.coord.y;
					if (!valid_position(x,y)) {
						p.likelihood = 0.0f;
					}
				}
			} else {
			
				if(lidar_read < LIDAR_MIN) {
				
					remove_particles_far_from_object();
					
				} else if (lidar_read >= LIDAR_MAX) {
					
					remove_particles_close_to_object();
					
				} 
				else {
					unsigned horizon_length = LIDAR_MAX*map.cellsPerMetre;
					#pragma omp parallel for num_threads(5) 
					for (auto& p : particles) {
						unsigned x = (unsigned) p.coord.x;
						unsigned y = (unsigned) p.coord.y;
						if (!valid_position(x,y)) {
							p.likelihood = 0.0f;
						} else {
							unsigned lidar_estimated_distance_ = lidar_read*map.cellsPerMetre;
							int simulation_distance = calculate_simulation_distance(p, horizon_length);
							
							p.likelihood *= rng.probabilityPointNormalDistribution(lidar_read*map.cellsPerMetre,
																				  simulation_distance,
																				  S_LIDAR*map.cellsPerMetre);
							//std::cout << "Normal Likelihood:" << p.likelihood << std::endl;
						}
					}
				}
			}
			
		}
		
		// Resample phase of the particle filter
		void resample() {
			int npart = particles.size();
			
			// We use the likelihood of each particle (or an increasing non linear function of it) as weight for the resampling
			std::vector<float> weights;
			weights.reserve(npart);
			for(auto& p : particles) {
				weights.push_back(p.likelihood*p.likelihood);
			}
			
			std::vector<unsigned> new_particle_indices = rng.generateNFromDiscreteDistribution(npart, weights);
			std::vector<particle> new_particles;
			float max_likelihood = 0.0f;
			for(int i=0; i<npart; ++i) {
				int j = new_particle_indices[i];
				particle new_particle = particles[j];
				new_particles.push_back(new_particle);
				
				if(new_particle.likelihood > max_likelihood) {
					max_likelihood = new_particle.likelihood;
				}
			}
			
			// Normalize particle weight
		    if (max_likelihood > 0.0f) {
				for(auto it=new_particles.begin(); it!=new_particles.end(); ++it) {
					it->likelihood /= max_likelihood;
				}
		    }
		    
/*		    std::cout << "Max weight: "<<max_likelihood << std::endl;*/
			
			particles = new_particles;
		}
		
		std::vector<particle> getParticles() {
			return particles;
		}
			
};

#endif
