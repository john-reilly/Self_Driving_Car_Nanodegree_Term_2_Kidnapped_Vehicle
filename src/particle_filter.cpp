/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
  
  num_particles = 10 ; //starting low will try 10 , 100 , 1000
  default_random_engine gen ;
  // This is based on Lesson 14 Video 6 Gausian Sampling Quiz solution
  double std_x, std_y, std_theta ; // Standard deviations for x, y, and theta
  std_x = std[0] ;
  std_y = std[1] ;
  std_theta = std[2] ;
  
  // Again below is based on Lesson 14 Video 6 Gausian Sampling Quiz solution
  // This line creates a normal (Gaussian) distribution for x
  normal_distribution<double> dist_x(x, std_x);
  normal_distribution<double> dist_y(y, std_y);
  normal_distribution<double> dist_theta(theta, std_theta);
  
  // Again below is based on Lesson 14 Video 6 Gausian Sampling Quiz solution
  for (int i = 0; i < num_particles; i++) { // I note Tifany used ++i here ti count to 3 from 0 for 3 counts I will use i++
		double sample_x, sample_y, sample_theta;
		
		sample_x = dist_x(gen);
		sample_y = dist_y(gen);
		sample_theta = dist_theta(gen);
    
        //this follows the struct in partcile_filter.h
        Particle single_particle; //make new instance of Particle  and set class variables
        single_particle.id = i ; 
        single_particle.x = sample_x ;
        single_particle.y = sample_y ;
        single_particle.theta = sample_theta ;
        single_particle.weight = 1.0 ;
   		 //also from header file in partcile_filter.h
    	// add new particle weight to weightrs vector
		weights.push_back(1.0) ;
    	// add new particle to particles vector
    	particles.push_back(single_particle) ; 
    
	}
  
  is_initialized = true ;//set boolean 
  
  
  

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
  
  double std_x = std_pos[0] ;
  double std_y = std_pos[1] ;
  double std_theta = std_pos[2] ;
  
  default_random_engine gen ; //same as init
  
  for (int i = 0; i < num_particles; i++) {
  
    double predicted_x, predicted_y,predicted_theta ;
    if( yaw_rate == 0){
    	yaw_rate = 0.0001;  //this is to avoid a divide by zero and get the below equations to work
    }
    //equations from LEsson 14 section 9 : "Prediction step quiz explanation"
    double theta = particles[i].theta ;
    
    predicted_x = (velocity/yaw_rate ) * ( sin(theta + ( delta_t * yaw_rate))- sin(theta) );
    predicted_y = (velocity/yaw_rate ) * (cos(theta) - cos(theta + (delta_t * yaw_rate))  ) ;
    predicted_theta = yaw_rate * delta_t ;
    
    //create Gausians around predictions
    normal_distribution<double> dist_x(predicted_x, std_x);
    normal_distribution<double> dist_y(predicted_y, std_y);
    normal_distribution<double> dist_theta(predicted_theta, std_theta);
    
    // select and add a random gausian of the prediction and assign to particle in vector
    particles[i].x = dist_x(gen) ;
    particles[i].y =  dist_y(gen) ;
    particles[i].theta = dist_theta(gen) ;  
    
  }
  

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
  
  //plan:
  //set up gausian variables......done
  //loop through each particle...done
  //make a vector to store in range landmarks//
  //loop through landmarks and extract those within sensor range
  //associate onservations with landmarks
  //multivariate gausian part
  //set assocaitions
  
  
  double std_landmark_x = std_landmark[0] ;
  double std_landmark_y = std_landmark[1] ;
  // this loop goes through each particle one at a time
  for(int i = 0 ; i < num_particles ; i++){ 
    //this vector will store the subset of landmarks within range of the particle
    vector<LandmarkObs> landmarks_in_range ; 
    
    for(int j = 0 ; j < map_landmarks.landmark_list.size() ; j++){
      //extracting particle x and y and landmark x and y for distance evaluation
      float x_f = map_landmarks.landmark_list[j].x_f ;
      float x_y = map_landmarks.landmark_list[j].y_f ;
      float id_i = map_landmarks.landmark_list[j].id_i ;
      float px = particles[i].x ;
      float py = particles[i].y ;
      // if landmark within sensor reange add to vector
      if(sensor_range >= dist(px,py,x_f,x_y)){
        LandmarkObs new_in_range_landmark ;
        new_in_range_landmark.x = x_f ;
        new_in_range_landmark.y = x_y ;
        new_in_range_landmark.id = id_i ;//need to set this?
        landmarks_in_range.push_back(new_in_range_landmark);
      }//end of landmarks in range loop should have produced a vector of landmarks within range of particle
      
      
    }//end of partcile loop
    
    
    
  
  }
  
  
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
