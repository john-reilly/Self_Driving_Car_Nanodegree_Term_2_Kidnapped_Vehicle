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
  cout << "in Init line 27" ;
  num_particles = 10 ; //starting low will try 10 , 100 , 1000
  default_random_engine gen ;
  // This is based on Lesson 14 Video 6 Gausian Sampling Quiz solution
  double std_x, std_y, std_theta ; // Standard deviations for x, y, and theta
  std_x = std[0] ;
  std_y = std[1] ;
  std_theta = std[2] ;
  cout << "in Init line 35" ;
  // Again below is based on Lesson 14 Video 6 Gausian Sampling Quiz solution
  // This line creates a normal (Gaussian) distribution for x
  normal_distribution<double> dist_x(x, std_x);
  normal_distribution<double> dist_y(y, std_y);
  normal_distribution<double> dist_theta(theta, std_theta);
  cout << "in Init line 41" ;
  // Again below is based on Lesson 14 Video 6 Gausian Sampling Quiz solution
  for (int i = 0; i < num_particles; i++) { // I note Tifany used ++i here ti count to 3 from 0 for 3 counts I will use i++
		double sample_x, sample_y, sample_theta;
		cout << "in Init line 43 in for loop" ;
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
  cout << "in Init line 64" ;
  is_initialized = true ;//set boolean 
  cout << "in Init line 65" ;
  
  
  

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
  cout << "in Prediction line 77" << endl;
  double std_x = std_pos[0] ;
  double std_y = std_pos[1] ;
  double std_theta = std_pos[2] ;
  
  default_random_engine gen ; //same as init
  
  for (int i = 0; i < num_particles; i++) {
  	cout << "in Prediction main loop line 85" << endl;
    double predicted_x, predicted_y,predicted_theta ;
    if( yaw_rate == 0){
    	yaw_rate = 0.0001;  //this is to avoid a divide by zero and get the below equations to work
    }
    //equations from LEsson 14 section 9 : "Prediction step quiz explanation"
    double theta = particles[i].theta ;
    cout << "in Prediction main loop line 92" << endl;
    predicted_x = (velocity/yaw_rate ) * ( sin(theta + ( delta_t * yaw_rate))- sin(theta) );
    predicted_y = (velocity/yaw_rate ) * (cos(theta) - cos(theta + (delta_t * yaw_rate))  ) ;
    predicted_theta = yaw_rate * delta_t ;
    cout << "in Prediction main loop line 96" << endl;
    //create Gausians around predictions
    normal_distribution<double> dist_x(predicted_x, std_x);
    normal_distribution<double> dist_y(predicted_y, std_y);
    normal_distribution<double> dist_theta(predicted_theta, std_theta);
    cout << "in Prediction main loop line 101" << endl;
    // select and add a random gausian of the prediction and assign to particle in vector
    particles[i].x = dist_x(gen) ;
    particles[i].y =  dist_y(gen) ;
    particles[i].theta = dist_theta(gen) ;  
    cout << "in Prediction main loop line 106" << endl;
    
  }
  

}

//void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> landmarks_in_range, std::vector<LandmarkObs>& transformed_observations) {
  //landmarks_in_range , transformed_observations)
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
  
  for(int i =0; i < transformed_observations.size() ; i++)
  {
    double nearest_neighbour_distance = 999999 ; //deliberatley initalising with very big distance that will be overwritten
    int nearest_neighbour_id ;//do I need to intialise ?? don't think so
      
  	for(int j =0; j < landmarks_in_range.size() ; j++)
    {
      double test_distance = dist(transformed_observations[i].x, transformed_observations[i].y , landmarks_in_range[j].x, landmarks_in_range[j].y);
      
      if(test_distance < nearest_neighbour_distance)
      {
        nearest_neighbour_distance = test_distance ;
        nearest_neighbour_id = landmarks_in_range[j].id;
        
      }
      
    }
    
    transformed_observations[i].id = nearest_neighbour_id;
    
    
  }

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
  //make a vector to store in range landmarks.....done
  //loop through landmarks and extract those within sensor range.....done
  // translate observations to particle point of view...done
  //associate onservations with landmarks....done
  //multivariate gausian part..done
  //set assocaitions...done
  
  
  double std_landmark_x = std_landmark[0] ;
  double std_landmark_y = std_landmark[1] ;
  // this loop goes through each particle one at a time
  for(int i = 0 ; i < num_particles ; i++){ 
    //this vector will store the subset of landmarks within range of the particle
    vector<LandmarkObs> landmarks_in_range ; 
    vector<int> associations;
    vector<double> sense_x ;
    vector<double> sense_y ;
    
    //find landmarks within senor range and add to vector landmarks_in_range
    for(int j = 0 ; j < map_landmarks.landmark_list.size() ; j++){
          //extracting particle x and y and landmark x and y for distance evaluation
          float x_f = map_landmarks.landmark_list[j].x_f ;
          float y_f = map_landmarks.landmark_list[j].y_f ;
          float id_i = map_landmarks.landmark_list[j].id_i ;
          float px = particles[i].x ;
          float py = particles[i].y ;

          // if landmark within sensor reange add to vector
          if(sensor_range >= dist(px,py,x_f,y_f)){
            LandmarkObs new_in_range_landmark ;
            new_in_range_landmark.x = x_f ;
            new_in_range_landmark.y = y_f ;
            new_in_range_landmark.id = id_i ;//need to set this?
            landmarks_in_range.push_back(new_in_range_landmark);
          }
        
      }// loop should have produced a vector of landmarks within range of particle
      
      //loop tp transform observations to particle point of view
      //transform equation from Lesson :14 Video : 16
      vector<LandmarkObs> transformed_observations ;
      for(int j = 0 ; j < observations.size() ; j++){
      	
          //using same terms as LEsson 14 Video 16 to make it easier to read
          double x_p = particles[i].x ;
          double y_p = particles[i].y ;
          double theta = particles[i].theta ;
          double x_c = observations[j].x ;
          double y_c = observations[j].y ;

          LandmarkObs  transformed_observation ;
          transformed_observation.id = observations[j].id ;
          transformed_observation.x = x_p + (cos(theta) * x_c) - ( sin(theta) * y_c) ; //same variable names as in lesson for clarity
          transformed_observation.y = y_p + (sin(theta) * x_c) + ( cos(theta) * y_c) ;
          //below OK just shortened to make more readable
          //transformed_obsesrvation.x = particles[i].x + (cos(particles[i].theta) * observations[j].x)- (sin(particles[i].theta) * observations[j].y);//is i here correct index??...think so
          //transformed_obsesrvation.y = particles[i].y + (sin(particles[i].theta) * observations[j].x)+ (cos(particles[i].theta) * observations[j].y);

          transformed_observations.push_back(transformed_observation);        
        
     	}//end of transformation loop new vector filled
    
   //dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations)
    	dataAssociation( landmarks_in_range , transformed_observations); // send to above method
    	
    	//mulitvariate gausian from Lesson 14 Video :19 and 20 converted from Python to C++
        //  # calculate normalization term
        //gauss_norm= (1/(2 * np.pi * sig_x * sig_y))
        //# calculate exponent
        //exponent= ((x_obs - mu_x)**2)/(2 * sig_x**2) + ((y_obs - mu_y)**2)/(2 * sig_y**2)
        //# calculate weight using normalization terms and exponent
        //weight= gauss_norm * math.exp(-exponent)
   		double gauss_norm = (1/(2 * M_PI * std_landmark_x * std_landmark_y )) ; //std_landmark_x from start of method
    	double exponent = 0;
    	double x_obs = 0 ;
        double mu_x = 0 ;
    	double y_obs = 0 ;
    	double mu_y = 0 ;
    	double weight = 1;
    
    	for(int k = 0 ;k < transformed_observations.size();k++)
        {
          	for(int l = 0; l < landmarks_in_range.size() ; l++)
            {
              	if( landmarks_in_range[l].id == transformed_observations[k].id )
                {
                  //variable names from Lesson 14 Video :19 and 20
                  x_obs = landmarks_in_range[l].x ;
                  mu_x  = transformed_observations[k].x ;
                  y_obs = landmarks_in_range[l].y ;
                  mu_y  = transformed_observations[k].y ;
                  
                  break;//got id match break to save time
                  
                }
            }
            //variable names from Lesson 14 Video :19 and 20
          	//exponent = ((x_obs - mu_x)**2)/(2 * sig_x**2) + ((y_obs - mu_y)**2)/(2 * sig_y**2)
          	exponent = (pow((x_obs - mu_x),2))/(2 * std_landmark_x * std_landmark_x) + (pow((y_obs - mu_y),2))/(2 * std_landmark_y * std_landmark_y) ;
          	//weight= gauss_norm * math.exp(-exponent)
          	double temp_weight;
          	temp_weight*= gauss_norm * exp(-exponent) ;
          	if(temp_weight > 0 )
            {
              weight *= temp_weight ; // to avoid a mulitply by zero
            }
          	sense_x.push_back(transformed_observations[k].x );
            sense_y.push_back(transformed_observations[k].y );
            associations.push_back(transformed_observations[k].id );
        }
    	particles[i].weight = weight ;
    	weights.push_back(weight); 
    	particles[i] = SetAssociations(particles[i], associations, sense_x, sense_y) ; 
    
      
    }//end of particle loop
    
  
  
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  
  //this is based on lesson 13 video 20 and suggested C++ link above
  
  
  //discrete_distribution(weights.begin(), weights.end()) //from https://en.cppreference.com/w/cpp/numeric/random/discrete_distribution/discrete_distribution
  //below is straight from the Q+A video using above C++ library similar to above library code example
  cout << "in resample line 294" << endl;
  default_random_engine gen;
  discrete_distribution<int> distribution(weights.begin(), weights.end() );
  cout << "in resample line 297" << endl;
  vector<Particle> resample_particles ;
  cout << "in resample line 299 num_partciles:" << num_particles << endl;
  cout << "in resample line 300 particles.size:" << particles.size() << endl;
  cout << "in resample line 301 weights.size:" << weights.size() << endl;
    
  for(int i = 0 ; i < num_particles ; i++)
  {
    cout << "in resample line 302" << endl;
    double test_gen = distribution(gen);
    cout << "test_gen" << test_gen << endl;// I am wondering is distribution(gen) producing an invalid index?? or maybe not enough partciles to choose from??
    //resample_particles.push_back(particles[distribution(gen)]);
    resample_particles.push_back(particles[test_gen]);
  }
  cout << "in resample line 305" << endl;
  particles = resample_particles ;
  
cout << "in resample line 308" << endl;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates
  
    particle.associations.clear();// I saw this clearing section in the Q+A video
    particle.sense_x.clear();
    particle.sense_y.clear();

    particle.associations = associations;
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
