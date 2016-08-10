#include <Pacer/controller.h>
#include <Pacer/utilities.h>
#include "plugin.h"
#include <iostream>
void loop(){
boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  
   std::vector<int> feet_vector;
   ctrl->get_data<std::vector<int> >("active-feet",feet_vector);
   int counter=0;

   for(int i=0; i<feet_vector.size();i++)
   {
    counter+=feet_vector[i];
   }

std::ofstream myFile;
myFile.open("feet.txt");
myFile << counter;
std::cout << counter << "\n" << "\n";
myFile.close();
}

void setup(){
}
