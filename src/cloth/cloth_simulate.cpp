#include "cloth.h"
#include <cmath>

void Cloth::Animate() {
  //TODO: Update for new data structure

  // Update Forces
  std::vector<std::vector<bool>> pointsToSubdivide = getShouldSubdivide(0.5);
  for(int i = 0; i < (int)particles.size(); i ++){
    for(int j = 0; j < (int)particles[i].size(); j ++){
      std::cout << pointsToSubdivide[i][j] << " ";
    }
    std::cout << std::endl;
  }

}

float getAngle(Vec3f p1, Vec3f p2, Vec3f p3){
  Vec3f p12 = p1 - p2;
  Vec3f p23 = p2 - p3;
  p12.Normalize();
  p23.Normalize();
  return acos(p12.Dot3(p23));
}


std::vector<std::vector<bool>> Cloth::getShouldSubdivide(float threshold){
  std::vector<std::vector<bool>> result;
  for(int i = 0; i < (int)particles.size(); i++){
    std::vector<bool> rowResults;
    for(int j = 0; j < (int)particles[i].size(); j ++){
      ClothParticle p = getParticle(i, j);
      Vec3f pos = getParticle(i, j).position;
      Vec3f north = pos;
      Vec3f south = pos;
      Vec3f east = pos;
      Vec3f west = pos;
      if (i - 1 >= 0)
        north = getParticle(i - 1, j).position;
      if (i + 1 < nx)
        south = getParticle(i + 1, j).position;
      if (j - 1 >= 0)
        east = getParticle(i, j - 1).position;
      if (j + 1 < ny)
        west = getParticle(i, j + 1).position;
      if(i - 1 >= 0 && i + 1 < nx){
        if(j - 1 >= 0 && j + 1 < ny){
          float t1 = getAngle(south, pos, north);
          float t2 = getAngle(west, pos, east);
          std::cout << t1 << " " << t2 << std::endl;
          if(t1 > threshold || t2 > threshold){
            rowResults.push_back(true);
            continue;
          }
        }
        else{
          float t1 = getAngle(south, pos, north);
          if(t1 > threshold){
            rowResults.push_back(true);
            continue;
        }   
      }
    }
    else if(j - 1 >= 0 && j + 1 < ny){
      float t1 = getAngle(east, pos, west);
          if(t1 > threshold){
            rowResults.push_back(true);
            continue;
    }   
    }
    rowResults.push_back(false);
  }
    result.push_back(rowResults);
  }
  return result;
}


void Cloth::subdivide() {
  // Todo: figure out threshold
 

}
