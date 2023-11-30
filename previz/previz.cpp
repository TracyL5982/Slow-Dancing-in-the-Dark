//////////////////////////////////////////////////////////////////////////////////
// This is a front end for a set of viewer clases for the Carnegie Mellon
// Motion Capture Database:
//
//    http://mocap.cs.cmu.edu/
//
// The original viewer code was downloaded from:
//
//   http://graphics.cs.cmu.edu/software/mocapPlayer.zip
//
// where it is credited to James McCann (Adobe), Jernej Barbic (USC),
// and Yili Zhao (USC). There are also comments in it that suggest
// and Alla Safonova (UPenn) and Kiran Bhat (ILM) also had a hand in writing it.
//
//////////////////////////////////////////////////////////////////////////////////
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <float.h>
#include "SETTINGS.h"
#include "skeleton.h"
#include "displaySkeleton.h"
#include "motion.h"
#include "parse_stl.h"
#include "utils.cpp"
#include "primitives.cpp"
using namespace std;
using namespace stl;

float PI = 3.1415926535;

// Stick-man classes
DisplaySkeleton displayer;
Skeleton* skeleton;
Motion* motion;

int windowWidth = 640;
int windowHeight = 480;

VEC3 eye(-6, 0.5, 1);
VEC3 lookingAt(5, 0.5, 1);
VEC3 up(0,1,0);

float x_min;
float x_max;
float y_min;
float y_max;
float z_min;
float z_max;
double cameraOffset;
int camera_y_count;

// scene geometry
vector<Primitive *> primitives;

#include "model.cpp"
vector<Model> models;

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void writePPM(const string& filename, int& xRes, int& yRes, const float* values)
{
  int totalCells = xRes * yRes;
  unsigned char* pixels = new unsigned char[3 * totalCells];
  for (int i = 0; i < 3 * totalCells; i++)
    pixels[i] = values[i];

  FILE *fp;
  fp = fopen(filename.c_str(), "wb");
  if (fp == NULL)
  {
    cout << " Could not open file \"" << filename.c_str() << "\" for writing." << endl;
    cout << " Make sure you're not trying to write from a weird location or with a " << endl;
    cout << " strange filename. Bailing ... " << endl;
    exit(0);
  }

  fprintf(fp, "P6\n%d %d\n255\n", xRes, yRes);
  fwrite(pixels, 1, totalCells * 3, fp);
  fclose(fp);
  delete[] pixels;
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void rayColor(const VEC3& rayPos, const VEC3& rayDir, VEC3& pixelColor)
{
  pixelColor = VEC3(1,1,1);

  Ray r;
  r.d = rayDir;
  r.o = rayPos;

  // look for intersections
  int hitID = -1;
  float tMinFound = FLT_MAX;
  Primitive *hit = NULL;
  VEC3 hitNormal;

  for(int i = 0; i < primitives.size(); i++){
    Primitive * prim = primitives[i];
    float tMin;
    VEC3 thisNorm;
    if(prim->intersectRay(r, tMin, thisNorm)){
      if(tMin < tMinFound){
        hitID = i;
        hit = primitives[i];
        tMinFound = tMin;
        hitNormal = thisNorm;
      }
    }
  }

  if(hit){
    pixelColor = hit->getColor(r, tMinFound);
  }
  return;
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void renderImage(int& xRes, int& yRes, const string& filename)
{
  // allocate the final image
  const int totalCells = xRes * yRes;
  float* ppmOut = new float[3 * totalCells];

  // compute image plane
  const float halfY = (lookingAt - eye).norm() * tan(45.0f / 360.0f * M_PI);
  const float halfX = halfY * 4.0f / 3.0f;

  const VEC3 cameraZ = (lookingAt - eye).normalized();
  const VEC3 cameraX = up.cross(cameraZ).normalized();
  const VEC3 cameraY = cameraZ.cross(cameraX).normalized();

  for (int y = 0; y < yRes; y++)
    for (int x = 0; x < xRes; x++)
    {
      const float ratioX = 1.0f - x / float(xRes) * 2.0f;
      const float ratioY = 1.0f - y / float(yRes) * 2.0f;
      const VEC3 rayHitImage = lookingAt +
                               ratioX * halfX * cameraX +
                               ratioY * halfY * cameraY;
      const VEC3 rayDir = (rayHitImage - eye).normalized();

      // get the color
      VEC3 color;
      rayColor(eye, rayDir, color);

      // set, in final image
      ppmOut[3 * (y * xRes + x)] = clamp(color[0]) * 255.0f;
      ppmOut[3 * (y * xRes + x) + 1] = clamp(color[1]) * 255.0f;
      ppmOut[3 * (y * xRes + x) + 2] = clamp(color[2]) * 255.0f;
    }
  writePPM(filename, xRes, yRes, ppmOut);

  delete[] ppmOut;
}

//////////////////////////////////////////////////////////////////////////////////
// Load up a new motion captured frame
//////////////////////////////////////////////////////////////////////////////////
void setSkeletonsToSpecifiedFrame(int frameIndex)
{
  if (frameIndex < 0)
  {
    printf("Error in SetSkeletonsToSpecifiedFrame: frameIndex %d is illegal.\n", frameIndex);
    exit(0);
  }
  if (displayer.GetSkeletonMotion(0) != NULL)
  {
    int postureID;
    if (frameIndex >= displayer.GetSkeletonMotion(0)->GetNumFrames())
    {
      cout << " We hit the last frame! You might want to pick a different sequence. " << endl;
      postureID = displayer.GetSkeletonMotion(0)->GetNumFrames() - 1;
    }
    else
      postureID = frameIndex;
    displayer.GetSkeleton(0)->setPosture(* (displayer.GetSkeletonMotion(0)->GetPosture(postureID)));
  }
}

VEC3 color_255to1 (VEC3 color){
  VEC3 newColor (double (color[0]/255.0), double (color[1]/255.0), double (color[2]/255.0));
  return newColor;
}

//////////////////////////////////////////////////////////////////////////////////
// Build a list of spheres in the scene
//////////////////////////////////////////////////////////////////////////////////

void buildGround(){
  vector<VEC3> triangle1;
  triangle1.push_back(VEC3(5, -0.0586798, -5));
  triangle1.push_back(VEC3(5, -0.0586798, 7));
  triangle1.push_back(VEC3(-6, -0.0586798, -5));

  vector<VEC3> triangle2;
  triangle2.push_back(VEC3(-6,-0.0586798, -5));
  triangle2.push_back(VEC3(5, -0.0586798, 7));
  triangle2.push_back(VEC3(-6,-0.0586798, 7));

  VEC3 color (76, 159, 209);
  VEC3 thisColor = color_255to1(color);

  Tri * tri_1 = new Tri(triangle1, thisColor);
  Tri * tri_2 = new Tri(triangle2, thisColor);
  primitives.push_back(tri_1);
  primitives.push_back(tri_2);
  return;
}

void buildClouds(){
  VEC3 s1_center (3.5, 2.25, -0.4);
  VEC3 s2_center (3.5, 2, 0);
  VEC3 s3_center (3.5, 2.5, 0.3);
  VEC3 s4_center (3.25, 2, -0.2);
  float r1 = 0.5;
  float r2 = 0.5;
  float r3 = 0.5;
  float r4 = 0.3;
  VEC3 color (162, 220, 202);
  VEC3 thisColor = color_255to1(color);
  Sphere * s1 = new Sphere(s1_center, r1, thisColor);
  Sphere * s2 = new Sphere(s2_center, r2, thisColor);
  Sphere * s3 = new Sphere(s3_center, r3, thisColor);
  Sphere * s4 = new Sphere(s4_center, r4, thisColor);
  primitives.push_back(s1);
  primitives.push_back(s2);
  primitives.push_back(s3);
  primitives.push_back(s4);
}

void buildFigure(){
  displayer.ComputeBonePositions(DisplaySkeleton::BONES_AND_LOCAL_FRAMES);
  // retrieve all the bones of the skeleton
  vector<MATRIX4>& rotations = displayer.rotations();
  vector<MATRIX4>& scalings  = displayer.scalings();
  vector<VEC4>& translations = displayer.translations();
  vector<float>& lengths     = displayer.lengths();

  vector<Primitive *> sticks;
  int totalBones = rotations.size();

  for (int x = 1; x < totalBones; x++)
  {
    MATRIX4& rotation = rotations[x];
    MATRIX4& scaling = scalings[x];
    VEC4& translation = translations[x];

    //camera movements
    VEC4 pelvisTranslation = translations[1];
    VEC3 pelvisPosition = pelvisTranslation.head<3>();
    double camera_y = camera_y_count * 0.0005 + 1;
    eye = pelvisPosition + VEC3(cameraOffset, camera_y, 0);
    lookingAt = pelvisPosition;
    cameraOffset -= 0.001;
    camera_y_count ++;

    // get the endpoints of the cylinder
    VEC4 leftVertex(0,0,0,1);
    VEC4 rightVertex(0,0,lengths[x],1);

    leftVertex = rotation * scaling * leftVertex + translation;
    rightVertex = rotation * scaling * rightVertex + translation;
    vector<VEC3> c_ends;
    c_ends.push_back(leftVertex.head<3>());
    c_ends.push_back(rightVertex.head<3>());
    VEC3 color (239, 202, 205);
    VEC3 thisColor = color_255to1(color);
    Cylinder * cylin = new Cylinder(c_ends, 0.05, thisColor);
    primitives.push_back(cylin);
    sticks.push_back(cylin);
  }

}
void buildScene(float frame_num)
{
  primitives.clear();
  buildClouds();
  buildFigure();
  buildGround();
  for(Model m : models){
    m.addToScene (frame_num);
  }
//   cout << "y_min: "<< y_min << endl;
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  cameraOffset = -2;
  camera_y_count = 0;
  string skeletonFilename("02.asf");
  string motionFilename("55_01.amc");
  y_min = FLT_MAX;

  // load up skeleton stuff
  skeleton = new Skeleton(skeletonFilename.c_str(), MOCAP_SCALE);
  skeleton->setBasePosture();
  displayer.LoadSkeleton(skeleton);

  // load up the motion
  motion = new Motion(motionFilename.c_str(), MOCAP_SCALE, skeleton);
  displayer.LoadMotion(motion);
  skeleton->setPosture(*(displayer.GetSkeletonMotion(0)->GetPosture(0)));

  // load model
  Model test;
  test.Load("resources/test.stl");
  test.setColor(VEC3(1,1,0));
  test.setPosKeys({0, 450, 900, 1350, 1800},
                  {VEC3(0,0,-2), VEC3(2,1,0), VEC3(0,2,2), VEC3(-2,1,0), VEC3(0,0,-2)},
                  {0,0,0,0,0});
  test.setScaleKeys({0},{VEC3(.01, .01, .01)}, {0});
  test.setRotKeys({0, 450, 900, 1350, 1800},
                  {VEC3(-PI/2.0,0,0), VEC3(-PI/2.0,0,0), VEC3(-PI/2.0,0,0), VEC3(-PI/2.0,0,0), VEC3(-PI/2.0,0,0)},
                  {0, 0, 0, 0, 0});
  test.setActiveKeys({0,1800}, {1, 1});
  models.push_back(test);

  // Note we're going 6 frames at a time, otherwise the animation
  // is really slow.
  for (int x = 0; x < 1800; x += 6)
  {
    setSkeletonsToSpecifiedFrame(x);
    buildScene(x);

    char buffer[256];
    snprintf(buffer, 256, "./frames/frame.%04i.ppm", x / 6);
    renderImage(windowWidth, windowHeight, buffer);
    cout << "Rendered " + to_string(x / 6) + " frames" << endl;
  }

  return 0;
}
