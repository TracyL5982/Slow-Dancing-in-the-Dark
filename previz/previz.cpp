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
#include <thread>
#include <vector>

using namespace std;
using namespace stl;

float PI = 3.1415926535;
int PHONG = 10;

// Stick-man classes
DisplaySkeleton displayer;
Skeleton* skeleton;
Motion* motion;

// int windowWidth = 160;
// int windowHeight = 120;

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

int hello;

VEC3 figureLocation;
VEC3 headLocation;
float cReflection = 0.5;
float cGaussian = 0.2;

// scene geometry
vector<Primitive *> primitives;
vector<Light *> lights;

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

float * addPPM (float* a, float* b, int length){
  float* ppmOut = new float[3*length];
  for(int i = 0; i < length; i++){
    ppmOut[i] = a[i] + b[i];
  }
  return ppmOut;
}

float* dividePPM (float* a, float div, int length){
  float* ppmOut = new float[3*length];
  for(int i = 0; i < length; i++){
    ppmOut[i] = a[i]/div;
  }
  return ppmOut;
}
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

VEC3 calculateShading(Primitive *primitive, VEC3 point, VEC3 normal, Light &light, VEC3 eyeDir, VEC3 surfaceColor) {

	VEC3 mat_specular = VEC3(1.0, 1.0, 1.0);	

	// normalize interpolated normal
	VEC3 N = (normal).normalized();

	// light vector 
  VEC3 lightPos = light.getPos() + 0.001*normal;
	VEC3 L = (light.getPos() - point).normalized();

	// view vector
	VEC3 V = (eyeDir -point).normalized();

	// half-vector
	VEC3 H = (L + V).normalized();
	
	// scalar products
	float NdotH = max(0.0, N.dot(H));
	float VdotH = V.dot(H);
	float NdotV = N.dot(V);
	float NdotL = N.dot(L);

	// diffuse
	VEC3 diffuse = surfaceColor * NdotL;

	// D 
	float alpha = acos(NdotH);
	float D = cGaussian * exp(- alpha * alpha * cGaussian); 

	// G
	float G1 = 2.0 * NdotH * NdotV / VdotH;
	float G2 = 2.0 * NdotH * NdotL / VdotH;
	float smaller = min(G1, G2);
	float G = min((float) 1.0, smaller);

	// F
	float k = pow(1.0 - NdotV, 5.0);
	float F = cReflection + (1.0 - cReflection) * k;

	VEC3 specular = mat_specular * (F * D * G) / NdotV;

	VEC4 finalColor = 0.4*VEC4(diffuse[0], diffuse[1], diffuse[2], 1.0) + 0.8*VEC4(specular[0], specular[1], specular[2], 1.0);
  VEC3 color = finalColor.head<3>();
	return color;
}

void rayColor(const VEC3& rayPos, const VEC3& rayDir, VEC3& pixelColor, int frame_num)
{
  pixelColor = VEC3(0,0,0);

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

  if(!hit){
    return;
  }
 
  VEC3 surfaceColor = hit->getColor(r, tMinFound);
  VEC3 hitPoint = r.o + (tMinFound - 0.0001) * r.d;
  VEC3 e = eye - hitPoint;
  e.normalize();

  VEC3 lightSum (0,0,0);
  for(Light* l : lights){
    lightSum += calculateShading(hit, hitPoint, hitNormal, *l, e, surfaceColor);
    // Ray shadow;
    // shadow.o = hitPoint;
    // shadow.d = (l->getPos() - hitPoint).normalized();
    // VEC3 r = -shadow.d + 2*(shadow.d.dot(hitNormal))*hitNormal;
    // r.normalize();
    // VEC3 e = eye - hitPoint;
    // e.normalize();
    // VEC3 lightColor = l->getColor();
    // lightSum += lightColor * max(0, hitNormal.dot(shadow.d));
    // lightSum += lightColor * pow(max (0, e.dot(r)), PHONG);
  }

  pixelColor = lightSum;

  // pixelColor[0] = surfaceColor[0] * lightSum[0];
  // pixelColor[1] = surfaceColor[1] * lightSum[1];
  // pixelColor[2] = surfaceColor[2] * lightSum[2];
  return;
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
  triangle1.push_back(VEC3(10, -0.0586798, -10));
  triangle1.push_back(VEC3(10, -0.0586798, 10));
  triangle1.push_back(VEC3(-10, -0.0586798, -10));

  vector<VEC3> triangle2;
  triangle2.push_back(VEC3(-10,-0.0586798, -10));
  triangle2.push_back(VEC3(10, -0.0586798, 10));
  triangle2.push_back(VEC3(-10,-0.0586798, 10));

  VEC3 color (0, 0, 0);
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

void buildFigure(float frame_num){
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

      if(frame_num <= 180){
        double camera_x = 6.68;
        double camera_y = 4.672;
        eye = VEC3 (-0.0492647,1.02053,-0.0316883) + VEC3 (camera_x, camera_y, 0);
        lookingAt = pelvisPosition;
        if (frame_num == 180){
          cout << "**********FRAME 180**********"<< endl;
          cout << "eye: "<< endl;
          cout << eye << endl;
          cout << "lookingAt: "<< endl;
          cout << lookingAt << endl;
        }
        
      }

      else if (frame_num > 180 && frame_num <= 1080){
        double camera_x = 6.68- 0.0036*(frame_num-180);
        double camera_y = 4.672- 0.00353875*(frame_num-180);
        eye = VEC3 (-0.0492647,1.02053,-0.0316883) + VEC3 (camera_x, camera_y, 0);
        lookingAt = pelvisPosition;
        if (frame_num == 1080){
          cout << "**********FRAME 1080**********"<< endl;
          cout << "eye: "<< endl;
          cout << eye << endl;
          cout << "lookingAt: "<< endl;
          cout << lookingAt << endl;
        }
      }

      else if (frame_num > 1080 && frame_num <= 1440){
        float new_frame_num = frame_num - 1080;
        double angleOffset = PI / 2.0 + (1 / 360.0 * new_frame_num * PI / 2.0);
        double camera_x = sin(angleOffset) * 2.19435;
        double camera_z = cos(angleOffset) * 2.19435;
        double camera_y = 0.022 * new_frame_num + 1.42587;
        eye = pelvisPosition + VEC3(camera_x, camera_y, camera_z);
        lookingAt = pelvisPosition;
         if (frame_num == 1440){
          cout << "**********FRAME 1440**********"<< endl;
          cout << "eye: "<< endl;
          cout << eye << endl;
          cout << "lookingAt: "<< endl;
          cout << lookingAt << endl;
        }
      }else if ( frame_num > 1440 && frame_num <= 1560){
        eye =  VEC3(0.530943,  10.4037, -2.83704);
        lookingAt = pelvisPosition;
      }else{
        float new_frame_num = frame_num - 1560;
        double camera_y = -0.0222 * new_frame_num +  10.4037;
        eye = VEC3(0.530943, 0, -2.83704) + VEC3 (0, camera_y, 0);
        lookingAt = pelvisPosition;
        if(frame_num == 1774){
          cout << "**********FRAME 1774**********"<< endl;
          cout << "eye: "<< endl;
          cout << eye << endl;
          cout << "lookingAt: "<< endl;
          cout << lookingAt << endl;
        }
      }    

    // get the endpoints of the cylinder
    VEC4 leftVertex(0,0,0,1);
    VEC4 rightVertex(0,0,lengths[x],1);

    leftVertex = rotation * scaling * leftVertex + translation;
    rightVertex = rotation * scaling * rightVertex + translation;

    if(x == 5){
      figureLocation = rightVertex.head<3>();
      if(frame_num == 0 || frame_num == 300 ||frame_num == 600 ||frame_num == 900 ||frame_num == 1200 ||frame_num == 1500 ||frame_num == 1794) {
        cout << frame_num << ": " << figureLocation << endl;
      }
      
    }

    if(x == 15){
      headLocation = rightVertex.head<3>();
    }

    vector<VEC3> c_ends;
    c_ends.push_back(leftVertex.head<3>());
    c_ends.push_back(rightVertex.head<3>());
    VEC3 color (1, 1, 1);
    VEC3 testColor (1, 0, 0);
    VEC3 thisColor;
    Cylinder * cylin = new Cylinder(c_ends, 0.05, color);
    primitives.push_back(cylin);
    sticks.push_back(cylin);
  }

}

void buildFibCubes(float frame_num) {
    // Constants for the spiral and animation
    float radius = 1.0;
    double a = 0;
    float scaleFactorBase = 0.1;
    float cubeSizeBase = 0.0015;  // Base size of the cube

    vector<VEC3> blueToGreenGradient = {
      color_255to1(VEC3(180, 220, 255)), // Bright light blue
      color_255to1(VEC3(150, 215, 255)), // Light blue
      color_255to1(VEC3(120, 210, 255)), // Transitioning blue
      color_255to1(VEC3(90, 205, 230)),  // Blue with a hint of green
      color_255to1(VEC3(60, 200, 205)),   // Light blue-green
      color_255to1(VEC3(30, 195, 180)),   // Mid blue-green
      color_255to1(VEC3(10, 190, 155)),   // Blue-green
      color_255to1(VEC3(10, 185, 130)),   // Transitioning to green
      color_255to1(VEC3(10, 180, 105)),   // Green with a hint of blue
      color_255to1(VEC3(10, 175, 80)),    // Dark green
};


    vector<VEC3> greens = {
        color_255to1(VEC3(0, 129, 167)), 
        color_255to1(VEC3(0, 175, 137)),
        color_255to1(VEC3(151, 192, 177)),
        color_255to1(VEC3(254, 217, 183)),
        color_255to1(VEC3(240, 113, 103)) 
    };

    int totalCubes = 512;
    int colorChangeInterval = 720;
    int layers =10;

    // Position and scale the cube model
    float maxScaleFactor = 0.5f; // Maximum scale factor for the largest cube
    float minScaleFactor = 0.1f; // Minimum scale factor for the smallest cube
    int totalRippleFrames = 450; // 450 frames for a ripple
    int totalLoops = 2; //2 loops of size change ripples
    
    for(int i = 0; i < totalCubes; i++) {
        float scaleFactor = (i / totalCubes) * 0.1 + scaleFactorBase;
        Model cubeModel;
        cubeModel.Load("resources/test1.stl");
        int colorIndex = i % blueToGreenGradient.size();
        VEC3 color = blueToGreenGradient[colorIndex];
        cubeModel.setColor(color);

        vector<float> scaleFrames;
        vector<VEC3> scaleValues;
        vector<int> scaleInterps;

        int layer = i / (totalCubes / layers); 

        float initialScaleFactor = minScaleFactor + (maxScaleFactor - minScaleFactor) * layer / (layers - 1);
        scaleFrames.push_back(0);
        scaleValues.push_back(VEC3(initialScaleFactor * cubeSizeBase, initialScaleFactor * cubeSizeBase, initialScaleFactor * cubeSizeBase));
        scaleInterps.push_back(0);

        int initialFrameOffset = layer * totalRippleFrames / layers; 

        for (int loop = 0; loop < totalLoops; loop++) {
            for (int frame = 180 + loop * totalRippleFrames; frame <= 180 + (loop + 1) * totalRippleFrames; frame += 90) {
                int adjustedFrame = frame - initialFrameOffset;
                while (adjustedFrame < 180) adjustedFrame += totalRippleFrames * totalLoops; // Ensure frame is within range

                float scaleFactor;
                int frameInLoop = adjustedFrame % totalRippleFrames; 
                if (frameInLoop <= totalRippleFrames / 2) {
                    scaleFactor = minScaleFactor + (maxScaleFactor - minScaleFactor) * frameInLoop / (totalRippleFrames / 2.0f);
                } else {
                    scaleFactor = maxScaleFactor - (maxScaleFactor - minScaleFactor) * (frameInLoop - totalRippleFrames / 2) / (totalRippleFrames / 2.0f);
                }

                scaleFrames.push_back(frame);
                scaleValues.push_back(VEC3(scaleFactor * cubeSizeBase, scaleFactor * cubeSizeBase, scaleFactor * cubeSizeBase));
                scaleInterps.push_back(0); 
            }
        }

        scaleFrames.push_back(1794);
        scaleValues.push_back(VEC3(initialScaleFactor * cubeSizeBase, initialScaleFactor * cubeSizeBase, initialScaleFactor * cubeSizeBase));
        scaleInterps.push_back(0);

        // Set scale keys for the cube model
        cubeModel.setScaleKeys(scaleFrames, scaleValues, scaleInterps);

        // Calculate positions for different frames to create a ripple effect
        std::vector<VEC3> positions;
        vector<float> frames;
        vector<int> interps;

        VEC3 cubeCenter;

        for (int frame = 0; frame <= 1800; frame++) {

            int layer = i / (totalCubes / layers); // Determine the layer of the cube
            float rotationSpeed = 0.0005;
            float dynamicRadius = radius;
            float groundLevel = -0.0586798;
            float wavePhase = (radius - 1.0) * 4.0; // Adjust wave phase based on radius
            float waveAmplitude = 0.5; 
            float waveFrequency = 1.0;
            float waveHeight = 1.0;

            if (frame <= 360) {
                cubeCenter = figureLocation;
              // waveHeight = sin((frame - wavePhase) * waveFrequency * (PI / 180)) * waveAmplitude;
              // waveHeight += groundLevel + waveAmplitude; 
              // float expansionFactor = 1.0 + (frame - 360) / 360.0; // expand up to twice the radius
              // float convergenceFactor = 2.0 - (frame - 360) / 360.0; // then converge back to original radius
              // dynamicRadius *= (frame <= 540) ? expansionFactor : convergenceFactor;
            }

            if (frame > 360 && frame <= 720) {
              cubeCenter = figureLocation;
              if (frame <= 540) {
                  // Converge from 1.0 to 0.5
                  float convergeFactor = 1 - ((frame - 360) / 180.0) * 0.5; 
                  dynamicRadius *= convergeFactor; 
              } else {
                  // Expand from 0.5 to 1.0
                  float expandFactor = 0.5 + ((frame - 540) / 180.0) * 0.5; 
                  dynamicRadius *= expandFactor; 
              }
          }

             if (frame > 720 && frame <= 1080) {
              cubeCenter = figureLocation;
                rotationSpeed = 0.0005 + (frame - 720) * 0.00000138;
                 if(frame <= 900){
                  // Expand from 1.0 to 1.5
                  float expandFactor = (frame-720)/360.0;
                  dynamicRadius *= (1+expandFactor);
                }else{
                  // Converge from 1.5 to 1.0
                  float convergeFactor = (frame-900)/360.0;
                  dynamicRadius *= (1.5-convergeFactor);
                }
             }

            if (frame > 1080 && frame <= 1260) {
              cubeCenter = figureLocation;
                  rotationSpeed = 0.001;
                  float expandFactor = (frame-1080)/360.0;
                  dynamicRadius *= (1+expandFactor);
            }

            if(frame > 1260 && frame <= 1440){
              cubeCenter = figureLocation;
                rotationSpeed = 0.001 - (frame - 1260) * 0.00000555;
                float convergeFactor = (frame - 1260)/360.0;
                dynamicRadius *= (1.5 - convergeFactor);
            }

            if(frame > 1440 && frame <= 1560){
              cubeCenter = figureLocation;
              rotationSpeed = 0;
            }

            if(frame > 1560 && frame <= 1800){
              cubeCenter = figureLocation;
              rotationSpeed = 0.00000138 * (frame - 1560);
              float convergeFactor = (frame - 1560)/180.0;
              dynamicRadius *= (1.0-convergeFactor);

            }

            double x = dynamicRadius * cos(a);
            double y = dynamicRadius * sin(a);
            float rotationAngle = frame * rotationSpeed;

            float rotatedX = cos(rotationAngle) * x - sin(rotationAngle) * y;
            float rotatedY = sin(rotationAngle) * x + cos(rotationAngle) * y;

            frames.push_back(frame);
            positions.push_back(VEC3(cubeCenter[0] + rotatedX, cubeCenter[1] + waveHeight, cubeCenter[2] + rotatedY));
            interps.push_back(0);
        }

        // Set position keys for the cube model
        cubeModel.setPosKeys(frames, positions, interps);

        // Rotation and active keys
        cubeModel.setRotKeys({0, 180, 360, 540, 720, 900, 1080, 1260, 1440, 1620, 1774},
                             {VEC3(0,0,0), VEC3(PI/2.0,0,0), VEC3(PI,0,0), VEC3(3*PI/2.0,0,0), VEC3(0,0,0),
                             VEC3(0,0,0), VEC3(PI/2.0,0,0), VEC3(PI,0,0), VEC3(3*PI/2.0,0,0), VEC3(0,0,0), VEC3(PI/2.0,0,0)},
                             {0, 0, 0, 0, 0});
        cubeModel.setActiveKeys({0,1800}, {1, 1});

        // Add the cube model to the scene
        cubeModel.addToScene(frame_num);

        // Update angle and radius for the next cube in the spiral
        a += 2 * PI / (137.5 / 360); // Golden angle
        radius += 0.02; // Adjust for spacing of the cubes
    }
}

void buildCubes(float frame_num){
  // load model
  float radius_1 = 1.0;
  float radius_2 = 2.0;
  float radius_3 = 3.0;
  float radius_4 = 4.0;
  VEC3 spiralCenter = figureLocation;

  //outer circle
  for(int i = 0; i < 6; i++){
    for(int j = 0; j < 32; j++){
      float angle = j * PI/16 + i * PI/32;
      while (angle > 2*PI){
        angle = angle - 2*PI;
      }
      Model test;
      test.Load("resources/test1.stl");
      VEC3 outer_color_255 (210, 214, 195);
      VEC3 outer_color_1 = color_255to1(outer_color_255);
      test.setColor(outer_color_1);

      //frame 0
      double y_incre_0 = 0.15 * i;
      double x_incre_0 = sin(angle) * radius_2;
      double z_incre_0 = cos(angle) * radius_2;
      VEC3 center_0 (0.987694, 0.0206927, -1.32609);
      VEC3 incre_0 (x_incre_0, y_incre_0, z_incre_0);

      //frame 300
      float angle_300 = angle + PI/2;
      while (angle_300 > 2*PI){
        angle_300 = angle_300 - 2*PI;
      }
      double y_incre_300 = 0.15 * i + 0.05;
      double x_incre_300 = sin(angle_300) * radius_3;
      double z_incre_300 = cos(angle_300) * radius_3;
      VEC3 center_300 (0.106954, 0.0463124, -0.678497);
      VEC3 incre_300 (x_incre_300, y_incre_300, z_incre_300);

      //frame 600
      float angle_600 = angle + PI;
      while (angle_600 > 2*PI){
        angle_600 = angle_600 - 2*PI;
      }
      double y_incre_600 = 0.15 * i + 0.1;
      double x_incre_600 = sin(angle_600) * radius_4;
      double z_incre_600 = cos(angle_600) * radius_4;
      VEC3 center_600 (0.0776122, 0.0749758, -0.213121);
      VEC3 incre_600 (x_incre_600, y_incre_600, z_incre_600);

      //frame 900
      float angle_900 = angle + PI * 3/2;
      while (angle_900 > 2*PI){
        angle_900 = angle_900 - 2*PI;
      }
      double y_incre_900 = 0.15 * i + 0.05;
      double x_incre_900 = sin(angle_900) * radius_4;
      double z_incre_900 = cos(angle_900) * radius_4;
      VEC3 center_900 (1.12374, 0.0602458, 0.429806);
      VEC3 incre_900 (x_incre_900, y_incre_900, z_incre_900);

      //frame 1200
      float angle_1200 = angle;
      while (angle_1200 > 2*PI){
        angle_1200 = angle_1200 - 2*PI;
      }
      double y_incre_1200 = 0.15 * i ;
      double x_incre_1200 = sin(angle_1200) * radius_4;
      double z_incre_1200 = cos(angle_1200) * radius_4;
      VEC3 center_1200 (0.94673, 0.0722061, -0.562125);
      VEC3 incre_1200 (x_incre_1200, y_incre_1200, z_incre_1200);

      //frame 1500
      float angle_1500 = angle + PI * 1/2;
      while (angle_1500 > 2*PI){
        angle_1500 = angle_1500 - 2*PI;
      }
      double y_incre_1500 = 0.15 * i + 0.05;
      double x_incre_1500 = sin(angle_1500) * radius_3;
      double z_incre_1500 = cos(angle_1500) * radius_3;
      VEC3 center_1500 (0.619105, 0.0538443, -0.198987);
      VEC3 incre_1500 (x_incre_1500, y_incre_1500, z_incre_1500);

      //frame 1800
      float angle_1800 = angle + PI;
      while (angle_1800 > 2*PI){
        angle_1800 = angle_1800 - 2*PI;
      }
      double y_incre_1800 = 0.15 * i ;
      double x_incre_1800 = sin(angle_1800) * radius_2;
      double z_incre_1800 = cos(angle_1800) * radius_2;
      VEC3 center_1800 (0.735377, 0.0320404, 1.14928);
      VEC3 incre_1800 (x_incre_1800, y_incre_1800, z_incre_1800);

      test.setPosKeys({0, 300, 600, 900, 1200, 1500, 1794},
                      {center_0+incre_0, center_300+incre_300, center_600+incre_600, center_900+incre_900, center_1200+incre_1200,
                      center_1500+incre_1500, center_1800+incre_1800},
                      {0,0,0,0,0,0});
      test.setScaleKeys({0},{VEC3(.0002, .0002, .0002)}, {0});
      test.setRotKeys({0, 450, 900, 1350, 1794},
                      {VEC3(-PI/2.0,0,0), VEC3(-PI/2.0,0,0), VEC3(-PI/2.0,0,0), VEC3(-PI/2.0,0,0), VEC3(-PI/2.0,0,0)},
                      {0, 0, 0, 0, 0});
      test.setActiveKeys({0,1800}, {1, 1});
      models.push_back(test);
    }
  }

  //inner circle
  for(int i = 0; i < 6; i++){
    for(int j = 0; j < 16; j++){
      float angle = j * PI/8 + i * PI/16;
      while (angle > 2*PI){
        angle = angle - 2*PI;
      }
      Model test;
      test.Load("resources/test1.stl");
      VEC3 inner_color_255 (224, 158, 159);
      VEC3 inner_color_1 = color_255to1(inner_color_255);
      test.setColor(inner_color_1);

      //frame 0
      double y_incre_0 = 0.15 * i;
      double x_incre_0 = sin(angle) * radius_1;
      double z_incre_0 = cos(angle) * radius_1;
      VEC3 center_0 (0.987694, 0.0206927, -1.32609);
      VEC3 incre_0 (x_incre_0, y_incre_0, z_incre_0);

      //frame 300
      float angle_300 = angle - PI/2;
      while (angle_300 < 0){
        angle_300 = angle_300 + 2*PI;
      }
      double y_incre_300 = 0.15 * i + 0.05;
      double x_incre_300 = sin(angle_300) * radius_2;
      double z_incre_300 = cos(angle_300) * radius_2;
      VEC3 center_300 (0.106954, 0.0463124, -0.678497);
      VEC3 incre_300 (x_incre_300, y_incre_300, z_incre_300);

      //frame 600
      float angle_600 = angle - PI;
      while (angle_600< 0){
        angle_600 = angle_600 + 2*PI;
      }
      double y_incre_600 = 0.15 * i + 0.1;
      double x_incre_600 = sin(angle_600) * radius_3;
      double z_incre_600 = cos(angle_600) * radius_3;
      VEC3 center_600 (0.0776122, 0.0749758, -0.213121);
      VEC3 incre_600 (x_incre_600, y_incre_600, z_incre_600);

      //frame 900
      float angle_900 = angle - PI * 3/2;
      while (angle_900 < 0){
        angle_900 = angle_900 + 2*PI;
      }
      double y_incre_900 = 0.15 * i + 0.05;
      double x_incre_900 = sin(angle_900) * radius_3;
      double z_incre_900 = cos(angle_900) * radius_3;
      VEC3 center_900 (1.12374, 0.0602458, 0.429806);
      VEC3 incre_900 (x_incre_900, y_incre_900, z_incre_900);


      //frame 1200
      float angle_1200 = angle;
      while (angle_1200 < 0){
        angle_1200 = angle_1200 + 2*PI;
      }
      double y_incre_1200 = 0.15 * i ;
      double x_incre_1200 = sin(angle_1200) * radius_3;
      double z_incre_1200 = cos(angle_1200) * radius_3;
      VEC3 center_1200 (0.94673, 0.0722061, -0.562125);
      VEC3 incre_1200 (x_incre_1200, y_incre_1200, z_incre_1200);

      //frame 1500
      float angle_1500 = angle - PI * 1/2;
      while (angle_1500 < 0){
        angle_1500 = angle_1500 + 2*PI;
      }
      double y_incre_1500 = 0.15 * i + 0.05;
      double x_incre_1500 = sin(angle_1500) * radius_2;
      double z_incre_1500 = cos(angle_1500) * radius_2;
      VEC3 center_1500 (0.619105, 0.0538443, -0.198987);
      VEC3 incre_1500 (x_incre_1500, y_incre_1500, z_incre_1500);

      //frame 1800
      float angle_1800 = angle - PI;
      while (angle_1800 < 0){
        angle_1800 = angle_1800 + 2*PI;
      }
      double y_incre_1800 = 0.15 * i ;
      double x_incre_1800 = sin(angle_1800) * radius_1;
      double z_incre_1800 = cos(angle_1800) * radius_1;
      VEC3 center_1800 (0.735377, 0.0320404, 1.14928);
      VEC3 incre_1800 (x_incre_1800, y_incre_1800, z_incre_1800);

      test.setPosKeys({0, 300, 600, 900, 1200, 1500, 1794},
                     {center_0+incre_0, center_300+incre_300, center_600+incre_600, center_900+incre_900, center_1200+incre_1200,
                      center_1500+incre_1500, center_1800+incre_1800},
                      {0,0,0,0,0,0});
      test.setScaleKeys({0},{VEC3(.0002, .0002, .0002)}, {0});
      test.setRotKeys({0, 450, 900, 1350, 1800},
                      {VEC3(-PI/2.0,0,0), VEC3(-PI/2.0,0,0), VEC3(-PI/2.0,0,0), VEC3(-PI/2.0,0,0), VEC3(-PI/2.0,0,0)},
                      {0, 0, 0, 0, 0});
      test.setActiveKeys({0,1800}, {1, 1});
      models.push_back(test);
    }
  }

  for(Model m : models){
    m.addToScene (frame_num);
  }
}

void buildLights(){
  VEC3 lightColor (1, 1, 1);
  // VEC3 lightPos (0.0, 10, 0.0);
  VEC3 lightPos (headLocation[0], 10.0, headLocation[2]);
  PointLight* l1 = new PointLight (lightPos, lightColor);
  lights.push_back(l1);
}

void buildScene(float frame_num)
{
  primitives.clear();
  lights.clear();
  buildFigure(frame_num);
  // buildGround();
  buildFibCubes(frame_num);
  buildLights();
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

VEC3 randomPointOnDisc() {
  float r = sqrt((float)rand() / RAND_MAX);
  float theta = ((float)rand() / RAND_MAX) * 2 * PI;
  return VEC3(r * cos(theta), r * sin(theta), 0);
}

float* renderPPM (int& xRes, int& yRes, int frame_num){
  const int totalCells = xRes * yRes;
  float* ppmOut = new float[3 * totalCells];
  const float halfY = (lookingAt - eye).norm() * tan(45.0f / 360.0f * M_PI);
  const float halfX = halfY * 4.0f / 3.0f;

  // DoF parameters
  float startApertureSize = 0.6f;
  float endApertureSize = 0.0f;
  float apertureRadius;

  const VEC3 cameraZ = (lookingAt - eye).normalized();
  const VEC3 cameraX = up.cross(cameraZ).normalized();
  const VEC3 cameraY = cameraZ.cross(cameraX).normalized();

    if (frame_num <= 180) {
   // Starting aperture size
    apertureRadius = startApertureSize * (1 - float(frame_num) / 180.0f);
    float focalLength = 7.0f;    // Distance of the focus plane
    int raysPerPixel = 4;

    for (int y = 0; y < yRes; y++){
      for (int x = 0; x < xRes; x++)
      {

        VEC3 colorSum (0,0,0);
        int rays = raysPerPixel;

        for (int r = 0; r < raysPerPixel; r++) {
          // Calculate ray direction as before
          const float ratioX = 1.0f - x / float(xRes) * 2.0f;
          const float ratioY = 1.0f - y / float(yRes) * 2.0f;
          const VEC3 rayHitImage = lookingAt + 
                                  ratioX * halfX * cameraX +
                                  ratioY * halfY * cameraY;
          VEC3 rayDir = (rayHitImage - eye).normalized();

          // Adjust origin for DoF
          VEC3 randomPointInAperture = apertureRadius * randomPointOnDisc();
          VEC3 newEye = eye + randomPointInAperture;
          VEC3 focalPoint = eye + rayDir * focalLength;

          // Adjust ray direction for new origin
          rayDir = (focalPoint - newEye).normalized();

          // Get color for this ray
          VEC3 pixelColor;
          rayColor(newEye, rayDir, pixelColor, frame_num);
          colorSum += pixelColor;
        }

        VEC3 color = colorSum / float(raysPerPixel);

        ppmOut[3 * (y * xRes + x)] = clamp(color[0]) * 255.0f;
        ppmOut[3 * (y * xRes + x) + 1] = clamp(color[1]) * 255.0f;
        ppmOut[3 * (y * xRes + x) + 2] = clamp(color[2]) * 255.0f;
      }
    }
  }else{
     for (int y = 0; y < yRes; y++){
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
          rayColor(eye, rayDir, color, frame_num);
          ppmOut[3 * (y * xRes + x)] = clamp(color[0]) * 255.0f;
          ppmOut[3 * (y * xRes + x) + 1] = clamp(color[1]) * 255.0f;
          ppmOut[3 * (y * xRes + x) + 2] = clamp(color[2]) * 255.0f;
        }
     }
  }
  return ppmOut;
}

void renderImage(int& xRes, int& yRes, const string& filename, int frame_num)
{
  // allocate the final image
  const int totalCells = xRes * yRes;
  float* ppmOut = new float[3 * totalCells];

  // Motion Blur parameters
  const int startBlurFrame = 180;
  const int endBlurFrame = 1800;
  bool applyMotionBlur = frame_num >= startBlurFrame && frame_num <= endBlurFrame;
  int motionBlurSamples = applyMotionBlur ? 5 : 1;
  float blurInterval = 1.0f / motionBlurSamples;

  if(applyMotionBlur){
    float* ppmSum = new float [3 * totalCells];
    for(int i = 0; i < totalCells; i++){
      ppmSum[i] = 0;
    }
    for(int i = -2; i < 3; i++){
      buildScene (frame_num + i);
      ppmSum = addPPM(ppmSum, renderPPM(xRes, yRes, frame_num), totalCells*3);
    }
    ppmOut = dividePPM(ppmSum, motionBlurSamples,  totalCells*3);\
    for(int i = 0; i < totalCells; i++){
      ppmOut[i] = clamp(ppmOut[i]/255.0f)*255.0f;
    }
  }else{
    buildScene (frame_num);
    ppmOut = renderPPM(xRes, yRes, frame_num);
  }

  writePPM(filename, xRes, yRes, ppmOut);
}
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////


int main(int argc, char** argv)
{

  int startFrame;
  int endFrame;

  if(argc == 1){
    startFrame = 0;
    endFrame = 1800;
    }
  if(argc == 3){
    startFrame = atoi(argv[1]);
    endFrame = atoi(argv[2]);
  }

  hello = 0;
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

  // Note we're going 6 frames at a time, otherwise the animation
  // is really slow.
  for (int x = startFrame; x < endFrame; x += 6)
  {
    setSkeletonsToSpecifiedFrame(x);
    // buildScene(x);

    char buffer[256];
    snprintf(buffer, 256, "./frames/frame.%04i.ppm", x / 6);
    renderImage(windowWidth, windowHeight, buffer, x);
    cout << "Rendered " + to_string(x / 6) + " frames" << endl;
  }

  return 0;
}
