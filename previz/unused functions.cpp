bool raySphereIntersect(VEC3& center,
                        float radius,
                        const VEC3& rayPos,
                        const VEC3& rayDir,
                        float& t)
{
  const VEC3 op = center - rayPos;
  const float eps = 1e-8;
  const float b = op.dot(rayDir);
  float det = b * b - op.dot(op) + radius * radius;

  // determinant check
  if (det < 0)
    return false;

  det = sqrt(det);
  t = b - det;
  if (t <= eps)
  {
    t = b + det;
    if (t <= eps)
      t = -1;
  }

  if (t < 0) return false;
  return true;
}

bool rayCylinderIntersect(const VEC3& rayPos, const VEC3& rayDir,
                          const VEC3& cylStart, const VEC3& cylEnd,
                          const float cylRadius, float& tMin) {
    VEC3 cylAxis = cylEnd - cylStart;
    float cylLength = cylAxis.norm();
    cylAxis.normalize();

    // Compute some necessary dot products for the coefficients of the quadratic equation
    VEC3 deltaP = rayPos - cylStart;
    float a = (rayDir - cylAxis * (rayDir.dot(cylAxis))).squaredNorm();
    float b = 2 * (rayDir - cylAxis * (rayDir.dot(cylAxis))).dot(deltaP - cylAxis * (deltaP.dot(cylAxis)));
    float c = (deltaP - cylAxis * (deltaP.dot(cylAxis))).squaredNorm() - cylRadius * cylRadius;

    // Solve the quadratic equation for t
    float discriminant = b * b - 4 * a * c;
    if (discriminant < 0) return false; // No real roots, no intersection

    float sqrtDiscriminant = sqrt(discriminant);
    float t0 = (-b - sqrtDiscriminant) / (2 * a);
    float t1 = (-b + sqrtDiscriminant) / (2 * a);

    // Check if the intersections are within the cylinder caps
    float z0 = (rayPos + t0 * rayDir - cylStart).dot(cylAxis);
    float z1 = (rayPos + t1 * rayDir - cylStart).dot(cylAxis);

    if (z0 < 0 || z0 > cylLength) t0 = FLT_MAX;
    if (z1 < 0 || z1 > cylLength) t1 = FLT_MAX;

    tMin = std::min(t0, t1);
    return tMin < FLT_MAX;
}


//////////////////////////////////////////////////////////////////////////////////
// calculate determinant
//////////////////////////////////////////////////////////////////////////////////
double getDeterminant(VEC3 a, VEC3 b, VEC3 c){
  return a[0] * (b[1] * c[2] - b[2] * c[1]) -
         a[1] * (b[0] * c[2] - b[2] * c[0]) +
         a[2] * (b[0] * c[1] - b[1] * c[0]);
}

VEC3 get_beta_gamma_t(VEC3 a_b, VEC3 a_c, VEC3 d, VEC3 a_e){
  double A = getDeterminant(a_b, a_c, d);
  if (abs(A) < std::numeric_limits<double>::epsilon()) {
    // Return a VEC3 with negative t to indicate no intersection
    return VEC3(-1, 0, 0);
  }
  double beta_top = getDeterminant(a_e, a_c, d);
  double gamma_top = getDeterminant(a_b, a_e, d);
  double t_top = -getDeterminant(a_b, a_c, a_e);
  double beta = beta_top/A;
  double gamma = gamma_top/A;
  double t = -t_top / A;
  return VEC3(beta, gamma, t);
}

//////////////////////////////////////////////////////////////////////////////////
// check triangle intersection
//////////////////////////////////////////////////////////////////////////////////
bool checkTriangleIntersection(vector<VEC3>& vertices, VEC3 d, VEC3 eye, double*t){
  VEC3 a = vertices[0];
  VEC3 b = vertices[1];
  VEC3 c = vertices[2];
  VEC3 a_b = b - a;
  VEC3 a_c = c - a;
  VEC3 a_e = eye - a;
  // Plane
  /*
  P = (x, y, z);
  N = a_b.cross(a_c)
  d = vertices[0]

  Ray
  P_0 = eye
  t = t
  V = d
  */
  // Step 1: find t
  VEC3 planeNormal = (a_b.cross(a_c)).normalized();
  float t1 = (vertices[0] - eye).dot(planeNormal) / (d.dot(planeNormal));
  if(t1 <= 0){
    return false;
  }

  VEC3 intersectionPoint = eye + d * t1;
  // Step 2: find whether intersectionPoint is in triangle
  VEC3 bgt = get_beta_gamma_t(vertices[0], vertices[1], vertices[2], intersectionPoint);
  double beta = bgt[0];
  double gamma = bgt[1];
  double alpha = bgt[2];
  if(alpha < 0) return false;
  if(gamma < 0 || gamma > 1) return false;
  if(beta < 0 || beta > 1-gamma) return false;
  *t = t1;
  return true;
}

VEC3 getCheckeredColor(const VEC3& point) {
  int checkSize = 1; // the size of the squares in the check pattern
  int ix = floor(point[0] / checkSize);
  int iz = floor(point[2] / checkSize);
  int checkPattern = (ix + iz) % 2;
  return (checkPattern == 0) ? VEC3(1, 1, 1) : VEC3(0, 0, 1); // white or black
}

void buildCube (vector<VEC3> bottom, double height, const VEC3& rayPos, const VEC3& rayDir, VEC3& pixelColor){
  double tMinFound = FLT_MAX;
  double tMin = FLT_MAX;

  for(int i = 0; i < 3; i++){
    VEC3 bottom_1 = bottom[i];
    VEC3 bottom_2 = bottom[i+1];
    VEC3 top_1 = bottom_1 + VEC3(0, height, 0);
    VEC3 top_2 = bottom_2 + VEC3(0, height, 0);

    vector<VEC3> triangle1;
    triangle1.push_back(bottom_1);
    triangle1.push_back(bottom_2);
    triangle1.push_back(top_1);

    vector<VEC3> triangle2;
    triangle2.push_back(top_1);
    triangle2.push_back(bottom_1);
    triangle2.push_back(top_2);
    double tMin = FLT_MAX;
    if(checkTriangleIntersection(triangle1, rayDir, rayPos, &tMin)){
      pixelColor = VEC3(0,0,1);
      return;
    }
    if(checkTriangleIntersection(triangle2, rayDir, rayPos, &tMin)){
      pixelColor = VEC3(0,0,1);
      return;
    }
  }
}

void buildArch(const VEC3& rayPos, const VEC3& rayDir, VEC3& pixelColor){
  pixelColor = VEC3(1,1,1);
  int hitID = -1;
  float tMinFound = FLT_MAX;
  double tMin = FLT_MAX;

  //Sphere top
  VEC3 sphereCenter (3, 4, 0);
  float sphereRadius = 1;

  float time;
  if (raySphereIntersect (sphereCenter, sphereRadius, rayPos, rayDir, time))
  {
   VEC3 intersectionPoint = rayPos + time * rayDir;
   if(intersectionPoint[1] >=4){
    pixelColor = VEC3(0,0,1);
   }
   return;
  }

  vector<VEC3> cube1;
  cube1.push_back(VEC3(2, -0.0586798, -1));
  cube1.push_back(VEC3(2, -0.0586798, -0.5));
  cube1.push_back(VEC3(2.5, -0.0586798, -0.5));
  cube1.push_back(VEC3(2.5, -0.0586798, -1));
  buildCube(cube1, 4, rayPos, rayDir, pixelColor);

  vector<VEC3> cube2;
  cube2.push_back(VEC3(2, -0.0586798, 0.5));
  cube2.push_back(VEC3(2, -0.0586798, 1));
  cube2.push_back(VEC3(2.5, -0.0586798, 1));
  cube2.push_back(VEC3(2.5, -0.0586798, 0.5));
  buildCube(cube2, 4, rayPos, rayDir, pixelColor);

  vector<VEC3> cube3;
  cube3.push_back(VEC3(3.5, -0.0586798, -1));
  cube3.push_back(VEC3(3.5, -0.0586798, -0.5));
  cube3.push_back(VEC3(4, -0.0586798, -0.5));
  cube3.push_back(VEC3(4, -0.0586798, -1));
  buildCube(cube3, 4, rayPos, rayDir, pixelColor);

  vector<VEC3> cube4;
  cube4.push_back(VEC3(3.5, -0.0586798, 0.5));
  cube4.push_back(VEC3(3.5, -0.0586798, 1));
  cube4.push_back(VEC3(4, -0.0586798, 1));
  cube4.push_back(VEC3(4, -0.0586798, 0.5));
  buildCube(cube4, 4, rayPos, rayDir, pixelColor);

  return;
}
