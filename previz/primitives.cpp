struct Ray{
	VEC3 o; // origin
	VEC3 d; // direction
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// PRIMITIVES ///////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Primitive defs
class Primitive{
	// bool lights = true;
	// bool castShadows = true;
public:
	// bool acceptsLights(){return lights;}
	// void setAcceptsLights(bool in){lights = in;}
	// bool castsShadows(){return castShadows;}
	// void setCastsShadows(bool in){castShadows = in;}
	virtual bool intersectRay(Ray &r, float &t, VEC3 &normal) = 0;
	virtual VEC3 getNormal(Ray &r, float &t) = 0;
	virtual VEC3 getColor(Ray &r, float &t) = 0;
	virtual VEC3 getMin() = 0;
	virtual VEC3 getMax() = 0;
};


class Tri : public Primitive{
	vector<VEC3> verts;
	VEC3 color;
	VEC3 mins;
	VEC3 maxes;

public:
	Tri(vector<VEC3> verts_in, VEC3 color_in){
		verts = verts_in;
		color = color_in;
        for(int i = 0; i < 3; i++){
            mins[i] = FLT_MAX;
            maxes[i] = -FLT_MAX;
        }
		for(int i = 0; i < 3; i++){ // For each vertex
			for(int j = 0; j < 3; j++){ // For x, y, z
				if(verts_in[i][j] > maxes[j])
					maxes[j] = verts_in[i][j];
				if(verts_in[i][j] < mins[j])
					mins[j] = verts_in[i][j];
			}
		}
	}

	VEC3 getMin(){return mins;}
	VEC3 getMax(){return maxes;}

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

    bool intersectRay(Ray& r, float &t, VEC3 &normalOut){
        		// Calculate normal
		VEC3 normal = ((verts[1]-verts[0]).cross(verts[2]-verts[0])).normalized();
		normalOut = -normal;

		// Check if parallel
		if(normal.dot(r.d) == 0){
			return false;
		}

		float D = normal.dot(verts[0]);
		t = (D - normal.dot(r.o))/(normal.dot(r.d));
		if(t < 0){
			return false;
		}
		VEC3 Q = r.o + t*r.d;

		// Check if inside triangle
		if(((verts[1]-verts[0]).cross(Q-verts[0])).dot(normal) >= 0){
			if(((verts[2]-verts[1]).cross(Q-verts[1])).dot(normal) >= 0){
				if(((verts[0]-verts[2]).cross(Q-verts[2])).dot(normal) > 0){
					return true;
				}
			}
		}
		return false;
    }

	VEC3 getNormal(Ray &r, float &t){
		return verts[0].cross(verts[1]).normalized();
	}

	VEC3 getColor(Ray &r, float &t){
		return color;
	}
};

class Cylinder : public Primitive{
	vector<VEC3> verts;
	float radius;
	VEC3 color;
	VEC3 mins;
	VEC3 maxes;

public:
	Cylinder(vector<VEC3> verts_in, float rad_in, VEC3 color_in){
		verts = verts_in;
		radius = rad_in;
		color = color_in;

		vector<VEC4> candidates; // x candidates, y candidates, z candidates
		candidates.push_back(VEC4(verts[0][0] - radius, verts[0][0] + radius, verts[1][0] - radius, verts[1][0] + radius)); // [0] = x
		candidates.push_back(VEC4(verts[0][1] - radius, verts[0][1] + radius, verts[1][1] - radius, verts[1][1] + radius)); // [0] = y
		candidates.push_back(VEC4(verts[0][2] - radius, verts[0][2] + radius, verts[1][2] - radius, verts[1][2] + radius)); // [0] = z
		for(int i = 0; i < 3; i++){ // For x, y, z
			float min = FLT_MAX;
			float max = -FLT_MAX;
			for(int j = 0; j < 4; j++){ // For each candidate
				float test = candidates[i][j];
				if(test < min)
					min = test;
				if(test > max)
					max = test;
			}
			mins[i] = min;
			maxes[i] = max;
		}
	}

	VEC3 getMin(){return mins;}
	VEC3 getMax(){return maxes;}

	bool intersectRay(Ray &r, float &t, VEC3 &normal){
		VEC3 dir = verts[1] - verts[0];
		VEC3 u = (verts[1] - verts[0]).normalized();
		VEC3 v;
		if(abs(u[0]) == 1){
			//cout << "Special case" << endl;
			v = u.cross(u+VEC3(0,1,0));
		}else{
			v = u.cross(u+VEC3(1,0,0));
		}

		VEC3 w = u.cross(v).normalized();

		MATRIX3 A;
		A(0,0) = u[0]; A(0,1) = v[0]; A(0,2) = w[0];
		A(1,0) = u[1]; A(1,1) = v[1]; A(1,2) = w[1];
		A(2,0) = u[2]; A(2,1) = v[2]; A(2,2) = w[2];
		MATRIX3 Ai = A.inverse();

		// Translate and rotate vectors
		VEC3 v0_t = Ai*(verts[0]-verts[0]); // You probably don't actuall have to calculate the whole vector
		VEC3 v1_t = Ai*(verts[1]-verts[0]);
		VEC3 ori_t = Ai*(r.o-verts[0]);
		VEC3 dir_t = Ai*r.d;

		// Perform circle intersection test
		VEC2 D = VEC2(dir_t[1], dir_t[2]);
		VEC2 O = VEC2(ori_t[1], ori_t[2]);
		float a = D.dot(D);
		float b = 2*D.dot(O);
		float c = O.dot(O) - pow(radius,2);
		float det = pow(b,2) - 4*a*c;
		if(det < 0){
			//cout << "no intersection" << endl;
			return false; // No intersection
		}
		float t1 = (-b + sqrt(pow(b,2) - 4*a*c))/(2*a);
		float t2 = (-b - sqrt(pow(b,2) - 4*a*c))/(2*a);
		if((t1 < 0) && (t2 < 0))
			return false; // Intersect behind
		if(t1 < 0)
			t1 = t2;
		if(t2 < 0)
			t2 = t1;

		// Check side intersections
		bool t1Int = false;
		VEC3 intPos_t = ori_t + t1*dir_t;
		if((intPos_t[0] > v0_t[0]) && (intPos_t[0] < v1_t[0]))
			t1Int = true;

		bool t2Int = false;
		intPos_t = ori_t + t2*dir_t;
		if((intPos_t[0] > v0_t[0]) && (intPos_t[0] < v1_t[0]))
			t2Int = true;

		// check for cap intersection
		bool topInt = false;
		float top_u = v1_t[0];
		float t_topInt = (top_u-ori_t[0])/dir_t[0];
		VEC3 topInt_point = ori_t + t_topInt*dir_t;
		if((t_topInt > 0) && (pow(topInt_point[1],2) + pow(topInt_point[2],2)) <= pow(radius,2))
			topInt = true;

		bool bottomInt = false;
		float t_bottomInt = (0-ori_t[0])/dir_t[0];
		VEC3 bottomInt_point = ori_t + t_bottomInt*dir_t;
		if((t_bottomInt > 0) &&(pow(bottomInt_point[1],2) + pow(bottomInt_point[2],2)) <= pow(radius,2))
			bottomInt = true;

		if((!t2Int) && (!t1Int) && (!topInt) && (!bottomInt))
			return false;

		int hitID = -1; // -1 = no intersection, 1 = t1, 2 = t2, 3 = top cap, 4 = bottom cap
		float t_min = FLT_MAX;

		if(t1Int  && (t1 < t_min)){
			t_min = t1;
			hitID = 1;
		}
		if(t2Int  && (t2 < t_min)){
			t_min = t2;
			hitID = 2;
		}
		if(topInt && (t_topInt < t_min)){
			t_min = t_topInt;
			hitID = 3;
		}
		if(bottomInt && (t_bottomInt < t_min)){
			t_min = t_bottomInt;
			hitID = 4;
		}

		t = t_min;
		intPos_t = ori_t + t*dir_t;
		if((hitID == 1) || (hitID == 2)){
			normal = VEC3(0, intPos_t[1], intPos_t[2]);
			normal = A*normal;
			normal.normalize();
		}else if(hitID == 3){
			normal = VEC3(1, 0, 0);
			normal = A*normal;
			normal.normalize();
		}else if(hitID == 4){
			normal = VEC3(-1, 0, 0);
			normal = A*normal;
			normal.normalize();
		}
		return true;
	}

	VEC3 getNormal(Ray &r, float &t){
		return VEC3(0,0,0);
	}

	VEC3 getColor(Ray &r, float &t){
		return color;
	}
};


class Sphere : public Primitive{
	VEC3 center;
	float radius;
	VEC3 color;
	VEC3 mins;
	VEC3 maxes;
public:

	Sphere(VEC3 center_in, float rad_in, VEC3 color_in){
		center = center_in;
		radius = rad_in;
		color = color_in;
		maxes = VEC3(center[0]+rad_in, center[1]+rad_in, center[2]+rad_in);
		mins = VEC3(center[0]-rad_in, center[1]-rad_in, center[2]-rad_in);
	}

	VEC3 getMin(){return mins;}
	VEC3 getMax(){return maxes;}

	bool intersectRay(Ray &r, float &t, VEC3 &normal){
		const VEC3 op = center - r.o;
	  const float eps = 1e-8;
	  const float b = op.dot(r.d);
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
  	normal = ((r.o + t*r.d) - center).normalized();
  	return true;
	}

	VEC3 getNormal(Ray &r, float &t){
		return VEC3(0,0,0);
	}

	VEC3 getColor(Ray &r, float &t){
		return color;
	}
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// BOUNDING /////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class BoundingBox{
	VEC3 max;
	VEC3 min;
	vector<Primitive *> primitives;
public:
	BoundingBox(VEC3 max_in, VEC3 min_in, vector<Primitive *> prim_in){
		max = max_in;
		min = min_in;
		primitives = prim_in;
	}

	BoundingBox(vector<Primitive *> prims){
		VEC3 mins;
		VEC3 maxes;
        for(int i = 0; i < 3; i++){
            mins[i] = FLT_MAX;
            maxes[i] = -FLT_MAX;
        }
		for(Primitive * p : prims){
			VEC3 thisMin = p->getMin();
			VEC3 thisMax = p->getMax();
			for(int i = 0; i < 3; i++){ // for x, y, z
				if(thisMax[i] > maxes[i])
					maxes[i] = thisMax[i];
				if(thisMin[i] < mins[i])
					mins[i] = thisMin[i];
			}
		}
		max = maxes;
		min = mins;
		primitives = prims;
	}

	VEC3 getMax(){return max;}
	VEC3 getMin(){return min;}

	vector<Primitive *> getPrimitives(){return primitives;}

	bool intersectRay(Ray r){
		VEC3 min_trans = min-r.o;
		VEC3 max_trans = max-r.o;

		// Check if any direction is 0 and will never enter the box
		for(int i = 0; i < 3; i++){
			if( (r.d[i] == 0) && ((r.o[i] < min[i]) || (r.o[i] > max[i])) )
				return false;
		}

        VEC3 tmins;
        for(int i = 0; i < 3; i++){
            tmins[i] = min_trans[i]/r.d[i];
        }
		if(r.d[0] == 0) tmins[0] = FLT_MAX;
		if(r.d[1] == 0) tmins[1] = FLT_MAX;
		if(r.d[2] == 0) tmins[2] = FLT_MAX;

		VEC3 tmaxes (max_trans[0]/r.d[0], max_trans[1]/r.d[1], max_trans[2]/r.d[2]);
		if(r.d[0] == 0) tmaxes[0] = -FLT_MAX;
		if(r.d[1] == 0) tmaxes[1] = -FLT_MAX;
		if(r.d[2] == 0) tmaxes[2] = -FLT_MAX;

		// Check if any intersection is fully behind ray
		if(((tmins[0] < 0) && (tmaxes[0] < 0)) || ((tmins[1] < 0) && (tmaxes[1] < 0)) || ((tmins[2] < 0) && (tmaxes[2] < 0)))
			return false;


		VEC3 hit1;
		VEC3 hit2;
		for(int i = 0; i < 3; i++){
			if(tmins[i] < tmaxes[i]){
				hit1[i] = tmins[i];
				hit2[i] = tmaxes[i];
			}else{
				hit2[i] = tmins[i];
				hit1[i] = tmaxes[i];
			}
		}



		if((hit1[0] > hit2[1]) || (hit1[1] > hit2[0]) || (hit1[0] > hit2[2]) || (hit1[2] > hit2[0])){
			return false;
		}else{
			return true;
		}
	}
};

BoundingBox * getBoundingBox(vector<Primitive *> prims){
	VEC3 mins (FLT_MAX, FLT_MAX, FLT_MAX);
	VEC3 maxes (-FLT_MAX, -FLT_MAX, -FLT_MAX);

	for(Primitive * p : prims){
		VEC3 thisMin = p->getMin();
		VEC3 thisMax = p->getMax();
		for(int i = 0; i < 3; i++){ // for x, y, z
			if(thisMax[i] > maxes[i])
				maxes[i] = thisMax[i];
			if(thisMin[i] < mins[i])
				mins[i] = thisMin[i];
		}
	}

	return new BoundingBox(maxes, mins, prims);
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// LIGHTS ///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// class Light{
// 	VEC3 pos;
// 	VEC3 color;
// public:
// 	virtual bool isVisible(Ray &r, vector<Primitive *> scene, vector<BoundingBox *> bounders, VEC3 &color) = 0;
// 	virtual float getIntensity(VEC3 point) = 0;
// 	void setPos(VEC3 pos_in){pos = pos_in;}
// 	VEC3 getPos(){return pos;}
// 	void setColor(VEC3 color_in){color = color_in;}
// 	VEC3 getColor(){return color;}
// };

// class PointLight: public Light{
// public:
// 	PointLight(VEC3 pos_in, VEC3 color_in){
// 		setPos(pos_in);
// 		setColor(color_in);
// 	}

// 	// bool isVisible(Ray &shadow, vector<Primitive *> scene, vector<BoundingBox *> bounders, VEC3 &color_in){
// 	// 	for(Primitive * p : scene){
// 	// 		float t;
// 	// 		VEC3 thisNorm;
// 	// 		if(p->castsShadows() && p->intersectRay(shadow, t, thisNorm))
// 	// 			return false;
// 	// 	}
// 	// 	color_in = getColor();
// 	// 	return true;
// 	// }

// 	float getIntensity(VEC3 point){
// 		float dist = (getPos() - point).norm();
// 		if(dist > .5){
// 			return 0;
// 		}else{
// 			return 1.0f - dist*2;
// 		}
// 	}
// };

// class SpotLight: public Light{
// 	VEC3 direction;
// 	float angle;
// public:
// 	SpotLight(VEC3 pos_in, VEC3 lookAt, float angle_in, VEC3 color_in){
// 		setPos(pos_in);
// 		setColor(color_in);
// 		direction = (lookAt - pos_in).normalized();
// 		angle = angle_in;
// 	}

// 	float getIntensity(VEC3 point){
// 		return 2;
// 	}

// 	bool isVisible(Ray &shadow, vector<Primitive *> scene, vector<BoundingBox *> bounders, VEC3 &color_in){

// 		float theta = acos((shadow.d).dot(-direction));
// 		if(theta > angle){
// 			//cout << "angle too great" << endl;
// 			return false;
// 		}

// 		// Intersect primitives
// 		for(Primitive * p : scene){
// 			float t;
// 			VEC3 thisNorm;
// 			if(p->castsShadows() && p->intersectRay(shadow, t, thisNorm))
// 				return false;
// 		}

// 		// Intersect bounders
// 		for(BoundingBox * b : bounders){
// 			if(b->intersectRay(shadow)){
// 				vector<Primitive *> prims = b->getPrimitives();
// 				for(Primitive * p : prims){
// 					float t;
// 					VEC3 thisNorm;
// 					if(p->castsShadows() && p->intersectRay(shadow, t, thisNorm))
// 						return false;
// 				}
// 			}
// 		}

// 		color_in = getColor();

// 		return true;
// 	}

// };