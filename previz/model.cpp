class Model{
	vector<triangle> triangles;
	VEC3 scale;
	VEC3 anchor;
	VEC3 rotation;
	VEC3 position;
	VEC3 color;

	vector<float> scaleKeys;
	vector<VEC3> scaleVals;
	vector<int> scaleInterps;
	vector<float> rotKeys;
	vector<VEC3> rotVals;
	vector<int> rotInterps;
	vector<float> posKeys;
	vector<VEC3> posVals;
	vector<int> posInterps;
	vector<float> activeKeys;
	vector<float> activeVals;

public:
	void Load(string filename){
		auto info = parse_stl(filename);
		triangles = info.triangles;
		color = VEC3(1,1,1);
		scale = VEC3(1,1,1);
		anchor = VEC3(0,0,0);
		rotation = VEC3(0,0,0);
		position = VEC3(0,0,0);
	}

	void setColor(VEC3 color_in){
		color = color_in;
	}

	void addToScene(int frame_num){
		float isActive = keyframeFloat(activeKeys, activeVals, {}, frame_num);
		if(isActive != 0){
			vector<Primitive *> tris;
			for(triangle t : triangles){
				VEC3 vert3 = VEC3(t.v1.x, t.v1.y, t.v1.z);
				VEC3 vert2 = VEC3(t.v2.x, t.v2.y, t.v2.z);
				VEC3 vert1 = VEC3(t.v3.x, t.v3.y, t.v3.z);
                vector<VEC3> verts;
				verts.push_back(vert1);
                verts.push_back(vert2);
                verts.push_back(vert3);
				MATRIX4 M_scale = scaleMatrix(keyframeVec(scaleKeys, scaleVals, scaleInterps, frame_num), anchor);
				MATRIX4 M_translate = translateMatrix(keyframeVec(posKeys, posVals, posInterps, frame_num));
				MATRIX4 M_rotate = rotMatrix(keyframeVec(rotKeys, rotVals, rotInterps, frame_num), anchor);

				for(int i = 0; i < 3; i++){
					verts[i] = truncate(M_scale * extend(verts[i]));
					verts[i] = truncate(M_rotate * extend(verts[i]));
					verts[i] = truncate(M_translate * extend(verts[i]));
				}

				Tri * tri = new Tri({verts[0], verts[1], verts[2]}, color);
                primitives.push_back(tri);
				// tris.push_back(tri);
			}
			// bounders.push_back(new BoundingBox(tris)); // Add as bounded tris
		}
	}

	void setScaleKeys(vector<float> keyframes, vector<VEC3> values, vector<int> interps){
		scaleKeys = keyframes;
		scaleVals = values;
		scaleInterps = interps;
	}

	void setRotKeys(vector<float> keyframes, vector<VEC3> values, vector<int> interps){
		rotKeys = keyframes;
		rotVals = values;
		rotInterps = interps;
	}

	void setPosKeys(vector<float> keyframes, vector<VEC3> values, vector<int> interps){
		posKeys = keyframes;
		posVals = values;
		posInterps = interps;
	}

	void setActiveKeys(vector<float> keyframes, vector<float> values){
		activeKeys = keyframes;
		activeVals = values;
	}


	void setScale(VEC3 scale_in){
		scale = scale_in;
	}

	void setAnchor(VEC3 anchor_in){
		anchor = anchor_in;
	}

	void setRotation(VEC3 rotation_in){
		rotation = rotation_in;
	}

	void setPosition(VEC3 position_in){
		position = position_in;
	}

};