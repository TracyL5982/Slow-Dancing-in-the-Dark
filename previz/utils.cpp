
float clamp(float value)
{
  if (value < 0.0)      return 0.0;
  else if (value > 1.0) return 1.0;
  return value;
}

VEC3 lerpVec(VEC3 start, VEC3 end, float t){
	clamp(t);
	return start + (end-start)*t;
}

float lerpFloat(float start, float end, float t){
	clamp(t);
	return start + (end-start)*t;
}

VEC3 smerpVec(VEC3 start, VEC3 end, float t){
	clamp(t);
	t = 3*pow(t,2) - 2*pow(t,2);
	return start + (end-start)*t;
}

float smerpFloat(float start, float end, float t){
	t = 3*pow(t,2) - 2*pow(t,2);
	return start + (end-start)*t;
}

float keyframeFloat(vector<float> keyframes, vector<float> values, vector<int> interpolation, float t){
	int i = 0;
	while((i < keyframes.size()) && (t >= keyframes[i])) i++;
	int nextFrame = i;
	if(nextFrame == keyframes.size()) nextFrame--;
	int lastFrame = i-1;
	if(lastFrame < 0) lastFrame = 0;
	float percent;
	if(nextFrame != lastFrame){
		percent = (t-keyframes[lastFrame])/(keyframes[nextFrame]-keyframes[lastFrame]);
	}else{
		percent = 1;
	}

	return lerpFloat(values[lastFrame], values[nextFrame], percent);
}

VEC3 keyframeVec(vector<float> keyframes, vector<VEC3> values, vector<int> interpolation, float t){
	int i = 0;
	while((i < keyframes.size()) && (t >= keyframes[i])) i++;
	int nextFrame = i;
	if(nextFrame == keyframes.size()) nextFrame--;
	int lastFrame = i-1;
	if(lastFrame < 0) lastFrame = 0;
	float percent;
	if(nextFrame != lastFrame){
		percent = (t-keyframes[lastFrame])/(keyframes[nextFrame]-keyframes[lastFrame]);
	}else{
		percent = 1;
	}

	if(interpolation[lastFrame] == 0){
		return lerpVec(values[lastFrame], values[nextFrame], percent);
	}else if(interpolation[lastFrame] == 1){
		return smerpVec(values[lastFrame], values[nextFrame], percent);
	}

}

float min(float a, float b){
	if(a < b){
		return a;
	}else{
		return b;
	}
}

float max(float a, float b){
	if(a > b){
		return a;
	}else{
		return b;
	}
}

float area(VEC3 v0, VEC3 v1){
  VEC3 cross = v0.cross(v1);
  return cross.norm()/2.0;
}

VEC4 extend(VEC3 in){
	return VEC4(in[0], in[1], in[2], 1);
}

VEC3 truncate(VEC4 in){
	return VEC3(in[0], in[1], in[2]);
}

MATRIX4 rotMatrix(VEC3 rotate, VEC3 anchor){
	float rad_x = rotate[0];
	float rad_y = rotate[1];
	float rad_z = rotate[2];
	MATRIX4 RX;
	RX.setZero();
	RX(0,0) = 1;
	RX(1,1) = cos(rad_x);
	RX(1,2) = -sin(rad_x);
	RX(2,1) = sin(rad_x);
	RX(2,2) = cos(rad_x);
	RX(3,3) = 1;

	MATRIX4 RY;
	RY.setZero();
	RY(1,1) = 1;
	RY(0,0) = cos(rad_y);
	RY(2,0) = -sin(rad_y);
	RY(0,2) = sin(rad_y);
	RY(2,2) = cos(rad_y);
	RY(3,3) = 1;

	MATRIX4 RZ;
	RZ.setZero();
	RZ(2,2) = 1;
	RZ(0,0) = cos(rad_z);
	RZ(0,1) = -sin(rad_z);
	RZ(1,0) = sin(rad_z);
	RZ(1,1) = cos(rad_z);
	RZ(3,3) = 1;

	MATRIX4 T1;
	T1.setZero();
	T1(0,0) = 1;
	T1(1,1) = 1;
	T1(2,2) = 1;
	T1(3,3) = 1;
	T1(0,3) = -anchor[0];
	T1(1,3) = -anchor[1];
	T1(2,3) = -anchor[2];

	MATRIX4 T2;
	T2.setZero();
	T2(0,0) = 1;
	T2(1,1) = 1;
	T2(2,2) = 1;
	T2(3,3) = 1;
	T2(0,3) = anchor[0];
	T2(1,3) = anchor[1];
	T2(2,3) = anchor[2];

	return T2*RX*RY*RZ*T1;
}

MATRIX4 scaleMatrix(VEC3 scale, VEC3 anchor){
	float scale_x = scale[0];
	float scale_y = scale[1];
	float scale_z = scale[2];
	MATRIX4 T1;
	T1.setZero();
	T1(0,0) = 1;
	T1(1,1) = 1;
	T1(2,2) = 1;
	T1(3,3) = 1;
	T1(0,3) = -anchor[0];
	T1(1,3) = -anchor[1];
	T1(2,3) = -anchor[2];

	MATRIX4 T2;
	T2.setZero();
	T2(0,0) = 1;
	T2(1,1) = 1;
	T2(2,2) = 1;
	T2(3,3) = 1;
	T2(0,3) = anchor[0];
	T2(1,3) = anchor[1];
	T2(2,3) = anchor[2];

	MATRIX4 SCALE;
	SCALE.setZero();
	SCALE(0,0) = scale_x;
	SCALE(1,1) = scale_y;
	SCALE(2,2) = scale_z;
	SCALE(3,3) = 1;

	return T2*SCALE*T1;
}

MATRIX4 translateMatrix(VEC3 translate){
	float trans_x = translate[0];
	float trans_y = translate[1];
	float trans_z = translate[2];
	MATRIX4 T;
	T.setZero();
	T(0,0) = 1;
	T(1,1) = 1;
	T(2,2) = 1;
	T(3,3) = 1;
	T(0,3) = trans_x;
	T(1,3) = trans_y;
	T(2,3) = trans_z;

	return T;
}