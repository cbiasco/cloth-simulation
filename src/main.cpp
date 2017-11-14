
// The loaders are included by glfw3 (glcorearb.h) if we are not using glew.
#ifdef USE_GLEW
#include <GL/glew.h>
#endif
#include <GLFW/glfw3.h>

// Includes
#include <omp.h>
#include "trimesh.hpp"
#include "shader.hpp"
#include <cstring> // memcpy
#include <random>
#include <algorithm>
#include <omp.h>

// Constants
#define WIN_WIDTH 800
#define WIN_HEIGHT 800
#define PI 4.0*atan(1.0)

#define DEBUG 0
#define USE_BVH 0
#define USE_OMP 0
#define FLOOR_EXISTS 1

typedef Vec<2,float> Vec2f; // Declare to use inside of deformableMesh

class Mat4x4 {
public:

	float m[16];

	Mat4x4(){ // Default: Identity
		m[0] = 1.f;  m[4] = 0.f;  m[8]  = 0.f;  m[12] = 0.f;
		m[1] = 0.f;  m[5] = 1.f;  m[9]  = 0.f;  m[13] = 0.f;
		m[2] = 0.f;  m[6] = 0.f;  m[10] = 1.f;  m[14] = 0.f;
		m[3] = 0.f;  m[7] = 0.f;  m[11] = 0.f;  m[15] = 1.f;
	}

	void make_identity(){
		m[0] = 1.f;  m[4] = 0.f;  m[8]  = 0.f;  m[12] = 0.f;
		m[1] = 0.f;  m[5] = 1.f;  m[9]  = 0.f;  m[13] = 0.f;
		m[2] = 0.f;  m[6] = 0.f;  m[10] = 1.f;  m[14] = 0.f;
		m[3] = 0.f;  m[7] = 0.f;  m[11] = 0.f;  m[15] = 1.f;
	}

	void print(){
		std::cout << m[0] << ' ' <<  m[4] << ' ' <<  m[8]  << ' ' <<  m[12] << "\n";
		std::cout << m[1] << ' ' <<   m[5] << ' ' <<  m[9]  << ' ' <<   m[13] << "\n";
		std::cout << m[2] << ' ' <<   m[6] << ' ' <<  m[10] << ' ' <<   m[14] << "\n";
		std::cout << m[3] << ' ' <<   m[7] << ' ' <<  m[11] << ' ' <<   m[15] << "\n";
	}

	void make_scale(float x, float y, float z){
		make_identity();
		m[0] = x; m[5] = y; m[10] = x;
	}
};

static inline const Vec3f operator*(const Mat4x4 &m, const Vec3f &v) {
	Vec3f r(m.m[0] * v[0] + m.m[4] * v[1] + m.m[8] * v[2],
		m.m[1] * v[0] + m.m[5] * v[1] + m.m[9] * v[2],
		m.m[2] * v[0] + m.m[6] * v[1] + m.m[10] * v[2]);
	return r;
}
static inline const Vec3f operator*(const double d, const Vec3f &v) {
	Vec3f r(v[0] * d, v[1] * d, v[2] * d);
	return r;
}
static inline const Vec3f operator*(const Vec3f &v, const double d) {
	Vec3f r(v[0] * d, v[1] * d, v[2] * d);
	return r;
}
static inline const Vec3f operator+(const Vec3f &v1, const Vec3f &v2) {
	Vec3f r(v1[0] + v2[0], v1[1] + v2[1], v1[2] + v2[2]);
	return r;
}


typedef struct {
	bool active = false;
	GLdouble prev_x;
	GLdouble prev_y;
} MouseInfo;


//Container class for our Spring data.
typedef struct {
	float springConstant;
	//equilibrium point of spring
	float restLength;
	//the dampening factor of the spring. Used to let the spring slowly move towards equilibrium.
	float dampeningFactor;

	//bool to check each frame whether the values for this spring have been calculated.
	bool sprungThisFrame;
	//point at which this spring will 'break'
	float tearLength;
}Spring;


//
//	Global state variables
//
namespace Globals {
	float win_width, win_height; // window size
	float aspect;
	int vert_dim = 2;
	GLuint position_vbo[1], colors_vbo[1], faces_ibo[1], normals_vbo[1], particles_vao;
	Vec3f lightDir = Vec3f(0, -1, -1);

	//  Model, view and projection matrices, initialized to the identity
	Mat4x4 model;
	Mat4x4 view;
	Mat4x4 projection;

	// Scene variables
	Vec3f eye;
	float near = .1;
	float far = 1000;
	float left = -.1;
	float right = .1;
	float top = .1;
	float bottom = -.1;
	Vec3f viewDir;
	Vec3f upDir;
	Vec3f rightDir;

	// Input variables
	bool key_up; // forward movement
	bool key_w;
	bool key_down; // backward movement
	bool key_s;
	bool key_right; // right strafing
	bool key_d;
	bool key_left; // left strafing
	bool key_a;
	bool key_rshift; // upward movement
	bool key_e;
	bool key_0; // downward movement
	bool key_q;
	bool key_rcontrol; // speed up
	bool key_lshift;
	bool paddle_up;
	bool paddle_down;
	bool paddle_stop;

	double theta;
	double phi;
	Mat4x4 xRot;
	Mat4x4 yRot;

	MouseInfo mouse;
	mcl::Shader currShader;
	GLuint tex;
	double currTime, prevTime;
	bool pause = false, addTimeMultiplier = false, subTimeMultiplier = false;
	double timeMultiplier = 1;
	double movementSpeed = 0.1;
	GLFWwindow *activeWindow;
}

class Sphere {
public:
	Vec3f pos;
	Vec3f vel;
	double rad;
	std::vector<Vec3f> points;
	std::vector<Vec3f> colors;
	std::vector<Vec3f> normals;
	std::vector<unsigned int> faces;
	int stacks;
	int slices;
	float spaceBetweenVertices;
	Sphere(float radius, Vec3f position, Vec3f velocity) {
		pos = position;
		vel = velocity;
		rad = radius;
		stacks = 10;
		slices = 10;
		for (int i = 0; i < stacks; i++) {
			double longitude = 2 * PI * i / stacks;
			//calculate the vertices and colors
			for (int j = 0; j < slices; j++) {
				double colatitude = PI * j / slices;
				double x = pos[0] + rad * cos(longitude) * sin(colatitude);
				double y = pos[1] + rad * sin(longitude) * sin(colatitude);
				double z = pos[2] + rad * cos(colatitude);
				points.push_back(Vec3f(x, y, z));
				x = (x + 1) / 2;
				y = (y + 1) / 2;
				z = (z + 1) / 2;
				colors.push_back(Vec3f(1, 0, 0));
				normals.push_back(points.back() - pos);
			}
		}
	}
	Sphere() {
		pos = Vec3f(0, 0, 0);
		vel = Vec3f(0, 0, 0);
		rad = 1;
		stacks = 10;
		slices = 10;
		for (int i = 0; i < stacks; i++) {
			double longitude = 2 * PI * i / stacks;
			//calculate the vertices and colors
			for (int j = 0; j < slices; j++) {
				double colatitude = PI * j / slices;
				double x = rad * cos(longitude) * sin(colatitude);
				double y = rad * sin(longitude) * sin(colatitude);
				double z = rad * cos(colatitude);
				points.push_back(Vec3f(x, y, z));
				x = (x + 1) / 2;
				y = (y + 1) / 2;
				z = (z + 1) / 2;
				colors.push_back(Vec3f(x, y, z));
			}
		}
	}

	void updatePosition(double dt) {
		Vec3f dv = vel*dt;
		pos += dv;
		for (int i = 0; i < points.size(); i++) {
			points[i] += vel*dt;
		}
	}
	void render() {
		using namespace Globals;
		glBindBuffer(GL_ARRAY_BUFFER, colors_vbo[0]);
		if (!colors.empty())
			glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(colors[0]), &colors[0][0], GL_DYNAMIC_DRAW);
		else
			glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		glBindBuffer(GL_ARRAY_BUFFER, position_vbo[0]);
		if (!points.empty())
			glBufferData(GL_ARRAY_BUFFER, points.size() * sizeof(points[0]), &points[0][0], GL_DYNAMIC_DRAW);
		else
			glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		glBindBuffer(GL_ARRAY_BUFFER, normals_vbo[0]);
		if (!normals.empty())
			glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(normals[0]), &normals[0][0], GL_DYNAMIC_DRAW);
		else
			glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		// Draw here
		glDrawArrays(GL_POINTS, 0, points.size());
	}
};

class Grid {
public:
	Vec3f min, max;
	int xSpaces, ySpaces, zSpaces;

	std::vector<std::vector<int>> indices;

	Grid() {
		min = Vec3f(0, 0, 0);
		max = Vec3f(1, 1, 1);
		xSpaces = 4;
		ySpaces = 4;
		zSpaces = 4;

		indices.resize(4 * 4 * 4);
	}

	Grid(Vec3f minRange, Vec3f maxRange, int x, int y, int z) {
		min = minRange;
		max = maxRange;
		xSpaces = x;
		ySpaces = y;
		zSpaces = z;

		indices.resize(x * y * z);
	}

	Grid(Vec3f minRange, Vec3f maxRange, Vec3i numSpaces) {
		min = minRange;
		max = maxRange;
		xSpaces = numSpaces[0];
		ySpaces = numSpaces[1];
		zSpaces = numSpaces[2];

		indices.resize(numSpaces[0] * numSpaces[1] * numSpaces[2]);
	}

	~Grid() {}

	void placeObjects(std::vector<Vec3f> objects) {
		int xGrid, yGrid, zGrid;
		Vec3f maxValues = max - min;

		#if USE_OMP
			#pragma omp parallel for
		#endif
		for (int i = 0; i < objects.size(); i++) {
			xGrid = xSpaces * (objects[i][0] - min[0]) / maxValues[0];
			yGrid = ySpaces * (objects[i][1] - min[1]) / maxValues[1];
			zGrid = zSpaces * (objects[i][2] - min[2]) / maxValues[2];

			indices[xGrid + yGrid*xSpaces + zGrid*ySpaces*xSpaces].push_back(i);
		}
	}
};

class BVH {
public:
	Vec3f min, max;
	std::vector<BVH> children;
	std::vector<int> indices;

	BVH() {
		min = Vec3f(0, 0, 0);
		max = Vec3f(0, 0, 0);
	}

	BVH(Vec3f minimum, Vec3f maximum) {
		min = minimum;
		max = maximum;
	}

	BVH(float minX, float minY, float minZ, float maxX, float maxY, float maxZ) {
		min = Vec3f(minX, minY, minZ);
		max = Vec3f(maxX, maxY, maxZ);
	}

	virtual ~BVH() {}

	bool checkCollision(Vec3f pos, float radius) {
		return pos[0] + radius > min[0] &&
				pos[0] - radius < max[0] &&
				pos[1] + radius > min[1] &&
				pos[1] - radius < max[1] &&
				pos[2] + radius > min[2] &&
				pos[2] - radius < max[2];
	}

	void updateBounds(Vec3f pos, float radius) {
		min[0] = std::min(min[0], pos[0] - radius);
		min[1] = std::min(min[1], pos[1] - radius);
		min[2] = std::min(min[2], pos[2] - radius);

		max[0] = std::max(max[0], pos[0] + radius);
		max[1] = std::max(max[1], pos[1] + radius);
		max[2] = std::max(max[2], pos[2] + radius);
	}
	
	void generateBVH(std::vector<Vec3f> objects, int xNum = -1, int yNum = -1, int zNum = -1) {
		children.clear();

		float tempRadius = .15; // consider giving all objects a radius
		Grid grid;
		int numSpaces;
		Vec3i spaceComponents;
		float rangeX, rangeY, rangeZ, rangeTotal;

		// Update the bounds of the BVH before generating
		// Wipe the values first
		max = objects[0];
		min = objects[0];

		for (int i = 0; i < objects.size(); i++) {
			updateBounds(objects[i], tempRadius);
		}

		// Calculate necessary information for grid
		if (xNum < 0 || yNum < 0 || zNum < 0) {
			rangeX = max[0] - min[0];
			rangeY = max[1] - min[1];
			rangeZ = max[2] - min[2];
			rangeTotal = rangeX + rangeY + rangeZ;

			numSpaces = sqrt(objects.size()); // arbitrary proportion; maybe use sqrt?

			spaceComponents[0] = numSpaces * rangeX/rangeTotal;
			spaceComponents[1] = numSpaces * rangeY/rangeTotal;
			spaceComponents[2] = numSpaces * rangeZ/rangeTotal;
			if (DEBUG) {
				std::cout << "numSpaces: " << numSpaces+1 << "; spaceComponents: " << spaceComponents[0] << ", " << spaceComponents[1] << ", " << spaceComponents[2] << std::endl;
				std::cout << "Ranges: " << rangeX << ", " << rangeY << ", " << rangeZ << std::endl;
				std::cout << "Max-Min: " << min[0] << ", " << min[1] << ", " << min[2] << "; " << max[0] << ", " << max[1] << ", " << max[2] << std::endl;
			}

			numSpaces -= spaceComponents[0];
			numSpaces -= spaceComponents[1];
			numSpaces -= spaceComponents[2];
			while (numSpaces > 0) { // make sure all of the spaces have been given out
				spaceComponents[0]++;
				numSpaces--;

				if (numSpaces <= 0)
					break;
				spaceComponents[1]++;
				numSpaces--;

				if (numSpaces <= 0)
					break;
				spaceComponents[2]++;
				numSpaces--;
			}
		}
		else { // values are passed in for the xNum, yNum, and zNum
			spaceComponents[0] = xNum;
			spaceComponents[1] = yNum;
			spaceComponents[2] = zNum;
		}

		// Place all of the objects on the grid
		grid = Grid(min, max, spaceComponents);
		grid.placeObjects(objects);

		std::vector<BVH> fillerBVH;
		int xIndex, yIndex, zIndex = 0;
		int nextDirection = 0;

		// Populate BVHs with the grid objects
		// Would use omp collapse, but MSVS only supports OpemMP 2.x
		#if USE_OMP
			#pragma omp parallel for
		#endif
		for (int k = 0; k < spaceComponents[2]; k++) {
			#if USE_OMP
				#pragma omp parallel for
			#endif
			for (int j = 0; j < spaceComponents[1]; j++) {
				#if USE_OMP
					#pragma omp parallel for
				#endif
				for (int i = 0; i < spaceComponents[0]; i++) {
					BVH newBVH = BVH();
					newBVH.indices = grid.indices[i + j*spaceComponents[0] + k*spaceComponents[0] * spaceComponents[1]];
					fillerBVH.push_back(newBVH);
				}
			}
		}
		fillerBVH.swap(children);

		// Unfinished; need to create actual BVH
	}
};


//The Mesh that will use spring systems to simulate a hanging cloth (and hopefully other stuff)
class deformableMesh {
public:
	//Really Dynamic Variables - updated every frame
	//location data of a point
	std::vector<Vec3f> points;
	//Used for Midpoint Integration
	std::vector<Vec3f> halfStepFuturePoints;
	//velocity information associated with a point
	std::vector<Vec3f> velocities;
	std::vector<Vec3f> halfStepVelocities;
	std::vector<Vec3f> nextStepVelocities;
	//Vector normal to a given point
	std::vector<Vec3f> normals;
	//Less Dynamic Variables - Won't be changed often, but modifiable
	//The points with the indices in this vector won't be altered by the physics simulation
	std::vector<int> fixedPoint;
	//tracks the index of each neighboring point of each point.
	//e.g. edgeNeighbors[i][k] will be the kth neighbor of point i.
	//edgeNeighbors should be listed counter-clockwise or clockwise to ensure surface normals are calculated correctly
	std::vector<std::vector<int>> edgeNeighbors;
	std::vector<std::vector<int>> skipNeighbors;
	std::vector<unsigned int> faces;
	//The defaultSpring values.
	Spring defaultHorizSpring;
	Spring defaultVertSpring;
	//this one may not get used, depending on how we implement the spring system
	Spring defaultAngleSpring;
	Spring defaultSkipSpring;
	//Spring values between vector i and its neighbor k.
	//springs[i][k] accesses that data. NEEDS TO CORRESPOND TO VALUES IN edgeNeighbors
	std::vector<std::vector<Spring *>> perPointEdgeSpringList;
	std::vector<std::vector<Spring>> perPointSkipSpringList;
	std::vector<std::vector<Spring>> horizSpringList;
	std::vector<std::vector<Spring>> vertSpringList;
	std::vector<std::vector<Spring>> skipSpringList;
	//Mass of each point. could also be a vector, if we want to vary mass distribution
	float pointMass = .001;
	float dragCoefficient = .9;
	
	//Static Variables - Set and forget
	std::vector<Vec2f> texCoords;
	std::vector<Vec3f> colors;
	//number of points across the cloth and down the cloth. Used to calculate texCoords
	int height;
	int width;
	Vec3f gravity = Vec3f(0, 0, 0);

	//makes each point in the vector 'fixed', meaning they won't be movable during the simulation.
	void fixPoints(std::vector<int> pointIndex) {
		for (int i = 0; i < pointIndex.size(); i++) {
			int index = pointIndex[i];
			fixedPoint[index] = 1;
		}
	}
	//sets the mass of the points in the cloth. Used in the spring simulation.
	void setClothMass(float m) {
		pointMass = m/(height*width);
	}

	void updatePosition(int pointIdx, float dt) {
		if (fixedPoint[pointIdx] == 0) {
			points[pointIdx] += halfStepVelocities[pointIdx] * dt;
			//if (pointIdx == 0)printf("%i, %f\n", pointIdx, velocities[pointIdx]);
			//points[pointIdx] += velocities[pointIdx] * dt;
		}
	}//end updatePosition


	/*
	Takes the forces of all the springs associated with a given point and updates the velocity of the point
	in relation to these forces
	*/

	void bufferHalfPosition(int pointIdx, float halfDt) {
		if (fixedPoint[pointIdx] == 0)
			halfStepFuturePoints[pointIdx] = points[pointIdx] + velocities[pointIdx] * halfDt;
	}

	void bufferHalfStepVelocity(int pointIdx, float halfDt) {
		//calculate the Spring forces on all of this point's edge springs
		for (int i = 0; i < edgeNeighbors[pointIdx].size(); i++) {
			if (edgeNeighbors[pointIdx][i] != -1) {
				//if it's been sprung, then we know the neighbor has already calculated the spring's effect on this points velocity, and we can skip the calculations.
				if (perPointEdgeSpringList[pointIdx][i]->sprungThisFrame == 1) {
					perPointEdgeSpringList[pointIdx][i]->sprungThisFrame = 0;
				}
				else {
					perPointEdgeSpringList[pointIdx][i]->sprungThisFrame = 1;
					int neighborIdx = edgeNeighbors[pointIdx][i];
					Vec3f vecBtwnPts = points[neighborIdx] - points[pointIdx];
					float len = vecBtwnPts.len();
					float fs = -perPointEdgeSpringList[pointIdx][i]->springConstant * (perPointEdgeSpringList[pointIdx][i]->restLength - len);
					float fd = -perPointEdgeSpringList[pointIdx][i]->dampeningFactor * (vecBtwnPts.dot(velocities[pointIdx]) - vecBtwnPts.dot(velocities[neighborIdx]));
					vecBtwnPts.normalize();
					halfStepVelocities[pointIdx] += (vecBtwnPts * (fs + fd)) * halfDt;
					if (fixedPoint[neighborIdx] == 0) {
						halfStepVelocities[neighborIdx] += -1 * (vecBtwnPts * (fs + fd)) * halfDt;
					}
					else {
						perPointEdgeSpringList[pointIdx][i]->sprungThisFrame = 0;
					}
				}
			}
		}
		//calculate the Spring forces on this points 'child' skip springs and itself. Treats the angle springs like a tree
		for (int i = 0; i < skipNeighbors[pointIdx].size(); i++) {
			int neighborIdx = skipNeighbors[pointIdx][i];
			if (fixedPoint[neighborIdx]) continue;
			Vec3f vecBtwnPts = points[neighborIdx] - points[pointIdx];
			float len = vecBtwnPts.len();
			float fs = -perPointSkipSpringList[pointIdx][i].springConstant * (perPointSkipSpringList[pointIdx][i].restLength - len);
			vecBtwnPts.normalize();
			float v1 = vecBtwnPts.dot(velocities[pointIdx]);
			float v2 = vecBtwnPts.dot(velocities[neighborIdx]);
			float fd = -perPointSkipSpringList[pointIdx][i].dampeningFactor * (v1 - v2);
			halfStepVelocities[pointIdx] += (vecBtwnPts * (fs + fd)) * halfDt;
			if (!fixedPoint[neighborIdx]) {
				nextStepVelocities[neighborIdx] += -1 * (vecBtwnPts * (fs + fd)) * halfDt;
			}
		}
		//Drag Calculation
		int neighborIdx1 = -1;
		int neighborIdx2 = -1;
		for (int i = 0; i < edgeNeighbors[pointIdx].size() && (neighborIdx1 == -1 || neighborIdx2 == -1); i++) {
			if (edgeNeighbors[pointIdx][i] != -1 && edgeNeighbors[pointIdx][(i + 1) % edgeNeighbors[pointIdx].size()] != -1) {
				neighborIdx1 = i;
				neighborIdx2 = (i + 1) % edgeNeighbors[pointIdx].size();
			}
		}
		if (neighborIdx1 == -1 || neighborIdx2 == -1) {
			Vec3f fGravity = gravity * pointMass;
			halfStepVelocities[pointIdx] += (fGravity) * halfDt;
			return;
		}
		Vec3f velocity = velocities[pointIdx];
		int neighbor1 = edgeNeighbors[pointIdx][neighborIdx1];
		int neighbor2 = edgeNeighbors[pointIdx][neighborIdx2];
		double area = (points[neighbor1] - points[pointIdx]).cross(points[neighbor2] - points[pointIdx]).len()/2;
		double fDragScalar = -dragCoefficient * velocity.dot(normals[pointIdx]) * velocity.dot(normals[pointIdx]) * area;
		velocity.normalize();
		Vec3f fDrag = fDragScalar * velocity;

		//Gravity Calculations
		Vec3f fGravity = gravity * pointMass;

		halfStepVelocities[pointIdx] += (fGravity + fDrag) * halfDt;
	}//end bufferHalfStepVelocity




	void bufferVelocity(int pointIdx, float dt) {

		//calculate the Spring forces on all of this point's edge springs
		for (int i = 0; i < edgeNeighbors[pointIdx].size(); i++) {
			if (edgeNeighbors[pointIdx][i] != -1) {
				//if it's been sprung, then we know the neighbor has already calculated the spring's effect on this points velocity, and we can skip the calculations.
				if (perPointEdgeSpringList[pointIdx][i]->sprungThisFrame == 1) {
					perPointEdgeSpringList[pointIdx][i]->sprungThisFrame = 0;
				}
				else {
					perPointEdgeSpringList[pointIdx][i]->sprungThisFrame = 1;
					int neighborIdx = edgeNeighbors[pointIdx][i];
					Vec3f vecBtwnPts = points[neighborIdx] - points[pointIdx];
					float len = vecBtwnPts.len();
					float fs = -perPointEdgeSpringList[pointIdx][i]->springConstant * (perPointEdgeSpringList[pointIdx][i]->restLength - len);
					float fd = -perPointEdgeSpringList[pointIdx][i]->dampeningFactor * (vecBtwnPts.dot(halfStepVelocities[pointIdx]) - vecBtwnPts.dot(halfStepVelocities[neighborIdx]));
					vecBtwnPts.normalize();
					nextStepVelocities[pointIdx] += (vecBtwnPts * (fs + fd)) * dt;
					if (fixedPoint[neighborIdx] == 0) {
						nextStepVelocities[neighborIdx] += -1 * (vecBtwnPts * (fs + fd)) * dt;
					}
					else {
						perPointEdgeSpringList[pointIdx][i]->sprungThisFrame = 0;
					}
					//Check if the force of this edge spring is greater than the tear force. If it is, we can remove that spring from the list associated with the vector, effectively deleting it.
					//We then need to delete it from the corresponding list for the neighbor it just removed.

					if (len > perPointEdgeSpringList[pointIdx][i]->tearLength) {
						perPointEdgeSpringList[pointIdx][i];
						edgeNeighbors[pointIdx][i] = -1;
						int pointIdxInNeighbor = 0;
						for (int k = 0; k < edgeNeighbors[neighborIdx].size(); k++) {
							if (edgeNeighbors[neighborIdx][k] == pointIdx) {
								pointIdxInNeighbor = k;
							}
							perPointEdgeSpringList[neighborIdx][pointIdxInNeighbor];
							edgeNeighbors[neighborIdx][pointIdxInNeighbor] = -1;
							for (int k = 0; k < faces.size(); k += 3) {
								bool pt1OnFace = false;
								bool pt2OnFace = false;
								for (int j = 0; j < 3; j++) {
									if (faces[k + j] == pointIdx)
										pt1OnFace = true;
									if (faces[k + j] == neighborIdx)
										pt2OnFace = true;
								}
								if (pt1OnFace && pt2OnFace) {
									faces.erase(faces.begin() + k);
									faces.erase(faces.begin() + k);
									faces.erase(faces.begin() + k);
									k -= 3;
								}
							}
						}
					}
				}
			}
		}
		//calculate the Spring forces on this points 'child' skip springs and itself. Treats the angle springs like a tree
		for (int i = 0; i < skipNeighbors[pointIdx].size(); i++) {
			int neighborIdx = skipNeighbors[pointIdx][i];
			if (fixedPoint[neighborIdx]) continue;
			Vec3f vecBtwnPts = points[neighborIdx] - points[pointIdx];
			float len = vecBtwnPts.len();
			float fs = -perPointSkipSpringList[pointIdx][i].springConstant * (perPointSkipSpringList[pointIdx][i].restLength - len);
			vecBtwnPts.normalize();
			float v1 = vecBtwnPts.dot(halfStepVelocities[pointIdx]);
			float v2 = vecBtwnPts.dot(halfStepVelocities[neighborIdx]);
			float fd = -perPointSkipSpringList[pointIdx][i].dampeningFactor * (v1 - v2);
			nextStepVelocities[pointIdx] += (vecBtwnPts * (fs + fd)) * dt;
			if (!fixedPoint[neighborIdx]) {
				nextStepVelocities[neighborIdx] += -1 * (vecBtwnPts * (fs + fd)) * dt;
			}

			//Check if the force of this skip spring is greater than the tear force. If it is, we can remove that spring from the list associated with the vector, effectively deleting it.
			
			if (len > perPointSkipSpringList[pointIdx][i].tearLength) {
				perPointSkipSpringList[pointIdx][i].springConstant = 0;
				perPointSkipSpringList[pointIdx].erase(perPointSkipSpringList[pointIdx].begin() + i);
				skipNeighbors[pointIdx].erase(skipNeighbors[pointIdx].begin() + i);
			}
		}
		//Drag Calculation
		int neighborIdx1 = -1;
		int neighborIdx2 = -1;
		for (int i = 0; i < edgeNeighbors[pointIdx].size() && (neighborIdx1 == -1 || neighborIdx2 == -1); i++) {
			if (edgeNeighbors[pointIdx][i] != -1 && edgeNeighbors[pointIdx][(i + 1) % edgeNeighbors[pointIdx].size()] != -1) {
				neighborIdx1 = i;
				neighborIdx2 = (i + 1) % edgeNeighbors[pointIdx].size();
			}
		}
		if (neighborIdx1 == -1 || neighborIdx2 == -1) {
			Vec3f fGravity = gravity * pointMass;
			halfStepVelocities[pointIdx] += (fGravity)* dt;
			return;
		}
		Vec3f velocity = halfStepVelocities[pointIdx];
		int neighbor1 = edgeNeighbors[pointIdx][neighborIdx1];
		int neighbor2 = edgeNeighbors[pointIdx][neighborIdx2];
		double area = (points[neighbor1] - points[pointIdx]).cross(points[neighbor2] - points[pointIdx]).len()/2;
		double fDragScalar = -dragCoefficient * velocity.dot(normals[pointIdx]) * velocity.dot(normals[pointIdx]) * area;
		velocity.normalize();
		Vec3f fDrag = fDragScalar * velocity;
		//Gravity Calculations
		Vec3f fGravity = gravity * pointMass;

		nextStepVelocities[pointIdx] += (fGravity + fDrag) * dt;
	}//end bufferVelociyt

	void updateNormal(int pointIdx) {
		//Drag Calculation
		int neighborIdx1 = -1;
		int neighborIdx2 = -1;
		Vec3f normalAcc = Vec3f(0, 0, 0);
		int numNeighbors = edgeNeighbors[pointIdx].size();
		for (int i = 0; i < numNeighbors; i++) {
			if (edgeNeighbors[pointIdx][i] != -1 && edgeNeighbors[pointIdx][(i + 1) % numNeighbors] != -1) {
				Vec3f e1 = points[edgeNeighbors[pointIdx][i]] - points[pointIdx];
				Vec3f e2 = points[edgeNeighbors[pointIdx][(i + 1) % numNeighbors]] - points[pointIdx];
				Vec3f normTemp = e1.cross(e2);
				normTemp.normalize();
				normalAcc += normTemp;
			}
		}
		normals[pointIdx] = normalAcc;
		normals[pointIdx].normalize();

	}//end updateNormal



	/*void checkCollision(Sphere s) {
		for (int i = 0; i < points.size(); i++) {
			//A collision has occurred
			Vec3f v = s.pos - points[i];
			float len = v.len();
			if (len < (s.r + .009)) {
				v.normalize();
				v = v * -1;
				Vec3f bounce = v * (v.dot(velocities[i]));
				velocities[i] += -.02*bounce;
				points[i] += v * (.01 + s.r - len);
			}
			else if (points[i][1] < .009) {
				v = Vec3f(0, 1, 0);
				Vec3f bounce = v * v.dot(velocities[i]);
				velocities[i] += -.02 * bounce;
				points[i][1] = .01;
			}
		}
	}//End Check Collision*/

	deformableMesh() {
		return;
	}



};//End DeformableMesh
class Cloth : public deformableMesh {
public:
	Cloth() : deformableMesh() {}


	Cloth(Vec3f startPt, Vec3f startHorizDir, Vec3f startVertDir, Vec3f cornerColors[4], int h, int w, int lengthbetweenPts, double clothWeight, Spring s) : deformableMesh() {
		if (DEBUG) printf("In Constructors of Cloth\n");
		height = h;
		width = w;
		pointMass = clothWeight / (h*w);
		defaultHorizSpring = s;
		defaultVertSpring = s;
		defaultAngleSpring.restLength = sqrt((s.restLength * s.restLength) + (s.restLength * s.restLength));
		defaultAngleSpring.springConstant = s.springConstant * .2;
		defaultAngleSpring.dampeningFactor = s.dampeningFactor;
		defaultSkipSpring.restLength = s.restLength * 2;
		defaultSkipSpring.dampeningFactor = s.dampeningFactor;
		defaultSkipSpring.springConstant = s.springConstant * .2;
		for (int i = 0; i < height; i++) {
			std::vector<Spring>rowOfSprings;
			std::vector<Spring>columnOfSprings;
			for (int j = 0; j < width; j++) {
				columnOfSprings.push_back(defaultVertSpring);
				rowOfSprings.push_back(defaultHorizSpring);
			}
			vertSpringList.push_back(columnOfSprings);
			horizSpringList.push_back(rowOfSprings);
		}
		for (int i = 0; i < height; i++) {
			for (int j = 0; j < width; j++) {
				std::vector<Spring *> thisPointEdgeSprings;
				std::vector<Spring> thisPointSkipSprings;
				normals.push_back(Vec3f(0, 1, 0));
				velocities.push_back(Vec3f(0, 0, 0));
				points.push_back(startPt + j * startHorizDir + i * startVertDir);
				colors.push_back(
					(cornerColors[0] * (1 - double(j) / double(width)) +
						cornerColors[1] * (double(j) / double(width))) *
						(1 - double(i) / double(height)) +
					(cornerColors[2] * (1 - double(j) / double(width)) +
						cornerColors[3] * (double(j) / double(width))) *
						(double(i) / double(height))
				);
				fixedPoint.push_back(0);
				//ordered W-S-E-N
				std::vector<int> edgeNeighborsArray;
				//ordered W-SW-S-SE-E-NE-N-NW
				std::vector<int> skipNeighborsArray;
				//If there is a guaranteed left edge neighbor
				if (j > 0) {
					edgeNeighborsArray.push_back((i*width) + j - 1);
					thisPointEdgeSprings.push_back(&horizSpringList[i][j - 1]);
				}
				else {
					edgeNeighborsArray.push_back(-1);
					thisPointEdgeSprings.push_back(0);
				}
				//If there is a garuanteed lower edge neighbor
				if (i < height - 1) {
					edgeNeighborsArray.push_back(((i + 1)*width) + j);
					thisPointEdgeSprings.push_back(&vertSpringList[i+1][j]);
				}
				else {
					edgeNeighborsArray.push_back(-1);
					thisPointEdgeSprings.push_back(0);
				}
				//If there is a garuanteed right edge neighbor
				if (j < width - 1) {
					edgeNeighborsArray.push_back((i*width) + j + 1);
					thisPointEdgeSprings.push_back(&horizSpringList[i][j + 1]);
				}
				else {
					edgeNeighborsArray.push_back(-1);
					thisPointEdgeSprings.push_back(0);
				}
				//If there is a garuanteed upper edge neighbor
				if (i > 0) {
					edgeNeighborsArray.push_back(((i - 1)*width) + j);
					thisPointEdgeSprings.push_back(&vertSpringList[i-1][j]);
				}
				else {
					edgeNeighborsArray.push_back(-1);
					thisPointEdgeSprings.push_back(0);
				}
				//If there is a garuanteed lower skip neighbor
				if (i < height - 2) {
					skipNeighborsArray.push_back(((i+2) * width) + j);
					thisPointSkipSprings.push_back(defaultSkipSpring);
				}
				//If there is a garuanteed right skip neighbor
				if (j < width - 2) {
					skipNeighborsArray.push_back((i*width) + j + 2);
					thisPointSkipSprings.push_back(defaultSkipSpring);
				}
				//If there is a SE neighbor
				if ((i < height - 1) && (j < width - 1)) {
					skipNeighborsArray.push_back(((i+1)*width) + j + 1);
					thisPointSkipSprings.push_back(defaultAngleSpring);
				}
				//If there is a NE neighbor
				if ((i > 0) && (j < width - 1)) {
					skipNeighborsArray.push_back(((i-1)*width) + j + 1);
					thisPointSkipSprings.push_back(defaultAngleSpring);
				}
				
				edgeNeighbors.push_back(edgeNeighborsArray);
				perPointEdgeSpringList.push_back(thisPointEdgeSprings);
				skipNeighbors.push_back(skipNeighborsArray);
				perPointSkipSpringList.push_back(thisPointSkipSprings);
			}//end inner for loop
		}//end outer for loop

		 // top edge
		for (int i = 0; i < width; i++) {
			for (int k = 0; k < 4; k++) {
				if (edgeNeighbors[i][k] == -1)
					continue;

				perPointEdgeSpringList[i][k]->springConstant *= 2;
				perPointEdgeSpringList[i][k]->dampeningFactor += (1 - perPointEdgeSpringList[i][k]->dampeningFactor)/2;
			}
		}

		// bottom edge
		for (int i = (width-1)*height; i < width*height; i++) {
			for (int k = 0; k < 4; k++) {
				if (edgeNeighbors[i][k] == -1)
					continue;

				perPointEdgeSpringList[i][k]->springConstant *= 2;
				perPointEdgeSpringList[i][k]->dampeningFactor += (1 - perPointEdgeSpringList[i][k]->dampeningFactor) / 2;
			}
		}

		// left edge
		for (int i = 0; i < width*height; i += width) {
			for (int k = 0; k < 4; k++) {
				if (edgeNeighbors[i][k] == -1)
					continue;

				perPointEdgeSpringList[i][k]->springConstant *= 2;
				perPointEdgeSpringList[i][k]->dampeningFactor += (1 - perPointEdgeSpringList[i][k]->dampeningFactor) / 2;
			}
		}

		// right edge
		for (int i = (width - 1); i < width*height; i += width) {
			for (int k = 0; k < 4; k++) {
				if (edgeNeighbors[i][k] == -1)
					continue;

				perPointEdgeSpringList[i][k]->springConstant *= 2;
				perPointEdgeSpringList[i][k]->dampeningFactor += (1 - perPointEdgeSpringList[i][k]->dampeningFactor) / 2;
			}
		}

		halfStepFuturePoints = points;
		nextStepVelocities = velocities;
		halfStepVelocities = velocities;
		for (int i = 0; i < height - 1; i++) {
			for (int j = 0; j < width - 1; j++) {
				faces.push_back(j + i*width);
				faces.push_back(j + (i + 1)*width);
				faces.push_back((j + 1) + (i + 1)*width);
				faces.push_back(j + i*width);
				faces.push_back((j + 1) + (i + 1)*width);
				faces.push_back((j + 1) + i*width);
			}
		}
	}//End Constructor

	//Edge case fixing
	//void 

	//Cloth Rendering
	void render() {
		using namespace Globals;
		//update the color and position buffers/
		glBindBuffer(GL_ARRAY_BUFFER, colors_vbo[0]);
		if (!colors.empty())
			glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(colors[0]), &colors[0][0], GL_DYNAMIC_DRAW);
		else
			glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		glBindBuffer(GL_ARRAY_BUFFER, position_vbo[0]);
		if (!points.empty())
			glBufferData(GL_ARRAY_BUFFER, points.size() * sizeof(points[0]), &points[0][0], GL_DYNAMIC_DRAW);
		else
			glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		glBindBuffer(GL_ARRAY_BUFFER, normals_vbo[0]);
		if (!normals.empty())
			glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(normals[0]), &normals[0][0], GL_DYNAMIC_DRAW);
		else
			glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);


		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, faces_ibo[0]);
		if (!faces.empty())
			glBufferData(GL_ELEMENT_ARRAY_BUFFER, faces.size() * sizeof(faces[0]), &faces[0], GL_DYNAMIC_DRAW);
		else
			glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		// Draw here
		glDrawElements(GL_TRIANGLES, faces.size(), GL_UNSIGNED_INT, (GLvoid*)0);

		//glUniform1i(currShader.uniform("pcolor"), 1);
		//glDrawElements(GL_POINTS, faces.size(), GL_UNSIGNED_INT, (GLvoid*)0);

		//glUniform1i(currShader.uniform("pcolor"), 0);
	}
};

class Box : deformableMesh {
public:
	double length;
	Box() {
		height = 1;
		length = 1;
	}
};

namespace Globals {
	Sphere s;
	BVH bvh;
	Cloth cloth;
}



float dot(Vec3f v1, Vec3f v2) {
	return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}


void updateViewProjection() {
	using namespace Globals;

	// Calculate the orthogonal axes based on the viewing parameters
	Vec3f n = viewDir * (-1.f / viewDir.len());
	Vec3f u = upDir.cross(n);
	u.normalize();
	Vec3f v = n.cross(u);

	// Calculate the translation based on the new axes
	float dx = -(eye.dot(u));
	float dy = -(eye.dot(v));
	float dz = -(eye.dot(n));

	// Fill in the matrix
	view.m[0] = u[0];	view.m[4] = u[1];	view.m[8] = u[2];	view.m[12] = dx;
	view.m[1] = v[0];	view.m[5] = v[1];	view.m[9] = v[2];	view.m[13] = dy;
	view.m[2] = n[0];	view.m[6] = n[1];	view.m[10] = n[2];	view.m[14] = dz;
	view.m[3] = 0;		view.m[7] = 0;		view.m[11] = 0;		view.m[15] = 1;
}

//
//	Callbacks
//
static void error_callback(int error, const char* description){ fprintf(stderr, "Error: %s\n", description); }


// function that is called whenever a mouse or trackpad button press event occurs
static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
		glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
		glfwGetCursorPos(window, &Globals::mouse.prev_x, &Globals::mouse.prev_y);
		Globals::mouse.active = true;
	}
}

// Function to rotate the viewing transform about the y axis
static const Mat4x4 rotateY(float theta) {
	float t = theta*PI/180.f;
	
	Mat4x4 mat;
	mat.m[0] = cos(t);		mat.m[4] = 0.f;		mat.m[8] = sin(t);		mat.m[12] = 0.f;
	mat.m[1] = 0.f;			mat.m[5] = 1.f;		mat.m[9] = 0.f;			mat.m[13] = 0.f;
	mat.m[2] = -sin(t);		mat.m[6] = 0.f;		mat.m[10] = cos(t);		mat.m[14] = 0.f;
	mat.m[3] = 0.f;			mat.m[7] = 0.f;		mat.m[11] = 0.f;		mat.m[15] = 1.f;
	
	return mat;
}

// Function to rotate the viewing transform about the y axis
static const Mat4x4 rotateX(float phi) {
	float t = phi*PI/180.f;
	
	Mat4x4 mat;
	mat.m[0] = 1.f;		mat.m[4] = 0.f;		mat.m[8] = 0.f;			mat.m[12] = 0.f;
	mat.m[1] = 0.f;		mat.m[5] = cos(t);	mat.m[9] = -sin(t);		mat.m[13] = 0.f;
	mat.m[2] = 0.f;		mat.m[6] = sin(t);	mat.m[10] = cos(t);		mat.m[14] = 0.f;
	mat.m[3] = 0.f;		mat.m[7] = 0.f;		mat.m[11] = 0.f;		mat.m[15] = 1.f;
	
	return mat;
}

static void cursor_position_callback(GLFWwindow* window, double xpos, double ypos)
{
	using namespace Globals;
	if (!mouse.active)
		return;

	viewDir = Vec3f(0, 0, -1);
	upDir = Vec3f(0, 1, 0);
	
	if (xpos != mouse.prev_x) {	
		theta -= 0.2*(xpos - mouse.prev_x);
		yRot = rotateY(theta);
		mouse.prev_x = xpos;
	}

	if (ypos != mouse.prev_y) {
		phi -= 0.2*(ypos - mouse.prev_y);
		if (phi > 89)
			phi = 89;
		else if (phi < -89)
			phi = -89;
		xRot = rotateX(phi);
		mouse.prev_y = ypos;
	}
	
	viewDir = xRot*viewDir;
	viewDir = yRot*viewDir;

	upDir = xRot*upDir;
	upDir = yRot*upDir;

	rightDir = upDir.cross(viewDir);
}


static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods){
	using namespace Globals;

	// Close on escape or Q
	Vec3f rev;
	if (action == GLFW_PRESS) {
		switch (key) {
			// Close on escape
		case GLFW_KEY_ESCAPE: glfwSetWindowShouldClose(window, GL_TRUE); break;
			// Pause the simulation
		case GLFW_KEY_SPACE: if (pause) pause = false;
							 else pause = true;
							 break;



			//Moves the sphere paddle around
		case GLFW_KEY_KP_6: paddle_up = true; break;
		case GLFW_KEY_KP_4: paddle_down = true; break;
		case GLFW_KEY_KP_5: paddle_stop = true; break;

							 // Movement keys trigger booleans to be processed during the graphics loop
							 // Forward movement
		case GLFW_KEY_UP: key_up = true; break;
		case GLFW_KEY_W: key_w = true; break;

			// Backward movement
		case GLFW_KEY_DOWN: key_down = true; break;
		case GLFW_KEY_S: key_s = true; break;

			// Right strafing movement
		case GLFW_KEY_RIGHT: key_right = true; break;
		case GLFW_KEY_D: key_d = true; break;

			// Left strafing movement
		case GLFW_KEY_LEFT: key_left = true; break;
		case GLFW_KEY_A: key_a = true; break;

			// Upward movement
		case GLFW_KEY_RIGHT_SHIFT: key_rshift = true; break;
		case GLFW_KEY_E: key_e = true; break;

			// Downward movement
		case GLFW_KEY_KP_0: key_0 = true; break;
		case GLFW_KEY_Q: key_q = true; break;

			// Speed up
		case GLFW_KEY_RIGHT_CONTROL: key_rcontrol = true; break;
		case GLFW_KEY_LEFT_SHIFT: key_lshift = true; break;

			// Release mouse
		case GLFW_KEY_KP_ENTER:
			glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
			mouse.active = false;
			break;

			// Increase/decrease game speed
		case GLFW_KEY_MINUS: subTimeMultiplier = true; break;
		case GLFW_KEY_EQUAL: addTimeMultiplier = true; break;
		case GLFW_KEY_BACKSPACE: timeMultiplier = 1.0; break;
		}
	}
	else if ( action == GLFW_RELEASE ) {
		switch ( key ) {

			case GLFW_KEY_KP_6: paddle_up = false; break;
			case GLFW_KEY_KP_4: paddle_down = false; break;
			case GLFW_KEY_KP_5: paddle_stop = false; break;
			// Movement keys trigger booleans to be processed during the graphics loop
			// Forward movement
			case GLFW_KEY_UP: key_up = false; break;
			case GLFW_KEY_W: key_w = false; break;

			// Backward movement
			case GLFW_KEY_DOWN: key_down = false; break;
			case GLFW_KEY_S: key_s = false; break;

			// Right strafing movement
			case GLFW_KEY_RIGHT: key_right = false; break;
			case GLFW_KEY_D: key_d = false; break;

			// Left strafing movement
			case GLFW_KEY_LEFT: key_left = false; break;
			case GLFW_KEY_A: key_a = false; break;

			// Upward movement
			case GLFW_KEY_RIGHT_SHIFT: key_rshift = false; break;
			case GLFW_KEY_E: key_e = false; break;

			// Downward movement
			case GLFW_KEY_KP_0: key_0 = false; break;
			case GLFW_KEY_Q: key_q = false; break;

			// Speed up
			case GLFW_KEY_RIGHT_CONTROL: key_rcontrol = false; break;
			case GLFW_KEY_LEFT_SHIFT: key_lshift = false; break;

			// Increase/decrease game speed
			case GLFW_KEY_MINUS: subTimeMultiplier = false; break;
			case GLFW_KEY_EQUAL: addTimeMultiplier = false;
		}
	}
}

void updatePerspectiveProjection() {
	using namespace Globals;

	for (int i = 0; i < 15; i++) {
		projection.m[i] = 0;
	}
	left = aspect * bottom;
	right = aspect * top;
	//diagonal values done first
	projection.m[0] = 2 * near / (right - left);
	projection.m[5] = 2 * near / (top - bottom);
	projection.m[10] = -(near + far) / (far - near);
	projection.m[15] = 0;
	//other values are then calculated.
	projection.m[8] = (right + left) / (right - left);
	projection.m[9] = (top + bottom) / (top - bottom);
	projection.m[14] = -2 * far*near / (far - near);
	projection.m[11] = -1;
}

static void framebuffer_size_callback(GLFWwindow* window, int width, int height){
	Globals::win_width = float(width);
	Globals::win_height = float(height);
    Globals::aspect = Globals::win_width/Globals::win_height;
	
    glViewport(0,0,width,height);

	// ToDo: update the perspective matrix according to the new window size
	updatePerspectiveProjection();
}





// Function to set up geometry
void init_scene();

//
//	Main
//
int main(int argc, char *argv[]){

	// Set up window
	GLFWwindow* window;
	glfwSetErrorCallback(&error_callback);

	// Initialize the window
	if( !glfwInit() ){ return EXIT_FAILURE; }

	// Ask for OpenGL 3.2
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

	// Create the glfw window
	Globals::win_width = WIN_WIDTH;
	Globals::win_height = WIN_HEIGHT;
	window = glfwCreateWindow(int(Globals::win_width), int(Globals::win_height), "HW1", NULL, NULL);
	if( !window ){ glfwTerminate(); return EXIT_FAILURE; }
	Globals::activeWindow = window;
	// Bind callbacks to the window
	glfwSetKeyCallback(window, &key_callback);
	glfwSetFramebufferSizeCallback(window, &framebuffer_size_callback);

	// Make current
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	// Initialize glew AFTER the context creation and before loading the shader.
	// Note we need to use experimental because we're using a modern version of opengl.
	#ifdef USE_GLEW
		glewExperimental = GL_TRUE;
		glewInit();
	#endif

	// Initialize the shader (which uses glew, so we need to init that first).
	// MY_SRC_DIR is a define that was set in CMakeLists.txt which gives
	// the full path to this project's src/ directory.
	std::stringstream ss; ss << MY_SRC_DIR << "shader.";
	Globals::currShader.init_from_files( ss.str()+"vert", ss.str()+"frag" );

	// Initialize the scene
	// IMPORTANT: Only call after gl context has been created
	init_scene();
	framebuffer_size_callback(window, int(Globals::win_width), int(Globals::win_height));

	// Enable the shader, this allows us to set uniforms and attributes
	Globals::currShader.enable();
	glBindVertexArray(Globals::particles_vao);

	// Initialize OpenGL
	glEnable(GL_DEPTH_TEST);
	glClearColor(0.0, 0.0, 0.0, 1.f);

	updatePerspectiveProjection();
	updateViewProjection();

	double timePassed = 0;
	double dt = 0;
	int frames = 0;
	double counter = 0;
	int seconds = 0;

	using namespace Globals;
	if (DEBUG) {
		for (int i = 0; i < cloth.points.size(); i++) {
			printf("Point %i edgeNeighbors: ", i);
			for (int j = 0; j < cloth.edgeNeighbors[i].size(); j++) {
				printf("%i, ", cloth.edgeNeighbors[i][j]);
			}
			printf("\n");
		}
	}

	int framesPassed = 0;
	BVH currBVH;
	int currObjectIndex;
	std::vector<BVH> bvhList;
	std::vector<int> collisionList;
	std::vector<int> collisionGroup;
	for (int i = 0; i < cloth.points.size(); i++) {
		collisionGroup.push_back(i);
	}

	// Game loop
	while (!glfwWindowShouldClose(window)) {

		framesPassed++;
		// Clear screen
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		prevTime = currTime;
		currTime = glfwGetTimerValue();
		timePassed = (double)(currTime - prevTime) / glfwGetTimerFrequency();
		dt = timePassed*timeMultiplier;
		// PHYSICS UPDATE
		if (!pause) {
		int updateFrames = 20;
			double sub_dt = dt / updateFrames;
			for (int i = 0; i < updateFrames; i++) {
				// Calculate velocities
				#if USE_OMP
					#pragma omp parallel for
				#endif
				for (int j = 0; j < cloth.points.size(); j++) {
					cloth.bufferHalfPosition(j, sub_dt / 2);
				}
				//then calculate the velocity at the half step
				#if USE_OMP
					#pragma omp parallel for
				#endif
				for (int j = 0; j < cloth.points.size(); j++) {
					cloth.bufferHalfStepVelocity(j, sub_dt / 2);
				}
				//Then we update the position based on that half step
				#if USE_OMP
					#pragma omp parallel for
				#endif
				for (int j = 0; j < cloth.points.size(); j++) {
					cloth.updatePosition(j, sub_dt);
				}
				//And finally, we update the velocity at the newly calculated Position
				#if USE_OMP
					#pragma omp parallel for
				#endif
				for (int j = 0; j < cloth.points.size(); j++) {
					cloth.bufferVelocity(j, sub_dt);
				}

				memcpy(&cloth.velocities[0], &cloth.nextStepVelocities[0], sizeof(Vec3f) * cloth.nextStepVelocities.size());
				memcpy(&cloth.halfStepVelocities[0], &cloth.velocities[0], sizeof(Vec3f) * cloth.velocities.size());

				// Check collisions ---- Hardcoded for sphere s, maybe should pull this out as a function
				if (USE_BVH) {
					bvhList.push_back(bvh);
					while (bvhList.size() > 0) {
						currBVH = bvhList.back();
						bvhList.pop_back();
						if (currBVH.checkCollision(s.pos, s.rad)) {
							if (currBVH.children.size() == 0) {
								for (int i = 0; i < currBVH.indices.size(); i++) {
									collisionList.push_back(currBVH.indices[i]);
								}
							}
							else {
								for (int i = 0; i < currBVH.children.size(); i++) {
									bvhList.push_back(currBVH.children[i]);
								}
							}
						}
					}
					while (collisionList.size() > 0) {
						currObjectIndex = collisionList.back();
						collisionList.pop_back();
						Vec3f v = s.pos - cloth.points[currObjectIndex];
						float len = v.len();
						if (len < (s.rad + .009)) {
							v.normalize();
							v = v * -1;
							Vec3f bounce = v * (v.dot(cloth.velocities[currObjectIndex]));
							cloth.velocities[currObjectIndex] += -.2*bounce;
							cloth.nextStepVelocities[currObjectIndex] = cloth.velocities[currObjectIndex];
							cloth.halfStepVelocities[currObjectIndex] = cloth.velocities[currObjectIndex];
							cloth.points[currObjectIndex] += v * (.01 + s.rad - len);
						}
						if (FLOOR_EXISTS && cloth.points[currObjectIndex][1] < .009) {
							v = Vec3f(0, 1, 0);
							Vec3f bounce = v * v.dot(cloth.velocities[currObjectIndex]);
							cloth.velocities[currObjectIndex] += -.2 * bounce;
							cloth.nextStepVelocities[currObjectIndex] = cloth.velocities[currObjectIndex];
							cloth.halfStepVelocities[currObjectIndex] = cloth.velocities[currObjectIndex];
							cloth.points[currObjectIndex][1] = .01;

						}
					}
				}
				// if USE_BVH is false
				else { 
					#if USE_OMP
						#pragma omp parallel for
					#endif
					for (int i = 0; i < cloth.points.size(); i++) {
						Vec3f v = cloth.points[i] - s.pos;
						float len = v.len();
						if (len < (s.rad + .009)) {
							v.normalize();
							Vec3f bounce = v * v.dot(cloth.velocities[i]);
							cloth.velocities[i] += -.2*bounce;
							cloth.points[i] += v * (.01 + s.rad - len);
						}
						if (FLOOR_EXISTS && cloth.points[i][1] < .009) {
							v = Vec3f(0, 1, 0);
							Vec3f bounce = v * v.dot(cloth.velocities[i]);
							cloth.velocities[i] = .9 * cloth.velocities[i];
							cloth.velocities[i] += .2 * bounce;
							cloth.points[i][1] = .01;

						}
					}
				}

				// Generate the new BVH
				if (USE_BVH) bvh.generateBVH(cloth.points);

				// Calculate normals occasionally
				if (i % (updateFrames/2) == 0) {
					#pragma omp parallel for
					for (int j = 0; j < cloth.points.size(); j++) {
						cloth.updateNormal(j);
					}
				}
			}
		}
		// INPUT PROCESSING
		if (addTimeMultiplier)
			timeMultiplier = std::min(1.0, timeMultiplier + .01);
		if (subTimeMultiplier)
			timeMultiplier = std::max(0.01, timeMultiplier - .01);

		if (key_rcontrol || key_lshift)
			movementSpeed += .01;
		else
			movementSpeed = .1;

		if (key_w) // Move the camera forward
			eye += viewDir*movementSpeed;
		if (key_s) // Move the camera backward
			eye += viewDir*(-movementSpeed);
		if (key_a) // Move the camera leftward
			eye += rightDir*movementSpeed;
		if (key_d) // Move the camera rightward
			eye += rightDir*(-movementSpeed);
		if (key_e) // Move the camera upward
			eye += upDir*movementSpeed;
		if (key_q) // Move the camera downward
			eye += upDir*(-movementSpeed);
		
		
		if (paddle_up) // 
			s.vel += Vec3f(0, .5, 0)*-movementSpeed;
		if (paddle_down) // 
			s.vel += Vec3f(0, -.5, 0)*-movementSpeed;
		if (paddle_stop) // 
			s.vel = Vec3f(0, 0, 0);
		s.vel[1] = std::min((float) 2, std::max((float)-2, s.vel[1]));
		// Send updated info to the GPU
		updateViewProjection();

		// FRAME RATE DISPLAY
		frames++;
		counter += timePassed;
		if (counter >= 1.0) {
			std::cout << "S" << seconds << " FPS: " << frames << std::endl;
			frames = 0;
			counter -= 1.0;
			seconds++;
		}
		s.updatePosition(dt);

		// RENDERING
		//glUniformMatrix4fv( shader.uniform("model"), 1, GL_FALSE, model.m  ); // model transformation
		glUniformMatrix4fv(Globals::currShader.uniform("view"), 1, GL_FALSE, view.m); // viewing transformation
		glUniformMatrix4fv(Globals::currShader.uniform("projection"), 1, GL_FALSE, projection.m); // projection matrix
		glUniform3f(Globals::currShader.uniform("viewdir"), viewDir[0], viewDir[1], viewDir[2]);
		glUniform3f(Globals::currShader.uniform("light"), lightDir[0], lightDir[1], lightDir[2]);

		cloth.render();
		s.render();
		// Finalize
		glfwSwapBuffers(window);
		glfwPollEvents();

		
	} // end game loop
	// Unbind
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	// Disable the shader, we're done using it
	Globals::currShader.disable();

	return EXIT_SUCCESS;
}


void init_scene(){

	using namespace Globals;
	int width = 30;
	float len = .05;
	Vec3f multColors[4] = {
		Vec3f(1, 0, 0),
		Vec3f(0, 1, 0),
		Vec3f(0, 0, 1),
		Vec3f(1, 1, 0)
	};
	Vec3f zVector = Vec3f(0, 0, 1);
	Vec3f xVector = Vec3f(1, 0, 0);
	Vec3f yVector = Vec3f(0, -1, 0);
	zVector.normalize();
	xVector.normalize();
	yVector.normalize();
	zVector = zVector * len;
	xVector = xVector * len;
	yVector = xVector * len;
	Spring spr;
	s = Sphere(.2, Vec3f(0, 0, 0), Vec3f(0, 0, 0));
	spr.dampeningFactor = .8;
	spr.restLength = len;
	spr.springConstant = 120;
	spr.tearLength = 3*len;
	cloth = Cloth(Vec3f(-1, 5, -1), xVector, zVector, multColors, width, width, len, 10, spr);
	cloth.setClothMass(.1);
	cloth.gravity = Vec3f(0, -9.8, 0);

	
	std::vector<int> fPts;
	//fPts.push_back(0);
	//fPts.push_back(width * width - 1);
	//fPts.push_back(width * (width - 1));
	for (int i = 0; i < width; i++) {
		fPts.push_back(i);
		fPts.push_back((i * width));
	}
	for (int i = 0; i < width ; i++) {
		fPts.push_back((i * width) + width - 1);
		fPts.push_back((width * (width - 1)) + i);
	}
	cloth.fixPoints(fPts);

	glPointSize(5);
	

	if (USE_BVH) {
		bvh = BVH();
		bvh.generateBVH(cloth.points);
	}

	// Define the keyboard callback function
	glfwSetKeyCallback(activeWindow, key_callback);
	// Define the mouse button callback function
	glfwSetMouseButtonCallback(activeWindow, mouse_button_callback);
	// Define the mouse motion callback function
	glfwSetCursorPosCallback(activeWindow, cursor_position_callback);
	viewDir = Vec3f(0, 0, -1);
	upDir = Vec3f(0, 1, 0);
	rightDir = Vec3f(-1, 0, 0);
	eye = Vec3f(0, 2, 5);

	glGenVertexArrays(1, &particles_vao);
	glBindVertexArray(particles_vao);

	glGenBuffers(1, position_vbo);
	// Create the buffer for colors
	glGenBuffers(1, colors_vbo);
	//create buffer for normals
	glGenBuffers(1, normals_vbo);
	//create buffer for texture
	glGenTextures(1, &tex);

	//GLint glnormal = glGetAttribLocation(currShader.program_id, "normal");
	//particle position
	glEnableVertexAttribArray(currShader.attribute("in_position"));
	glBindBuffer(GL_ARRAY_BUFFER, position_vbo[0]);
	glVertexAttribPointer(currShader.attribute("in_position"), 3, GL_FLOAT, GL_FALSE, 12, 0);
	//particle color
	glEnableVertexAttribArray(currShader.attribute("in_color"));
	glBindBuffer(GL_ARRAY_BUFFER, colors_vbo[0]);
	glVertexAttribPointer(currShader.attribute("in_color"), 3, GL_FLOAT, GL_FALSE, 12, 0);
	//particle normal
	glEnableVertexAttribArray(currShader.attribute("in_normal"));
	glBindBuffer(GL_ARRAY_BUFFER, normals_vbo[0]);
	glVertexAttribPointer(currShader.attribute("in_normal"), 3, GL_FLOAT, GL_TRUE, 12, 0);
	//texture
	glBindTexture(GL_TEXTURE_2D, tex);

	// Create buffer for indices
	glGenBuffers(1, faces_ibo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, faces_ibo[0]);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, cloth.faces.size() * sizeof(cloth.faces[0]), &cloth.faces[0], GL_STATIC_DRAW);

	glUniform3f(currShader.uniform("light"), lightDir[0], lightDir[1], lightDir[2]);
	glUniform3f(currShader.uniform("viewdir"), viewDir[0], viewDir[1], viewDir[2]);
	glUniform1i(currShader.uniform("pcolor"), 0);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	//done setting data for VAO
	glBindVertexArray(0);
	currTime = glfwGetTimerValue();
}
