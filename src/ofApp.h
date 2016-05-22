#pragma once

#include "ofMain.h"
#include "ofxDelaunay.h"
#include <Eigen/Core>
#include <Eigen/Eigen>
#include "ofxGui.h"

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);

		ofxDelaunay triangulation;

		int total_points;

		// init
		float mass;
		ofVec3f gravity;
		vector<ofVec3f> X; //position
		vector<ofVec3f> tmp_X; //predicted position
		vector<ofVec3f> V; //velocity
		vector<ofVec3f> F; // force
		vector<float> W; //inverse particle mass 

		void reset();
		float calcTriangleQuality(ofPoint p0, ofPoint p1, ofPoint p2);
		float calcArea(float a, float b, float c);

		// update
		void stepPhysics(float dt);
			void computeForce(float dt);
			void IntegrateExplicitWithDamping(float dt);
			void updateInternalConstraint(float dt);
			void updateExternalConstraint(float dt);
			void intergrate(float dt);
			void groundCollision();

		// constraint
		void addDistanceConstraint(int a, int b, float k);
		void updateDistanceConstraint(int i);
		struct DistanceConstraint {	int p1, p2;	float rest_length, k; float k_prime; };
		vector<DistanceConstraint> d_constraints;

		// gui
		ofxFloatSlider global_dampening;
		ofxFloatSlider time_step;
		ofxIntSlider solver_iterations;
		ofxToggle bWire;
		ofxToggle bPlay;
		ofxPanel gui;
		bool bHide;

		// interaction
		int state;
		float dist;
		int oldX, oldY;
		float rX, rY;
};
