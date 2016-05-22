#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	ofEnableSmoothing();
	ofBackground(50);
	ofSetFrameRate(60);

	// gui
	gui.setup(); // most of the time you don't need a name
	gui.setName("Parameters");
	gui.add(global_dampening.setup( "damping", 0.99, 0.0, 1.0 ));
	gui.add(solver_iterations.setup( "iteration", 2, 0, 10));	
	gui.add(time_step.setup( "time_step", 0.2, 0.0, 1.5));
	gui.add(bWire.setup("wire", true));
	gui.add(bPlay.setup("play", false));
	bHide = 0;

	// setup vertex
	size_t count=0;
	total_points = 8;
	mass = 1.f/(total_points);
	gravity.set(0.0, 9.8, 0.0); // inverse

	oldX=0, oldY=0;
	rX=15, rY=0;
	state = 1;
	dist=-23;

	X.resize(total_points);
	tmp_X.resize(total_points);
	V.resize(total_points);
	F.resize(total_points);
	W.resize(total_points); 

	for(int i=0;i<total_points;i++) {	
		W[i] = 1.0f/mass;
		V[i] = ofVec3f(0);
	}

	ofVec3f center = ofVec3f(ofGetWindowWidth()/2, ofGetWindowHeight()/2);
	X[0] = center + ofVec3f( -100,  80 );
	X[1] = center + ofVec3f(  100,  80 );
	X[2] = center + ofVec3f(  100, -80 );
	X[3] = center + ofVec3f( -100, -80 );
	X[4] = center + ofVec3f( -60,  40 );
	X[5] = center + ofVec3f(  60,  40 );
	X[6] = center + ofVec3f(  60, -40 );
	X[7] = center + ofVec3f( -60, -40 );

	// setup constraint
	// Horizontal
	addDistanceConstraint(0, 1, 1.0);
	addDistanceConstraint(2, 3, 1.0);
	addDistanceConstraint(4, 5, 1.0);
	addDistanceConstraint(6, 7, 1.0);

	// Vertical
	addDistanceConstraint(1, 2, 1.0);
	addDistanceConstraint(3, 0, 1.0);
	addDistanceConstraint(5, 6, 1.0);
	addDistanceConstraint(4, 7, 1.0);

	// Shearing distance constraint
	addDistanceConstraint(0, 4, 1.0);
	addDistanceConstraint(1, 5, 1.0);
	addDistanceConstraint(3, 7, 1.0);
	addDistanceConstraint(2, 6, 1.0);
	addDistanceConstraint(4, 6, 1.0);
	addDistanceConstraint(5, 7, 1.0);

	// trianglate
	triangulation.addPoint(X[0]);
	triangulation.addPoint(X[1]);
	triangulation.addPoint(X[2]);
	triangulation.addPoint(X[3]);
	triangulation.addPoint(ofPoint(0,0));
	triangulation.addPoint(ofPoint(ofGetWidth(),0));
	triangulation.addPoint(ofPoint(ofGetWidth(),ofGetHeight()));
	triangulation.addPoint(ofPoint(0,ofGetHeight()));
	triangulation.triangulate();
}

//--------------------------------------------------------------
void ofApp::update(){

	if(bPlay){
		stepPhysics(time_step);
		triangulation.triangleMesh.setVertex(0, X[0]);
		triangulation.triangleMesh.setVertex(1, X[1]);
		triangulation.triangleMesh.setVertex(2, X[2]);
		triangulation.triangleMesh.setVertex(3, X[3]);
	}
}

//--------------------------------------------------------------
void ofApp::draw(){

	ofBackgroundGradient(ofColor::gray, ofColor::black);

	vector<ofMeshFace> faces = triangulation.triangleMesh.getUniqueFaces();

	int count = 0;
	for (int i = 0; i < faces.size(); i++)
	{
		ofVec3f p[3];
		p[0] = faces[i].getVertex(0);
		p[1] = faces[i].getVertex(1);
		p[2] = faces[i].getVertex(2);
		if((p[1]-p[0]).cross(p[2]-p[0]).getNormalized().z != -1){
			triangulation.setPointAtIndex(X[0], 0);
			triangulation.setPointAtIndex(X[1], 1);
			triangulation.setPointAtIndex(X[2], 2);
			triangulation.setPointAtIndex(X[3], 3);
			count++;
		}
	}

	if(count > 0){
		std::cout << count << "	triangulation" << std::endl;
		triangulation.triangulate();
	}

	if(bWire){
		ofSetColor(150);
		//triangulation.triangleMesh.drawWireframe();
		ofNoFill();
		triangulation.draw();
	}

	ofFill();
	ofSetColor(200, 0, 0, 100);
	glBegin(GL_TRIANGLES);
		glVertex3f(X[0].x, X[0].y, X[0].z);
		glVertex3f(X[1].x, X[1].y, X[1].z);
		glVertex3f(X[2].x, X[2].y, X[2].z);
	glEnd();	 

	glBegin(GL_TRIANGLES);
		glVertex3f(X[2].x, X[2].y, X[2].z);
		glVertex3f(X[3].x, X[3].y, X[3].z);
		glVertex3f(X[0].x, X[0].y, X[0].z);
	glEnd();	 

	ofSetColor(255, 165, 0);
	glBegin(GL_TRIANGLES);
		glVertex3f(X[4].x, X[4].y, X[4].z);
		glVertex3f(X[5].x, X[5].y, X[5].z);
		glVertex3f(X[6].x, X[6].y, X[6].z);
	glEnd();	 

	glBegin(GL_TRIANGLES);
		glVertex3f(X[6].x, X[6].y, X[6].z);
		glVertex3f(X[7].x, X[7].y, X[7].z);
		glVertex3f(X[4].x, X[4].y, X[4].z);
	glEnd();	 

	ofSetColor(255);
	for(int i=0; i<4; i++){
		glBegin(GL_LINES);
			glVertex3f(X[i].x, X[i].y, X[i].z);
			glVertex3f(X[i+4].x, X[i+4].y, X[i+4].z);
		glEnd();	 
	}
	
	if(!bHide)
		gui.draw();

	ofSetWindowTitle(ofToString(ofGetFrameRate()));
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	switch (key)
	{
		case 'f':
			ofToggleFullscreen();
			triangulation.setPointAtIndex(ofPoint(0,0), 4);
			triangulation.setPointAtIndex(ofPoint(ofGetWidth(),0), 5);
			triangulation.setPointAtIndex(ofPoint(ofGetWidth(),ofGetHeight()), 6);
			triangulation.setPointAtIndex(ofPoint(0,ofGetHeight()), 7);
			break;
		case 's':
			bPlay = !bPlay;
			break;
		case 'g':
			bHide = !bHide;
			break;
		case 'w':
			bWire = !bWire;
			break;
		case 'r':
			reset();
			break;
		default:
			break;
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

	if (state == 1)
	{
		float valX = (x - oldX)/time_step; 
		float valY = (oldY - y)/time_step; 

		for (int selected_index = 4; selected_index < total_points; selected_index++)
		{
			V[selected_index].x += valX ;
			V[selected_index].y -= valY ;
		}

		oldX = x; 
		oldY = y; 
	}
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

	oldX = x;
	oldY = y;

	int count = 0;
	ofVec3f v(x,y);
	for (int i = 0; i < total_points; i++)
	{
		float distance = (X[i]-v).length();
		if(distance < 100){
			count += 1;
		}
	}

	if (count > 0)
		state = 1;
	else
		state = 0;
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}

//--------------------------------------------------------------
void ofApp::reset()
{
	ofVec3f center = ofVec3f(ofGetWindowWidth()/2, ofGetWindowHeight()/2);
	X[0] = center + ofVec3f( -80,  60 );
	X[1] = center + ofVec3f(  80,  20 );
	X[2] = center + ofVec3f(  80, -60 );
	X[3] = center + ofVec3f( -80, -60 );

	for(int i=0;i<total_points;i++) {	
		V[i] = ofVec3f(0);
	}
}

// update
//--------------------------------------------------------------
void ofApp::stepPhysics(float dt)
{
	computeForce(dt);
	IntegrateExplicitWithDamping(dt);
	updateInternalConstraint(dt);
	updateExternalConstraint(dt);

	intergrate(dt);
}


//--------------------------------------------------------------
void ofApp::computeForce(float dt)
{
	size_t i=0;
	
	for(i=0;i<total_points;i++) {
		F[i] = ofVec3f(0.0); 
		 
		//add gravity force
		if(W[i]>0)	{	 
			//F[i] += gravity;
		}
	}	
}

//--------------------------------------------------------------
void ofApp::IntegrateExplicitWithDamping(float dt)
{
	ofVec3f Xcm = ofVec3f(0);
	ofVec3f Vcm = ofVec3f(0);
	float sumM = 0;
	for(int i=0;i<total_points;i++) {

		V[i] *= global_dampening; //global velocity dampening !!!
		V[i] = V[i] + (F[i]*dt)*W[i];
		
		//calculate the center of mass's position 
		//and velocity for damping calc
		Xcm += (X[i]*mass);
		Vcm += (V[i]*mass);
		sumM += mass;
	}
	Xcm /= sumM; 
	Vcm /= sumM; 

	// TO DO
	//apply center of mass damping

	for(int i=0;i<total_points;i++) {
		if(W[i] <= 0.0) { 
			tmp_X[i] = X[i]; //fixed points
		} else {
			tmp_X[i] = X[i] + (V[i]*dt);
		}
	}
}

//--------------------------------------------------------------
void ofApp::updateInternalConstraint(float dt)
{
	// distance constraint
	for (size_t si=0; si<solver_iterations; ++si) {
		for(int i=0; i<d_constraints.size(); i++) {
			updateDistanceConstraint(i);
		} 
		// collision
		groundCollision();
	}
}

//--------------------------------------------------------------
void ofApp::groundCollision()
{
	for(size_t i=0;i<total_points;i++) {	
		if(tmp_X[i].y>ofGetHeight()){
			//collision with ground
			tmp_X[i].y=ofGetHeight();
		}else if(tmp_X[i].y<0){
			tmp_X[i].y=0;
		}else if(tmp_X[i].x<0){
			tmp_X[i].x=0;
		}else  if(tmp_X[i].x>ofGetWidth()){
			tmp_X[i].x=ofGetWidth();
		}
	}
}

//--------------------------------------------------------------
void ofApp::updateExternalConstraint(float dt)
{
	// collision
}

//--------------------------------------------------------------
void ofApp::intergrate(float dt)
{
	// TO DO
	float inv_dt = 1.0f/dt;
	size_t i=0; 

	for(i=0;i<total_points;i++) {
		V[i] = (tmp_X[i] - X[i])*inv_dt;
		X[i] = tmp_X[i];
	}
}

// constraint
//--------------------------------------------------------------
void ofApp::addDistanceConstraint(int a, int b, float k)
{
	DistanceConstraint c;
	c.p1=a;
	c.p2=b;
	c.k =k;
	c.k_prime = 1.0f-pow((1.0f-c.k), 1.0f/solver_iterations);  //1.0f-pow((1.0f-c.k), 1.0f/ns);
	
	if(c.k_prime>1.0) 
		c.k_prime = 1.0;
	 
	ofVec3f deltaP = X[c.p1]-X[c.p2];
	c.rest_length = deltaP.length();

	d_constraints.push_back(c);
}

//--------------------------------------------------------------
void ofApp::updateDistanceConstraint(int i)
{
	DistanceConstraint c = d_constraints[i];
	ofVec3f dir = tmp_X[c.p1] - tmp_X[c.p2];

	float len = dir.length();
	if(len <= EPSILON) 
		return;
	
	float w1 = W[c.p1];
	float w2 = W[c.p2];
	float invMass = w1+ w2; 
	if(invMass <= EPSILON) 
		return;
 
	ofVec3f dP = (1.0f/invMass) * (len-c.rest_length ) * (dir/len)* c.k_prime;
	if(w1 > 0.0)
		tmp_X[c.p1] -= dP*w1;

	if(w2 > 0.0)
		tmp_X[c.p2] += dP*w2;	
}

float ofApp::calcTriangleQuality(ofPoint p0, ofPoint p1, ofPoint p2)
{
	// The equation of Heron
	float a, b, c;
	a = (p1-p0).length();
	b = (p2-p1).length();
	c = (p0-p2).length();

	float area = calcArea(a, b, c);

	// This equation is borrowed from Air Mesh
	float quality = 12/sqrt(3) * area/(a*a + b*b + c*c);
	return quality;
}

float ofApp::calcArea(float a, float b, float c)
{
	float s = (a+b+c)/2;
	float area = sqrt( s * (s-a) * (s-b) * (s-c) );
	return area;
}