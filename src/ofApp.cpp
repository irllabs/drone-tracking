#include "ofApp.h"

void ofApp::setup(){
    
    ofSetWindowShape(1400, 500);
    ofSetFullscreen(false);
    ofDisableSmoothing();
    
    camW = 1920; camH = 1080;
    vid.setDeviceID(0);
    vid.initGrabber(camW, camH);
    
    artk.setup(camW, camH);
    artk.setThreshold(0);

    scale = 0.4;
    closeSize = 250;
    
}

void ofApp::update(){
    
    vid.update();
    
    // get image from camera
    colorImage.setFromPixels(vid.getPixels(), camW, camH);
    contourGrayImage = colorImage;
    artkGrayImage = colorImage;
    
    // find artk trackers
    artkGrayImage.threshold(mouseX);
    artk.update(artkGrayImage.getPixels());
    
    // find contours
    contourGrayImage.threshold(mouseX);
    contourGrayImage.invert();
    
    contourFinder.setMinArea(300);
    contourFinder.setMaxArea(camW*camH);
    contourFinder.setSortBySize(true);
    
    contourFinder.findContours(ofxCv::toCv(contourGrayImage));
    
    // update our list of tracked drones
    int nMarkersDetected = artk.getNumDetectedMarkers();
    for(int i = 0; i < nMarkersDetected; i++) {
        
        ofPoint position = artk.getDetectedMarkerCenter(i);
        int id = artk.getMarkerID(i);
        
        // get the matrix for this marker from artk
        ofMatrix4x4 h = artk.getMatrix(i);
        ofVec3f v;
        ofQuaternion rotation;
        ofVec3f s;
        ofQuaternion so;
        h.decompose(v, rotation, s, so);
        
        // do lots of trig or something to find tracker rotation
        double orientation = atan2(2*(rotation.x()*rotation.y()+rotation.w()*rotation.z()),rotation.w()*rotation.w()+rotation.x()*rotation.x()-rotation.y()*rotation.y()-rotation.z()*rotation.z());
        
        // update the list
        TrackedDrone drone;
        drone.position = position;
        drone.orientation = orientation;
        trackedDrones[id] = drone;
        
    }
    
}

void ofApp::draw(){
    
    // draw raw camera feed and thresholded camera feed
    ofSetColor(255,255,255);
    vid.draw(camW * scale, 0, camW * scale, camH * scale);
    artkGrayImage.draw(0, 0, camW * scale, camH * scale);
    
    ofScale(scale,scale,scale);
    
    int dronesCount = 0;
    for(std::map<int,TrackedDrone>::iterator iterator = trackedDrones.begin();
        iterator != trackedDrones.end();
        iterator++) {
        
        int id = iterator->first;
        TrackedDrone drone = iterator->second;
        
        ofDrawBitmapString("    " +ofToString(id),
                           drone.position.x,
                           drone.position.y);
        
        // draw a line in the direction that the tracker is facing
        ofSetLineWidth(3);
        ofSetColor(0,255,0);
        ofLine(drone.position.x,
               drone.position.y,
               drone.position.x + cos(drone.orientation)*150,
               drone.position.y + sin(drone.orientation)*150);
        ofSetLineWidth(1);
        
        // crop the tracker out of the camera image
        ofImage droneImg;
        droneImg.setFromPixels(vid.getPixelsRef());
        droneImg.crop(drone.position.x - closeSize /2,
                      drone.position.y - closeSize / 2,
                      closeSize,
                      closeSize);
        
        // draw the tracker in the corner of the screen
        ofPushMatrix();
        ofTranslate(200 + dronesCount*250, 200);
        ofRotate(drone.orientation*-57.2957795, 0,0,1);
        ofTranslate(-droneImg.width/2,-droneImg.height/2);
        ofSetColor(255,255,255);
        droneImg.draw(0,0);
        ofPopMatrix();
        
        dronesCount++;
        
    }
    
}

void ofApp::addContourSampleToClassifier(ofxCv::ContourFinder contourFinder) {
    
    // the format of our samples is as follows:
    // 10 pairs of values, a total of 20 values.
    // each pair of values corresponds to the 2d normal of a contour point.
    // these values are found by resamping a contour to twelve points
    // (two extra points) and calculating the normal.
    
    {
        vector<double> shapeSample;
        // put stuff here
        
        int label = 0;
        
        // add sample to our classifier
        classifier.addTrainingInstance(shapeSample, label);
    }
    
    classifier.train();
    
}

