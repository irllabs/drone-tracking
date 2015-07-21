#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"
#include "ofxARToolkitPlus.h"
#include "ofxLearn.h"

#define TRACKED_DRONE_TIMEOUT 30

class TrackedDrone {
    
public:
    
    ofPoint position;
    float orientation;
    
    bool detected;
    int ticksSinceLastDetection;
    
};

class ofApp : public ofBaseApp{
    
public:
    
    void setup();
    void update();
    void draw();
    
    std::map<int,TrackedDrone> trackedDrones;
    
    void exportSceneFrameJSON();
    
    void addContourSampleToClassifier(ofxCv::ContourFinder contourFinder);
    
    int camW, camH;
    int closeSize;
    float scale;
    
    ofVideoGrabber vid;
    
    ofxCv::ContourFinder contourFinder;
    ofxARToolkitPlus artk;
    
    ofxCvColorImage colorImage;
    ofxCvGrayscaleImage artkGrayImage;
    ofxCvGrayscaleImage contourGrayImage;
    
    ofxLearnSVM classifier;
    
};
