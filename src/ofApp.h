#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"
#include "ofxARToolkitPlus.h"
#include "ofxLearn.h"
#include "ofxJSONElement.h"

#define TRACKED_DRONE_TIMEOUT 30
#define IMG_DRAW_SCALE 0.5
#define CV_PREVIEW_SCALE 0.1
#define USE_LIVE_FEED false

class TrackedDrone {
    
public:
    
    ofPoint position;
    float orientation;
    
    bool detected;
    int ticksSinceLastDetection;
    
    float size;
    
    int trackerContourID;
    int classifierContourID;
    
    ofPolyline classifierContour;
    
};

class ofApp : public ofBaseApp{
    
public:
    
    void setup();
    void update();
    void draw();
    
    void keyReleased(int key);
    
    std::map<int,TrackedDrone> trackedDrones;
    
    void exportSceneFrameJSON();
    
    void addContourSampleToClassifier(vector<ofPoint> contour);
    
    int camW, camH;
    int closeSize;
    
    float flashTimer;
    
    ofVideoGrabber liveCam;
    ofImage camSnapshot;
    
    ofSoundPlayer sound;
    
    ofxJSONElement data;
    
    ofxCv::ContourFinder contourFinder;
    ofxARToolkitPlus artk;
    
    ofxCvColorImage colorImage;
    ofxCvGrayscaleImage artkGrayImage;
    ofxCvGrayscaleImage contourGrayImage;
    
    ofxLearnSVM classifier;
    
};
