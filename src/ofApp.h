#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"
#include "ofxARToolkitPlus.h"
#include "ofxLearn.h"
#include "ofxXmlSettings.h"

#define TRACKED_DRONE_TIMEOUT 30
#define IMG_DRAW_SCALE 0.5
#define CV_PREVIEW_SCALE 0.1
#define USE_LIVE_FEED true

class TrackedDrone {
    
public:
    
    ofPoint position;
    float altitude;
    float orientation;
    
    bool detected;
    int ticksSinceLastDetection;
    
    float size;
    
    int trackerContourID;
    int classifierContourID;
    
    int droneClass;
    
};

class ofApp : public ofBaseApp{
    
public:
    
    void setup();
    void update();
    void draw();
    
    void keyReleased(int key);
    
    void writeSceneFrameXML();
    
    vector<double> contourToClassifiableVector(ofPolyline contour, float droneOrientation, ofPoint dronePosition);
    void addContourToClassifier(vector<double> contour, int label);
    double classifyContour(vector<double> contour);
    
    void writeClassificationData();
    void trainClassifier();
    
    std::map<int,TrackedDrone> trackedDrones;
    
    int camW, camH;
    int closeSize;
    
    int frame;
    float timeSeconds;
    
    int nTrainingSamples;
    
    float flashTimer;
    
    ofVideoGrabber liveCam;
    ofImage camSnapshot;
    
    ofxXmlSettings trackedDronesXML;
    ofxXmlSettings classificationData;
    
    ofSoundPlayer sound;
    
    ofxCv::ContourFinder contourFinder;
    ofxARToolkitPlus artk;
    
    ofxCvColorImage colorImage;
    ofxCvGrayscaleImage artkGrayImage;
    ofxCvGrayscaleImage contourGrayImage;
    
    ofxLearnSVM classifier;
    
};
