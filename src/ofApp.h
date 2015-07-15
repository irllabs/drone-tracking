#pragma once

#include "ofMain.h"
#include "ImageFromCamera.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"

class ofApp : public ofBaseApp{

public:
    void setup();
    void update();
    void draw();

    ImageFromCamera cam;
    
    ofxCvGrayscaleImage cvImage;
    ofxCv::ContourFinder contourFinder;
    
};
