#include "ofApp.h"

void ofApp::setup(){
    
    cam.setup(640,480,0);
    
}

void ofApp::update(){

    cam.update();
    
    cvImage.setFromPixels(cam.image.getPixelsRef().getChannel(0));
    cvImage.threshold((float)mouseX/ofGetWidth()*255.0);
    cvImage.invert();
    
    contourFinder.setMinArea(300);
    contourFinder.setMaxArea(640*640);
    //contourFinder.setFindHoles(true);
    contourFinder.setSortBySize(true);
    
    //contourFinder.setThreshold(100);
    contourFinder.findContours(ofxCv::toCv(cvImage));
    
}

void ofApp::draw(){

    ofSetColor(255,255,255);
    cam.image.draw(0, 0);
    cvImage.draw(640, 0);
    
    ofSetColor(255,0,0);
    //contourFinder.draw();
    
    ofPolyline pl = contourFinder.getPolyline(0).getResampledBySpacing(2);
    pl.draw();
    
    ofSetColor(0,255,0);
    for(int i = 0; i < pl.size(); i++) {

        if(pl.getAngleAtIndexInterpolated(i) >= 135) {
            ofPoint p = pl.getVertices()[i];
            ofCircle(p.x, p.y, 3);
        }
    }
    
    
}

