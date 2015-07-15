#include "ofApp.h"

void ofApp::setup(){
    
    cam.setup(640,480,0);
    
    ofDisableSmoothing();
    
}

void ofApp::update(){

    cam.update();
    
    cvImage.setFromPixels(cam.image.getPixelsRef().getChannel(0));
    cvImage.threshold(244);
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
    cam.image.draw(640, 0);
    cvImage.draw(0, 0);

    
    for(int p =0; p < contourFinder.getPolylines().size(); p++) {
        ofSetColor(255,0,0);
        ofPolyline pl = contourFinder.getPolyline(p).getResampledBySpacing(1).getSmoothed(50);
    pl.draw();
    
    //ofSetColor(0,255,0);
    for(int i = 0; i < pl.size(); i+=30) {

        ofSetColor(0,0,255);
        ofPoint tpos = pl.getVertices()[i];
        ofCircle(tpos.x, tpos.y, 2);
        
        
        ofSetColor(0,255,0);
        if(pl.size()-i >= 30) {
            
            ofVec3f diff1 = pl.getVertices()[i+0]-pl.getVertices()[i+1];
            ofVec3f diff2 = pl.getVertices()[i+28]-pl.getVertices()[i+29];
            diff1.normalize();
            diff2.normalize();
            float a1 = atan(diff1.y/diff1.x);
            float a2 = atan(diff2.y/diff2.x);
            float totalChange = (a1 - a2)*57.2957795;
            ofLog() /*<< diff1 << " " << diff2 << " "*/ << totalChange;
            
            if((diff1-diff2).length() > 1) {
                ofSetColor(0,255,255);
                ofPoint pos = pl.getVertices()[i+4];
                ofCircle(pos.x, pos.y, 5);
            }
            
            ofPoint pos = pl.getVertices()[i];
            ofSetColor(0,255,0);
            ofLine(pos.x, pos.y, pos.x+diff1.x*10, pos.y+diff1.y*10);
            ofSetColor(255,0,0);
            ofLine(pos.x, pos.y, pos.x+diff2.x*10, pos.y+diff2.y*10);
         
        }
        
        /*
        if(pl.getAngleAtIndexInterpolated(i) >= 135) {
            ofPoint pos = pl.getVertices()[i];
            ofCircle(pos.x, pos.y, 3);
        }*/
    }
        ofLog() << " ";
    }
    
    
}

