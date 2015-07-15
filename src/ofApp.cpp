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
    contourFinder.setSortBySize(true);
    
    contourFinder.findContours(ofxCv::toCv(cvImage));
    
}

void ofApp::draw(){

    // draw raw camera feed and thresholded camera feed
    
    ofSetColor(255,255,255);
    cam.image.draw(640, 0);
    cvImage.draw(0, 0);

    for(int p = 0; p < contourFinder.getPolylines().size(); p++) {
        
        // get and draw the current polyline
        
        ofSetColor(255,0,0);
        ofPolyline pl = contourFinder.getPolyline(p).getResampledBySpacing(1).getSmoothed(20);
        pl.draw();
    
        // find changes of angle of each segment of the contour
        
        int segmentSize = 15;
        
        for(int i = 0; i < pl.size(); i += segmentSize) {

            // draw where this segment of the contour starts
            
            ofSetColor(0,0,255);
            ofPoint tpos = pl.getVertices()[i];
            ofCircle(tpos.x, tpos.y, 2);
            
            ofSetColor(0,255,0);
            if(pl.size()-i >= segmentSize) {
                
                // find total change in direction over these points
                
                ofVec3f diff1 =  pl.getVertices()[i+0]
                                -pl.getVertices()[i+1];
                ofVec3f diff2 =  pl.getVertices()[i+segmentSize-2]
                                -pl.getVertices()[i+segmentSize-1];
                diff1.normalize();
                diff2.normalize();
                float a1 = atan(diff1.y/diff1.x);
                float a2 = atan(diff2.y/diff2.x);
                float totalChange = (a1 - a2)*57.2957795;
                
                // if the change was big enough, we found a triangle edge
                
                if((diff1-diff2).length() > 1) {
                    ofSetColor(0,255,255);
                    ofPoint pos = pl.getVertices()[i+4];
                    ofCircle(pos.x, pos.y, 5);
                }
                
                // draw the angles (debug)
                
                ofPoint pos = pl.getVertices()[i];
                ofSetColor(0,255,0);
                ofLine(pos.x, pos.y, pos.x+diff1.x*10, pos.y+diff1.y*10);
                ofSetColor(255,0,0);
                ofLine(pos.x, pos.y, pos.x+diff2.x*10, pos.y+diff2.y*10);
             
            }
            
        }
        
    }
    
}

