#include "ofApp.h"

void ofApp::setup(){
    
    ofSetWindowShape(1080, 540);
    ofSetFullscreen(false);
    ofDisableSmoothing();
    
    camW = 1920; camH = 1080;
    
    if(USE_LIVE_FEED) {
        // setup camera
        liveCam.setDeviceID(0);
        liveCam.initGrabber(camW, camH);
    }
    else {
        // load a snapshot
        ofFileDialogResult openFileResult = ofSystemLoadDialog("Select a camera snapshot:",true);
        
        if (openFileResult.bSuccess){
            camSnapshot.loadImage(openFileResult.getPath());
        }
    }
    
    artk.setup(camW, camH);
    artk.setThreshold(0);

    closeSize = 250;
    
    sound.loadSound("camera.mp3");
    
}

void ofApp::update(){
    
    liveCam.update();
    
    // update cv source image
    if(USE_LIVE_FEED) {
        colorImage.setFromPixels(liveCam.getPixels(), camW, camH);
    } else {
        colorImage.setFromPixels(camSnapshot.getPixels(), camW, camH);
    }
    
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
    for(std::map<int,TrackedDrone>::iterator iterator = trackedDrones.begin();
        iterator != trackedDrones.end();
        iterator++) {
        
        TrackedDrone *drone = &iterator->second;
        drone->detected = false;
        
    }
    
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
        
        // decompose matrix to get quaternion
        h.decompose(v, rotation, s, so);
        
        // do lots of trig or something to find z-rotation from quaternion
        double orientation = atan2(2*(rotation.x()*rotation.y()+rotation.w()*rotation.z()),rotation.w()*rotation.w()+rotation.x()*rotation.x()-rotation.y()*rotation.y()-rotation.z()*rotation.z());
        
        // find tracker contour (should just be a square)
        int trackerContourID = -1;
        for(int c = 0; c < contourFinder.getContours().size(); c++) {
            ofPolyline contour = contourFinder.getPolyline(c);
            if(contour.inside(position)) {
                trackerContourID = c;
            }
        }
        
        // find the tracker size from the tracker contour
        float size = 0;
        if(trackerContourID != -1) {
            ofPolyline contour = contourFinder.getPolyline(trackerContourID);
            
            for(int p = 0; p < contour.size(); p++) {
                float d = (contour.getVertices()[p] - position).length();
                d = abs(d);
                if(d > size) {
                    size = d;
                }
            }
        }
        
        // find the contour of this drone's classifier
        int classifierContourID = -1;
        if(trackerContourID != -1) {
            ofPoint classifierCenter = ofPoint(position.x + cos(orientation)*size,
                                               position.y + sin(orientation)*size);
                                               
            for(int c = 0; c < contourFinder.getContours().size(); c++) {
                ofPolyline contour = contourFinder.getPolyline(c);
                if(contour.inside(classifierCenter)) {
                    classifierContourID = c;
                }
            }
        }
        
        // classify this drone
        ofPolyline classifierContour;
        if(classifierContourID != -1) {
            
            classifierContour = contourFinder.getPolyline(classifierContourID);
            ofPoint centroid = classifierContour.getCentroid2D();
            
            for(int p = 0; p < classifierContour.size(); p++) {
                classifierContour.getVertices()[p].rotate(orientation*-57.2,
                                                          centroid,
                                                          ofVec3f(0,0,1));
            }
            
        }
        
        // update this drone's data in the list
        TrackedDrone drone;
            drone.position = position;
            drone.orientation = orientation;
        
            drone.detected = true;
            drone.ticksSinceLastDetection = 0;
        
            drone.size = size;
        
            drone.trackerContourID = trackerContourID;
            drone.classifierContourID = classifierContourID;
        
            drone.classifierContour = classifierContour;
        trackedDrones[id] = drone;
        
    }
    
    for(std::map<int,TrackedDrone>::iterator iterator = trackedDrones.begin();
        iterator != trackedDrones.end();
        iterator++) {
        
        TrackedDrone *drone = &iterator->second;
        
        if(drone->detected) {
            drone->ticksSinceLastDetection = 0;
        } else {
            drone->ticksSinceLastDetection++;
        }
        
        if(drone->ticksSinceLastDetection >= TRACKED_DRONE_TIMEOUT) {
            std::map<int,TrackedDrone>::iterator it = trackedDrones.find (iterator->first);
            trackedDrones.erase(it);
        }
        
    }
    
}

void ofApp::draw(){
    
    // draw raw camera feed (or snapshot) and thresholded camera feed
    ofSetColor(255,255,255);
    int drawW = camW*IMG_DRAW_SCALE;
    int drawH = camH*IMG_DRAW_SCALE;
    
    if(USE_LIVE_FEED) {
        liveCam.draw(camW*CV_PREVIEW_SCALE, 0, drawW, drawH);
    } else {
        camSnapshot.draw(camW*CV_PREVIEW_SCALE, 0, drawW, drawH);
    }
    
    // draw thresholded cv image
    artkGrayImage.draw(0, ofGetHeight()-camH*CV_PREVIEW_SCALE,
                       camW*CV_PREVIEW_SCALE, camH*CV_PREVIEW_SCALE);
    
    ofPushMatrix();
    ofTranslate(camW*CV_PREVIEW_SCALE,0,0);
    ofScale(IMG_DRAW_SCALE,IMG_DRAW_SCALE);
    
    // draw all tracked drones
    int dronesCount = 0;
    for(std::map<int,TrackedDrone>::iterator iterator = trackedDrones.begin();
        iterator != trackedDrones.end();
        iterator++) {
        
        int id = iterator->first;
        TrackedDrone drone = iterator->second;
        
        ofSetColor(255,0,0);
        ofDrawBitmapString("    " +ofToString(id)+" "+ofToString(drone.ticksSinceLastDetection),
                           drone.position.x,
                           drone.position.y);
        
        // draw a line in the direction that the tracker is facing
        ofSetLineWidth(3);
        ofSetColor(0,255,0);
        ofLine(drone.position.x,
               drone.position.y,
               drone.position.x + cos(drone.orientation)*drone.size,
               drone.position.y + sin(drone.orientation)*drone.size);
        ofSetLineWidth(1);
        
        // draw the contour of this drone's tracker
        if(drone.trackerContourID != -1) {
            ofPolyline contour = contourFinder.getPolyline(drone.trackerContourID);
            ofSetColor(255, 0, 0);
            contour.draw();
        }
        
        // draw the contour of this drone's classifier
        if(drone.classifierContourID != -1) {
            ofSetColor(0, 255, 0);
            drone.classifierContour.draw();
        }
        
        // crop the tracker out of the camera image
        ofImage droneImg;
        if(USE_LIVE_FEED) {
            droneImg.setFromPixels(liveCam.getPixelsRef());
        } else {
            droneImg.setFromPixels(camSnapshot.getPixelsRef());
        }
        droneImg.crop(drone.position.x - closeSize / 2,
                      drone.position.y - closeSize / 2,
                      closeSize,
                      closeSize);
        
        // draw the tracker in the corner of the screen
        ofPushMatrix();
        ofTranslate(200 + dronesCount*250, 200);
        ofRotate(drone.orientation*-57.2957795, 0,0,1);
        ofTranslate(-droneImg.width/2,
                    -droneImg.height/2);
        ofSetColor(255,255,255);
        //droneImg.draw(0,0);
        ofPopMatrix();
        
        dronesCount++;
        
    }
    
    ofPopMatrix();
    
    // draw camera flash
    flashTimer-=30;
    if(flashTimer < 0) {
        flashTimer = 0;
    } else {
        ofSetColor(ofColor(255,255,255,flashTimer));
        ofRect(0,0,ofGetWidth(),ofGetHeight());
    }
    
}

void ofApp::keyReleased(int key) {
    
    if(key == ' ') {
        
        // spacebar pressed...
        
        // save currently tracked drone data as json
        exportSceneFrameJSON();
        
        // save camera feed image as well
        ofImage i;
        i.setFromPixels(liveCam.getPixels(), camW, camH, OF_IMAGE_COLOR);
        i.saveImage("test"+ofGetTimestampString()+".png");
        
        // start the 'camera flash' (just for fun)
        flashTimer = 255;
    }
    
    if(key == 'l' && !USE_LIVE_FEED) {
        
        // load new snapshot
        ofFileDialogResult openFileResult = ofSystemLoadDialog("Select a camera snapshot:",true);
        
        if (openFileResult.bSuccess){
            camSnapshot.loadImage(openFileResult.getPath());
        }
        
    }
    
}

void ofApp::exportSceneFrameJSON() {
    
    ofFile newfile(ofToDataPath("coordinates.json"), ofFile::WriteOnly);
    string time = ofToString(ofGetElapsedTimef());
    
    
    
    /*for(Json::ValueIterator i = untimedData.begin() ; i != untimedData.end(); i++) {
        string id = i.key().asString();
        //data[id][time]["position"] = untimedData[id]["position"];
        //data[ofToString(id)][time]["rotation"] = untimedData[id]["rotation"];
        cout << data;
    }*/
    
    if (data != ofxJSONElement::null) {
        newfile << data;
    }
    
    sound.play();
    
}

void ofApp::addContourSampleToClassifier(vector<ofPoint> contour) {
    
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

