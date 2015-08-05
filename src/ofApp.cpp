#include "ofApp.h"

void ofApp::setup(){
    
    ofSetWindowShape(1280, 720);
    ofSetFullscreen(false);
    ofDisableSmoothing();
    
    camW = 640*CAM_IMG_SCALE;
    camH = 480*CAM_IMG_SCALE;
    
    if(USE_LIVE_FEED) {
        // setup camera
        liveCam.setDeviceID(0);
        liveCam.initGrabber(camW, camH);
    }
    else {
        // load a snapshot
        ofFileDialogResult openFileResult = ofSystemLoadDialog("Select a camera snapshot:",true);
        
        if (openFileResult.bSuccess) {
            camSnapshot.loadImage(openFileResult.getPath());
        }
    }
    
    artk.setup(camW, camH);
    artk.setThreshold(0);

    closeSize = 250;
    
    frame = 0;
    timeSeconds = 0.0;
    
    nTrainingSamples = 0;
    
    threshold = 77;
    
    sound.loadSound("camera.mp3");
    
    classificationData.load("classificationTrainingData.xml");
    
}
void ofApp::update(){
    
    liveCam.update();
    
    // update cv source image
    if(USE_LIVE_FEED) {
        colorImage.setFromPixels(liveCam.getPixels(), camW, camH);
        colorImage.rotate(180, colorImage.width/2, colorImage.height/2);
    } else {
        colorImage.setFromPixels(camSnapshot.getPixels(), camW, camH);
    }
    
    contourGrayImage = colorImage;
    artkGrayImage = colorImage;
    
    // find artk trackers
    artkGrayImage.threshold(threshold);
    artk.update(artkGrayImage.getPixels());
    
    // find contours
    contourGrayImage.threshold(threshold);
    contourGrayImage.invert();
    
    contourFinder.setMinArea(100);
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
        
        TrackedDrone drone;
        
        // get the unique id of this marker
        int id = artk.getMarkerID(i);
        
        // get the position of this marker
        drone.position = artk.getDetectedMarkerCenter(i);
        
        // get the matrix for this marker from artk
        ofMatrix4x4 h = artk.getOrientationMatrix(i);
        ofVec3f v;
        ofQuaternion rotation;
        ofVec3f s;
        ofQuaternion so;
        
        vector<ofPoint> corners;
        artk.getDetectedMarkerBorderCorners(i, corners);
        vector<ofPoint> displayImageCorners;
        displayImageCorners.push_back(ofPoint(0, 0));
        displayImageCorners.push_back(ofPoint(100, 0));
        displayImageCorners.push_back(ofPoint(100,100));
        displayImageCorners.push_back(ofPoint(0, 100));
        ofMatrix4x4 hom = artk.getHomography(i, displayImageCorners);
        
        // decompose matrix to get quaternion
        hom.decompose(v, rotation, s, so);
        
        // do lots of trig or something to find z-rotation from quaternion
        // which we use for the drone's orientation, and then find a more
        // stable orientation by finding the most similar angle from the
        // marker's corners which is very stable ...
        float noisyOrientation = atan2(2*(rotation.x()*rotation.y()+rotation.w()*rotation.z()),rotation.w()*rotation.w()+rotation.x()*rotation.x()-rotation.y()*rotation.y()-rotation.z()*rotation.z());
        
        float betterOrientation = noisyOrientation;
        float nearestAngleAmt = INT_MAX;
        vector<ofPoint> orientationCorners;
        artk.getDetectedMarkerCorners(i, orientationCorners);
        for(int i = 0; i < orientationCorners.size(); i++) {
            int ni = i+1;
            if(ni == 4) ni = 0;
            float cornerAngle = -atan2(orientationCorners[ni].x-orientationCorners[i].x,orientationCorners[ni].y-orientationCorners[i].y);
            float angleDist = abs(noisyOrientation-cornerAngle);
            // i'm sleepy
            if(angleDist < nearestAngleAmt) {
                betterOrientation = cornerAngle;
                nearestAngleAmt = angleDist;
            }
        }
        drone.orientation = betterOrientation;
        
        // find tracker contour (should just be a square)
        drone.trackerContourID = -1;
        for(int c = 0; c < contourFinder.getContours().size(); c++) {
            ofPolyline contour = contourFinder.getPolyline(c);
            if(contour.inside(drone.position)) {
                drone.trackerContourID = c;
            }
        }
        
        // find the tracker size from the tracker contour
        drone.size = 0;
        if(drone.trackerContourID != -1) {
            ofPolyline contour = contourFinder.getPolyline(drone.trackerContourID);
            
            for(int p = 0; p < contour.size(); p++) {
                float d = (contour.getVertices()[p] - drone.position).length();
                d = abs(d);
                if(d > drone.size) {
                    drone.size = d;
                }
            }
        }
        drone.size *= SCALE_SIZE; // nima's new drone markers have bigger margins
        
        // find the contour of this drone's classifier
        drone.classifierContourID = -1;
        if(drone.trackerContourID != -1) {
            float cx = drone.position.x + cos(drone.orientation-PI/2)*drone.size;
            float cy = drone.position.y + sin(drone.orientation-PI/2)*drone.size;
            ofPoint classifierCenter = ofPoint(cx,cy);
                                               
            for(int c = 0; c < contourFinder.getContours().size(); c++) {
                ofPolyline contour = contourFinder.getPolyline(c);
                if(contour.inside(classifierCenter)) {
                    drone.classifierContourID = c;
                }
            }
        }
        
        // find the altitude of this drone
        drone.altitudeContourID = -1;
        if(drone.trackerContourID != -1) {
            float cx = drone.position.x + cos(drone.orientation+PI/2)*drone.size*0.8;
            float cy = drone.position.y + sin(drone.orientation+PI/2)*drone.size*0.8;
            ofPoint altitudeCenter = ofPoint(cx,cy);
            
            for(int c = 0; c < contourFinder.getContours().size(); c++) {
                ofPolyline contour = contourFinder.getPolyline(c);
                if(contour.inside(altitudeCenter)) {
                    drone.altitudeContourID = c;
                }
            }
        }
        drone.altitude = 0;
        if(drone.altitudeContourID != -1) {
            ofPolyline altitudeContour = contourFinder.getPolyline(drone.altitudeContourID);
            float farthestDistance = 0;
            ofPoint farthestDistancePoint;
            for(int p = 0; p < altitudeContour.size(); p++) {
                float d = (altitudeContour.getVertices()[p]-drone.position).length();
                if(d > farthestDistance) {
                    farthestDistance = d;
                    farthestDistancePoint = altitudeContour.getVertices()[p];
                }
            }
            
            // so sleepy
            float altitude = atan2(drone.position.y-farthestDistancePoint.y,
                                   drone.position.x-farthestDistancePoint.x);
            altitude -= drone.orientation;
            if(altitude < 0) altitude+=2*PI;
            drone.altitude = altitude;
            
            if(drone.altitude < 4.7) {
                drone.altitudeClass = 'L';
            } else if(drone.altitude < 4.85) {
                drone.altitudeClass = 'M';
            } else {
                drone.altitudeClass = 'H';
            }
            
        }
        
        // classify this drone
        ofPolyline classifierContour;
        drone.droneClass = -1;
        if(drone.classifierContourID != -1) {
            
            classifierContour = contourFinder.getPolyline(drone.classifierContourID);
            
            drone.droneClass = classifyContour(contourToClassifiableVector(classifierContour, drone.orientation, drone.position));
            
        }
        
        // update this drone's data in the list
        drone.detected = true;
        drone.ticksSinceLastDetection = 0;
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
        artkGrayImage.draw(camW*CV_PREVIEW_SCALE, 0, drawW, drawH);
    } else {
        camSnapshot.draw(camW*CV_PREVIEW_SCALE, 0, drawW, drawH);
    }
    
    // draw thresholded cv image
    liveCam.draw(0, ofGetHeight()-camH*CV_PREVIEW_SCALE,
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
        ofDrawBitmapString("    "
                           +ofToString(id)+" "
                           +ofToString(drone.droneClass)+" "
                           +drone.altitudeClass+" "
                           +ofToString(drone.ticksSinceLastDetection),
                           drone.position.x,
                           drone.position.y);
        
        // draw a line in the direction that the tracker is facing
        ofSetLineWidth(2);
        ofSetColor(0,255,0);
        ofLine(drone.position.x,
               drone.position.y,
               drone.position.x + cos(drone.orientation-PI/2)*drone.size*ORIENTATION_VEC_DRAW_LEN,
               drone.position.y + sin(drone.orientation-PI/2)*drone.size*ORIENTATION_VEC_DRAW_LEN);
        ofSetLineWidth(1);
        
        // draw the marker's corners
        vector<ofPoint> corners;
        artk.getDetectedMarkerCorners(dronesCount, corners);
        for(int i = 0; i < corners.size(); i++) {
            ofSetColor(255,0,255);
            ofCircle(corners[i].x, corners[i].y, 5);
        }
        
        // draw the contour of this drone's marker
        if(drone.trackerContourID != -1) {
            ofPolyline contour = contourFinder.getPolyline(drone.trackerContourID);
            ofSetColor(255, 0, 0);
            contour.draw();
        }
        
        // draw the contour of this drone's classifier
        if(drone.classifierContourID != -1) {
            ofSetColor(0, 255, 0);
            contourFinder.getPolyline(drone.classifierContourID).draw();
            
        }
        
        // draw the contour of this drone's altitude meter arrow
        if(drone.altitudeContourID != -1) {
            ofSetColor(0, 255, 0);
            contourFinder.getPolyline(drone.altitudeContourID).draw();
            
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
    
    //ofSetColor(255,0,0);
    //contourFinder.draw();
    
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
        
        // save currently tracked drone data as xml
        writeSceneFrameXML();
        
        // start the 'camera flash' (just for fun)
        flashTimer = 255;
        
        // update frame and time
        frame++;
        timeSeconds += 1.0;
        
    }
    
    if(key == 'd') {
        
        ofFileDialogResult openFileResult = ofSystemSaveDialog("untitled.xml", "Export drone choreography");
        
        if (openFileResult.bSuccess) {
            trackedDronesXML.save(openFileResult.getPath());
        }
        
        trackedDronesXML.save("drones.xml");
        
    }
    
    if(key == 'c') {
        
        // save classification data
        writeClassificationData();
        
    }
    
    if(key == 't') {
        
        trainClassifier();
        
    }
    
    if(key == 'l' && !USE_LIVE_FEED) {
        
        // load new snapshot
        ofFileDialogResult openFileResult = ofSystemLoadDialog("Select a camera snapshot:",true);
        
        if (openFileResult.bSuccess){
            camSnapshot.loadImage(openFileResult.getPath());
        }
        
    }
    
}

void ofApp::writeSceneFrameXML() {
    
    string time = ofToString(ofGetElapsedTimef());
    
    trackedDronesXML.addTag("frame");
    trackedDronesXML.pushTag("frame", frame);
    
    trackedDronesXML.addValue("time", timeSeconds);
    
    int droneCount = 0;
    for(std::map<int,TrackedDrone>::iterator iterator = trackedDrones.begin();
        iterator != trackedDrones.end();
        iterator++) {
        
        int id = iterator->first;
        TrackedDrone drone = iterator->second;
        
        trackedDronesXML.addTag("drone");
        trackedDronesXML.pushTag("drone", droneCount);
            trackedDronesXML.addValue("id", id);
            trackedDronesXML.addTag("position");
            trackedDronesXML.pushTag("position");
                trackedDronesXML.addValue("x", drone.position.x);
                trackedDronesXML.addValue("y", drone.position.y);
                trackedDronesXML.addValue("z", drone.altitude);
            trackedDronesXML.popTag();
            trackedDronesXML.addValue("orientation", drone.orientation);
            trackedDronesXML.addValue("class", drone.droneClass);
        trackedDronesXML.popTag();
        
        droneCount++;
        
    }
    
    trackedDronesXML.popTag();
    
    sound.play();
    
}

vector<double> ofApp::contourToClassifiableVector(ofPolyline contour, float droneOrientation, ofPoint dronePosition) {
    
    // make sure contour is facing correct direction
    ofPoint c = contour.getCentroid2D();
    for(int p = 0; p < contour.size(); p++) {
        contour.getVertices()[p].rotate(droneOrientation*-57.2,
                                        c,
                                        ofVec3f(0,0,1));
    }
    
    // normalize entire contour
    
    float farthestDistance = 0;
    ofPoint farthestDistancePoint;
    for(int p = 0; p < contour.size(); p++) {
        float d = (contour.getVertices()[p]-c).length();
        if(d > farthestDistance) {
            farthestDistance = d;
            farthestDistancePoint = contour.getVertices()[p];
        }
    }
    for(int i = 0; i < contour.getVertices().size(); i++) {
        
        ofVec3f v = contour.getVertices()[i];
        v /= farthestDistance;
        contour.getVertices()[i].set(v);
        
    }
    
    // build the sample vector
    vector<double> classifiableVector;
    classifiableVector.push_back(contour.getArea());
    classifiableVector.push_back(contour.getPerimeter());
    classifiableVector.push_back(contour.getBoundingBox().getArea());
    classifiableVector.push_back(contour.getBoundingBox().getPerimeter());
    return classifiableVector;
    
}
void ofApp::addContourToClassifier(vector<double> contour, int label) {
    
    // the format of our samples is as follows:
    // 10 pairs of values, a total of 20 values.
    // each pair of values corresponds to the 2d normal of a contour point.
    // these values are found by resamping a contour to twelve points
    // (two extra points) and calculating the normal.
    
    // add sample to our classifier
    classifier.addTrainingInstance(contour, label);
    
    
}
double ofApp::classifyContour(vector<double> contour) {
    
    return classifier.predict(contour);
    
}

void ofApp::writeClassificationData() {
    
    for(std::map<int,TrackedDrone>::iterator iterator = trackedDrones.begin();
        iterator != trackedDrones.end();
        iterator++) {
        
        int id = iterator->first;
        TrackedDrone drone = iterator->second;
        
        classificationData.addTag("trainingSample");
        classificationData.pushTag("trainingSample",nTrainingSamples);
        classificationData.addValue("class", (int)(nTrainingSamples/20));
        vector<double> classifierContour = contourToClassifiableVector(contourFinder.getPolyline((drone.classifierContourID)),drone.orientation,drone.position);
        for(int i = 0; i < classifierContour.size(); i++) {
            classificationData.addValue("v"+ofToString(i), classifierContour[i]);
        }
        classificationData.popTag();
        classificationData.save("classificationTrainingData.xml");
        
        ofLog() << "wrote classifiction data to file for drone id " << id;
        ofLog() << "there are " << nTrainingSamples << " samples in the data set.";
        
        nTrainingSamples++;
        
        break;
        
    }
    
}
void ofApp::trainClassifier() {
    
    int nSamples = classificationData.getNumTags("trainingSample");
    ofLog() << nSamples;
    for(int i = 0; i < nSamples; i++) {
        vector<double> trainingValues;
        
        classificationData.pushTag("trainingSample",i);
        for(int v = 0; v < 20; v++) {
            trainingValues.push_back(classificationData.getValue("v"+ofToString(v), 0.0));
            ofLog() << classificationData.getValue("v"+ofToString(v), 0.0);
        }
        
        classifier.addTrainingInstance(trainingValues, classificationData.getValue("class", -1));
        
        ofLog() << classificationData.getValue("class", -1);
        
        classificationData.popTag();
    }
    
    classifier.train();
    
}
