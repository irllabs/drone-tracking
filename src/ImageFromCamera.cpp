#include "ImageFromCamera.h"

void ImageFromCamera::setup(int w, int h, int i) {
    
    camWidth = w;
    camHeight = h;
    
    vidGrabber.setDeviceID(i);
    vidGrabber.setDesiredFrameRate(60);
    vidGrabber.initGrabber(camWidth,camHeight);
    
    videoInverted = new unsigned char[camWidth*camHeight*3];
    videoTexture.allocate(camWidth,camHeight, GL_RGB);
    
    image.allocate(camWidth, camHeight, OF_IMAGE_COLOR);
    
}
void ImageFromCamera::update() {
    
    vidGrabber.update();
    
    if (vidGrabber.isFrameNew()){
        int totalPixels = camWidth*camHeight*3;
        unsigned char * pixels = vidGrabber.getPixels();
        for (int i = 0; i < totalPixels; i++){
            videoInverted[i] = pixels[i];
        }
        videoTexture.loadData(videoInverted, camWidth,camHeight, GL_RGB);
        
        image.setFromPixels(vidGrabber.getPixelsRef());
        
    }
    
}