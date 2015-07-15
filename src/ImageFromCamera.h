#ifndef __dranimate__ImageFromCamera__
#define __dranimate__ImageFromCamera__

#include <stdio.h>

#include "ofMain.h"

class ImageFromCamera {
    
public:
    
    void setup(int w, int h, int i);
    void update();
    
    ofVideoGrabber vidGrabber;
    unsigned char *videoInverted;
    ofTexture videoTexture;
    int camWidth;
    int camHeight;
    
    ofImage image;
    
};

#endif