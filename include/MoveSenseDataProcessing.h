#pragma once
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

namespace movesense {

#ifdef _WIN32
#define MOVESENSE_EXPORT __declspec(dllexport)
#else 
#define MOVESENSE_EXPORT
#endif
	
class MOVESENSE_EXPORT MoveSenseDataProcessing
{
public:
    MoveSenseDataProcessing();
    ~MoveSenseDataProcessing();

    bool compressionPreprocessing(unsigned char *src, unsigned char *des, int srcDataLen, int desDataLen);
    bool decompressionAfterprocessing(unsigned char *src, unsigned char * des, int srcDataLen, int desDataLen);
    bool setImageResolution(int width, int height);
    bool getImageResolution(int &width, int &height       );

private:
	
	int curResWidth;
	int curResHeight;
	int lowResWidth;
	int lowResHeight;
	int highResWidth;
	int highResHeight;

};
}

