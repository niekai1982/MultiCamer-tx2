EXEC = yuanrun_test

CC = g++
STRIP = strip

SRCS = $(wildcard ./src/*.cpp)
OBJS = $(patsubst %.cpp, %.o, $(SRCS))

CFLAGS += -I ./include
CFLAGS += -I /usr/local/cuda-9.0/include
#CFLAGS += -I../MoveSenseCamera/
#CFLAGS += -I../MoveSenseCamera/cameraV4L/

#CFLAGS += -I./Include/opencv/
#CFLAGS += -I./Include/opencv2/
#CFLAGS += -I./Include/
#CFLAGS += -I./Include/rapidjson/
#CFLAGS += -I /usr/local/opencv-3.4.0/include/
#CFLAGS += -I./Include/lz4

CFLAGS += -g3 -O0 -Wall
CFLAGS += -std=gnu++11
LDFLAGS += -L /usr/lib/aarch64-linux-gnu/
LDFLAGS += -lnvcaffe_parser -lnvinfer -lnvinfer_plugin -lnvparsers
LDFLAGS += -L /usr/local/cuda-9.0/lib64/
LDFLAGS += -lcublas -lcudart
LDFLAGS += -lprotobuf -ldl -lpthread -lresolv

LDFLAGS += -L ./lib
#LDFLAGS += -linferLib
LDFLAGS += -ldecode -ldisplay -lMoveSenseDataProcessing -lprotobuf  -lresolv -lclient

#LDFLAGS += -L /usr/local/lib 

LDFLAGS += -lopencv_highgui -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_calib3d  -lopencv_dnn -lopencv_features2d -lopencv_flann    -lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_shape -lopencv_stitching -lopencv_superres -lopencv_video -lopencv_videoio -lopencv_videostab  -lpthread -lrt -ldl

all:$(EXEC)

$(EXEC):$(OBJS)
	$(CC) $(CFLAGS) -o $@ $(OBJS) ./lib/libinferLib.so $(LDFLAGS)
	$(STRIP) $(EXEC)
$(OBJS):%.o:%.cpp
	$(CC) $(CFLAGS) -c $< -o $@ 

clean:
	@rm -vf $(OBJS) $(EXEC) *.o *~




