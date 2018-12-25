#ifndef __PLUGIN_LAYER_H__
#define __PLUGIN_LAYER_H__

#include <cassert>
#include <iostream>
#include <cudnn.h>
#include <cstring>
#include <cuda_runtime.h>
#include <cublas_v2.h>
#include <memory>
#include <chrono>

#include "NvCaffeParser.h"
#include "NvInferPlugin.h"

class Timer {
public:
    void tic() {
        start_ticking_ = true;
        start_ = std::chrono::high_resolution_clock::now();
    }   
    void toc() {
        if (!start_ticking_)return;
        end_ = std::chrono::high_resolution_clock::now();
        start_ticking_ = false;
        t = std::chrono::duration<double, std::milli>(end_ - start_).count();
        std::cout << "Time: " << t << " ms" << std::endl;
    }   
    double t;
private:
    bool start_ticking_ = false;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_;
    std::chrono::time_point<std::chrono::high_resolution_clock> end_;
};


void cudaSoftmax(int n, int channels,  float* x, float*y);

#define CHECK(status)                                                                                           \
    {                                                                                                                           \
        if (status != 0)                                                                                                \
        {                                                                                                                               \
            std::cout << "Cuda failure: " << cudaGetErrorString(status) \
                      << " at line " << __LINE__                                                        \
                      << std::endl;                                                                     \
            abort();                                                                                                    \
        }                                                                                                                               \
    }

using namespace nvinfer1;
using namespace nvcaffeparser1;
using namespace plugin;

enum FunctionType
{
    SELECT=0,
    SUMMARY
};

class bboxProfile {
public:
    bboxProfile(float4& p, int idx): pos(p), bboxNum(idx) {}

    float4 pos;
    int bboxNum = -1;
    int labelID = -1;
};

class tagProfile {
public:
    tagProfile(int b, int l): bboxID(b), label(l) {}
    int bboxID;
    int label;
};

//SSD Reshape layer : shape{0,-1,21}
template<int OutC>
class Reshape : public IPlugin
{
public:
    Reshape() {}
    Reshape(const void* buffer, size_t size)
    {
        assert(size == sizeof(mCopySize));
        mCopySize = *reinterpret_cast<const size_t*>(buffer);
    }

    int getNbOutputs() const override
    {
        return 1;
    }
    Dims getOutputDimensions(int index, const Dims* inputs, int nbInputDims) override
    {
        assert(nbInputDims == 1);
        assert(index == 0);
        assert(inputs[index].nbDims == 3);
        assert((inputs[0].d[0])*(inputs[0].d[1]) % OutC == 0);
        return DimsCHW( inputs[0].d[0] * inputs[0].d[1] / OutC,OutC, inputs[0].d[2]);
        //return DimsCHW(OutC, inputs[0].d[0] * inputs[0].d[1] / OutC, inputs[0].d[2]);
    }

    int initialize() override
    {
        return 0;
    }

    void terminate() override
    {
    }

    size_t getWorkspaceSize(int) const override
    {
        return 0;
    }

    // currently it is not possible for a plugin to execute "in place". Therefore we memcpy the data from the input to the output buffer
    int enqueue(int batchSize, const void*const *inputs, void** outputs, void*, cudaStream_t stream) override
    {
        CHECK(cudaMemcpyAsync(outputs[0], inputs[0], mCopySize * batchSize, cudaMemcpyDeviceToDevice, stream));
        return 0;
    }


    size_t getSerializationSize() override
    {
        return sizeof(mCopySize);
    }

    void serialize(void* buffer) override
    {
        *reinterpret_cast<size_t*>(buffer) = mCopySize;
    }

    void configure(const Dims*inputs, int nbInputs, const Dims* outputs, int nbOutputs, int)	override
    {
        mCopySize = inputs[0].d[0] * inputs[0].d[1] * inputs[0].d[2] * sizeof(float);
    }

protected:
    size_t mCopySize;
};




//Softmax layer.TensorRT softmax only support cross channel
class SoftmaxPlugin : public IPlugin
{
    //You need to implement it when softmax parameter axis is 2.
public:
    int initialize() override { return 0; }
    inline void terminate() override {}

    SoftmaxPlugin(){}
    SoftmaxPlugin( const void* buffer, size_t size)
    {
        assert(size == sizeof(mCopySize));
        mCopySize = *reinterpret_cast<const size_t*>(buffer);
    }
    inline int getNbOutputs() const override
    {
        //@TODO:  As the number of outputs are only 1, because there is only layer in top.
        return 1;
    }
    Dims getOutputDimensions(int index, const Dims* inputs, int nbInputDims) override
    {
        assert(nbInputDims == 1);
        assert(index == 0);
        assert(inputs[index].nbDims == 3);

        //确保输入的两个维度的积能被 类别数 整除  assert((inputs[0].d[0])*(inputs[0].d[1]) % OutC == 0);
        //printf("softmax inshape:%d,%d\n",(inputs[0].d[0])*(inputs[0].d[1])/2, 2);
        
        assert((inputs[0].d[0])*(inputs[0].d[1]) % 2== 0);

        // @TODO: Understood this.
        return DimsCHW( inputs[0].d[0] , inputs[0].d[1] , inputs[0].d[2] );
    }

    size_t getWorkspaceSize(int) const override
    {
        // @TODO: 1 is the batch size.
        return mCopySize*1;
    }

    int enqueue(int batchSize, const void*const *inputs, void** outputs, void* workspace, cudaStream_t stream) override
    {
        //std::cout<<"flatten enqueue:"<<batchSize<<";"<< mCopySize<<std::endl;
//        CHECK(cudaMemcpyAsync(outputs[0],inputs[0],batchSize*mCopySize*sizeof(float),cudaMemcpyDeviceToDevice,stream));

        //cudaSoftmax( 8732*21, 21, (float *) *inputs, static_cast<float *>(*outputs));
        //cudaSoftmax( 1917*5, 5, (float *) *inputs, static_cast<float *>(*outputs));
        cudaSoftmax( 4704*2, 2, (float *) *inputs, static_cast<float *>(*outputs));

        return 0;
    }

    size_t getSerializationSize() override
    {
        return sizeof(mCopySize);
    }
    void serialize(void* buffer) override
    {
        *reinterpret_cast<size_t*>(buffer) = mCopySize;
    }
    void configure(const Dims*inputs, int nbInputs, const Dims* outputs, int nbOutputs, int)	override
    {
        mCopySize = inputs[0].d[0] * inputs[0].d[1] * inputs[0].d[2] * sizeof(float);
    }

protected:
    size_t mCopySize;

};


//SSD Flatten layer

class FlattenLayer : public IPlugin
{
public:

    FlattenLayer(){}
    FlattenLayer(const void* buffer, size_t size)
    {
        assert(size == 3 * sizeof(int));
        const int* d = reinterpret_cast<const int*>(buffer);
        _size = d[0] * d[1] * d[2];
        dimBottom = DimsCHW{d[0], d[1], d[2]};
    }

    inline int getNbOutputs() const override { return 1; };
    Dims getOutputDimensions(int index, const Dims* inputs, int nbInputDims) override
    {
        assert(1 == nbInputDims);
        assert(0 == index);
        assert(3 == inputs[index].nbDims);
        _size = inputs[0].d[0] * inputs[0].d[1] * inputs[0].d[2];
        return DimsCHW(_size, 1, 1);
    }

    int initialize() override
    {
        return 0;
    }
    inline void terminate() override {}

    inline size_t getWorkspaceSize(int) const override { return 0; }

    int enqueue(int batchSize, const void*const *inputs, void** outputs, void*, cudaStream_t stream) override
    {
        //std::cout<<"flatten enqueue:"<<batchSize<<";"<<_size<<std::endl;
        CHECK(cudaMemcpyAsync(outputs[0],inputs[0],batchSize*_size*sizeof(float),cudaMemcpyDeviceToDevice,stream));
        return 0;
    }

    size_t getSerializationSize() override
    {
        return 3 * sizeof(int);
    }

    void serialize(void* buffer) override
    {
        int* d = reinterpret_cast<int*>(buffer);
        d[0] = dimBottom.c(); d[1] = dimBottom.h(); d[2] = dimBottom.w();
    }

    void configure(const Dims*inputs, int nbInputs, const Dims* outputs, int nbOutputs, int) override
    {
        dimBottom = DimsCHW(inputs[0].d[0], inputs[0].d[1], inputs[0].d[2]);
    }
protected:
    DimsCHW dimBottom;
    int _size;
};


//Concat layer . TensorRT Concat only support cross channel
class ConcatPlugin : public IPlugin
{
public:
    ConcatPlugin(int axis){ _axis = axis; };
    ConcatPlugin(int axis, const void* buffer, size_t size);

    inline int getNbOutputs() const override {return 1;};
    Dims getOutputDimensions(int index, const Dims* inputs, int nbInputDims) override ;
    int initialize() override;
    inline void terminate() override;

    inline size_t getWorkspaceSize(int) const override { return 0; };
    int enqueue(int batchSize, const void*const *inputs, void** outputs, void*, cudaStream_t stream) override;

    size_t getSerializationSize() override;
    void serialize(void* buffer) override;

    void configure(const Dims*inputs, int nbInputs, const Dims* outputs, int nbOutputs, int) override;

protected:
    DimsCHW dimsConv4_3, dimsFc7, dimsConv6, dimsConv7, dimsConv8, dimsConv9;
    int inputs_size;
    int top_concat_axis;//top 层 concat后的维度
    int* bottom_concat_axis = new int[9];//记录每个bottom层concat维度的shape
    int* concat_input_size_ = new int[9];
    int* num_concats_ = new int[9];
    int _axis;
};
class PluginFactory : public nvinfer1::IPluginFactory, public nvcaffeparser1::IPluginFactory
{
public:
    virtual nvinfer1::IPlugin* createPlugin(const char* layerName, const nvinfer1::Weights* weights, int nbWeights) override;
    IPlugin* createPlugin(const char* layerName, const void* serialData, size_t serialLength) override;

    void(*nvPluginDeleter)(INvPlugin*) { [](INvPlugin* ptr) {ptr->destroy(); } };

    bool isPlugin(const char* name) override;
    void destroyPlugin();
    //normalize layer
    //std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)> mNormalizeLayer{ nullptr, nvPluginDeleter };
    //permute layers
    std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)> mConv6_mbox_conf_perm_layer{ nullptr, nvPluginDeleter };
    std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)> mConv6_mbox_loc_perm_layer{ nullptr, nvPluginDeleter };
    std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)> mConv10_mbox_conf_perm_layer{ nullptr, nvPluginDeleter };
    std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)> mConv10_mbox_loc_perm_layer{ nullptr, nvPluginDeleter };
    std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)> mConv12_mbox_conf_perm_layer{ nullptr, nvPluginDeleter };
    std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)> mConv12_mbox_loc_perm_layer{ nullptr, nvPluginDeleter };

    //priorbox layers
    std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)> mConv6_mbox_priorbox_layer{ nullptr, nvPluginDeleter };
    std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)> mConv10_mbox_priorbox_layer{ nullptr, nvPluginDeleter };
    std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)> mConv12_mbox_priorbox_layer{ nullptr, nvPluginDeleter };

    //detection output layer
    std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)> mDetection_out{ nullptr, nvPluginDeleter };
    //concat layers
    std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)> mBox_loc_layer{ nullptr, nvPluginDeleter };
    std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)> mBox_conf_layer{ nullptr, nvPluginDeleter };
    std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)> mBox_priorbox_layer{ nullptr, nvPluginDeleter };
    //reshape layer
    std::unique_ptr<Reshape<2>> mMbox_conf_reshape{ nullptr };
    //flatten layers
    std::unique_ptr<FlattenLayer> mConv6_mbox_conf_flat_layer{ nullptr };
    std::unique_ptr<FlattenLayer> mConv10_mbox_conf_flat_layer{ nullptr };
    std::unique_ptr<FlattenLayer> mConv12_mbox_conf_flat_layer{ nullptr };
	
    std::unique_ptr<FlattenLayer> mConv6_mbox_loc_flat_layer{ nullptr };
    std::unique_ptr<FlattenLayer> mConv10_mbox_loc_flat_layer{ nullptr };
    std::unique_ptr<FlattenLayer> mConv12_mbox_loc_flat_layer{ nullptr };
	
    //softmax layer
    std::unique_ptr<SoftmaxPlugin> mPluginSoftmax{ nullptr };
    std::unique_ptr<FlattenLayer> mMbox_conf_flat_layer{ nullptr };


};

#endif
