#include "pluginImplement.h"
#include "mathFunctions.h"
#include <vector>
#include <algorithm>

/******************************/
// PluginFactory
/******************************/
nvinfer1::IPlugin* PluginFactory::createPlugin(const char* layerName, const nvinfer1::Weights* weights, int nbWeights)
{
    assert(isPlugin(layerName));
    if (!strcmp(layerName, "conv6_mbox_loc_perm"))
    {
        assert(mConv6_mbox_loc_perm_layer.get() == nullptr);
        mConv6_mbox_loc_perm_layer = std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)>
                (createSSDPermutePlugin({{0, 2, 3, 1}}), nvPluginDeleter);
        return mConv6_mbox_loc_perm_layer.get();
    }
    else if (!strcmp(layerName, "conv6_mbox_conf_perm"))
    {
        assert(mConv6_mbox_conf_perm_layer.get() == nullptr);
        mConv6_mbox_conf_perm_layer = std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)>
                (createSSDPermutePlugin({{0, 2, 3, 1}}), nvPluginDeleter);
        return mConv6_mbox_conf_perm_layer.get();
    }
    else if (!strcmp(layerName, "conv10_mbox_loc_perm"))
    {
        assert(mConv10_mbox_loc_perm_layer.get() == nullptr);
        mConv10_mbox_loc_perm_layer = std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)>
                (createSSDPermutePlugin({{0, 2, 3, 1}}), nvPluginDeleter);
        return mConv10_mbox_loc_perm_layer.get();
    }
    else if (!strcmp(layerName, "conv10_mbox_conf_perm"))
    {
        assert(mConv10_mbox_conf_perm_layer.get() == nullptr);
        mConv10_mbox_conf_perm_layer = std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)>
                (createSSDPermutePlugin({{0, 2, 3, 1}}), nvPluginDeleter);
        return mConv10_mbox_conf_perm_layer.get();
    }
    else if (!strcmp(layerName, "conv12_mbox_loc_perm"))
    {
        assert(mConv12_mbox_loc_perm_layer.get() == nullptr);
        mConv12_mbox_loc_perm_layer = std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)>
                (createSSDPermutePlugin({{0, 2, 3, 1}}), nvPluginDeleter);
        return mConv12_mbox_loc_perm_layer.get();
    }
    else if (!strcmp(layerName, "conv12_mbox_conf_perm"))
    {
        assert(mConv12_mbox_conf_perm_layer.get() == nullptr);
        mConv12_mbox_conf_perm_layer = std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)>
                (createSSDPermutePlugin({{0, 2, 3, 1}}), nvPluginDeleter);
        return mConv12_mbox_conf_perm_layer.get();
    }
    else if (!strcmp(layerName, "conv6_mbox_priorbox"))
    {
        assert(mConv6_mbox_priorbox_layer.get() == nullptr);
        //参数按照原来的prototxt中的prior_box_param设置
        PriorBoxParameters params;
        float minsize[1] = {22.4},maxsize[1] = {31.7},aspect_ratio[3] = {1.0,0.75,0.5};
        params.minSize = minsize;
        params.numMinSize = 1;
        params.maxSize = maxsize;
        params.numMaxSize = 1;
        params.aspectRatios = aspect_ratio;
        params.numAspectRatios = 3;
        params.flip = false;
        params.clip = false;
		
        params.variance[0] = 0.1;
        params.variance[1] = 0.1;
        params.variance[2] = 0.2;
        params.variance[3] = 0.2;
        params.imgH = 0;
        params.imgW = 0;
        params.stepH = 0;
        params.stepW = 0;
        params.offset = 0.5;
        mConv6_mbox_priorbox_layer = std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)>
                (createSSDPriorBoxPlugin(params), nvPluginDeleter);
        return mConv6_mbox_priorbox_layer.get();
    }
    else if (!strcmp(layerName, "conv10_mbox_priorbox"))
    {
        assert(mConv10_mbox_priorbox_layer.get() == nullptr);
        PriorBoxParameters params;
        float minsize[1] = {44.8}, maxsize[1] = {83.8}, aspect_ratio[3] = {1.0,0.75,0.5};
        params.minSize = minsize;
        params.numMinSize = 1;
        params.maxSize = maxsize;
        params.numMaxSize = 1;
        params.aspectRatios = aspect_ratio;
        params.numAspectRatios = 3;

        params.flip = false;
        params.clip = false;
        params.variance[0] = 0.1;
        params.variance[1] = 0.1;
        params.variance[2] = 0.2;
        params.variance[3] = 0.2;
        params.imgH = 0;
        params.imgW = 0;
        params.stepH = 0;
        params.stepW = 0;
        params.offset = 0.5;
        mConv10_mbox_priorbox_layer = std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)>
                (createSSDPriorBoxPlugin(params), nvPluginDeleter);
        return mConv10_mbox_priorbox_layer.get();
    }
    else if (!strcmp(layerName, "conv12_mbox_priorbox"))
    {
        assert(mConv12_mbox_priorbox_layer.get() == nullptr);
        PriorBoxParameters params;
        float minsize[1] = {156.8}, maxsize[1] = {205.3}, aspect_ratio[3] = {1.0,0.75,0.5};
        params.minSize = minsize;
        params.numMinSize = 1;
        params.maxSize = maxsize;
        params.numMaxSize = 1;
        params.aspectRatios = aspect_ratio;
        params.numAspectRatios = 3;

        params.flip = false;
        params.clip = false;
        params.variance[0] = 0.1;
        params.variance[1] = 0.1;
        params.variance[2] = 0.2;
        params.variance[3] = 0.2;
        params.imgH = 0;
        params.imgW = 0;
        params.stepH = 0;
        params.stepW = 0;
        params.offset = 0.5;
        mConv12_mbox_priorbox_layer = std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)>
                (createSSDPriorBoxPlugin(params), nvPluginDeleter);
        return mConv12_mbox_priorbox_layer.get();
    }
	
    else if (!strcmp(layerName, "mbox_loc"))
    {
        assert(mBox_loc_layer.get() == nullptr);
        mBox_loc_layer = std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)>
                (createConcatPlugin(1, true), nvPluginDeleter);
        return mBox_loc_layer.get();
    }
    else if (!strcmp(layerName, "mbox_conf"))
    {
        assert(mBox_conf_layer.get() == nullptr);
        mBox_conf_layer = std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)>
                (createConcatPlugin(1, true), nvPluginDeleter);
        return mBox_conf_layer.get();
    }
    else if (!strcmp(layerName, "mbox_priorbox"))
    {
        assert(mBox_priorbox_layer.get() == nullptr);
        mBox_priorbox_layer = std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)>
                (createConcatPlugin(2, true), nvPluginDeleter);
        return mBox_priorbox_layer.get();
    }
    //flatten
    else if (!strcmp(layerName, "conv6_mbox_conf_flat"))
    {
        assert(mConv6_mbox_conf_flat_layer.get() == nullptr);
        mConv6_mbox_conf_flat_layer = std::unique_ptr<FlattenLayer>(new FlattenLayer());
        return mConv6_mbox_conf_flat_layer.get();
    }
    else if (!strcmp(layerName, "conv10_mbox_conf_flat"))
    {
        assert(mConv10_mbox_conf_flat_layer.get() == nullptr);
        mConv10_mbox_conf_flat_layer = std::unique_ptr<FlattenLayer>(new FlattenLayer());
        return mConv10_mbox_conf_flat_layer.get();
    }
    else if (!strcmp(layerName, "conv12_mbox_conf_flat"))
    {
        assert(mConv12_mbox_conf_flat_layer.get() == nullptr);
        mConv12_mbox_conf_flat_layer = std::unique_ptr<FlattenLayer>(new FlattenLayer());
        return mConv12_mbox_conf_flat_layer.get();
    }
 
    else if (!strcmp(layerName, "conv6_mbox_loc_flat"))
    {
        assert(mConv6_mbox_loc_flat_layer.get() == nullptr);
        mConv6_mbox_loc_flat_layer = std::unique_ptr<FlattenLayer>(new FlattenLayer());
        return mConv6_mbox_loc_flat_layer.get();
    }
    else if (!strcmp(layerName, "conv10_mbox_loc_flat"))
    {
        assert(mConv10_mbox_loc_flat_layer.get() == nullptr);
        mConv10_mbox_loc_flat_layer = std::unique_ptr<FlattenLayer>(new FlattenLayer());
        return mConv10_mbox_loc_flat_layer.get();
    }
    else if (!strcmp(layerName, "conv12_mbox_loc_flat"))
    {
        assert(mConv12_mbox_loc_flat_layer.get() == nullptr);
        mConv12_mbox_loc_flat_layer = std::unique_ptr<FlattenLayer>(new FlattenLayer());
        return mConv12_mbox_loc_flat_layer.get();
    }

    else if (!strcmp(layerName, "mbox_conf_flatten"))
    {
        assert(mMbox_conf_flat_layer.get() == nullptr);
        mMbox_conf_flat_layer = std::unique_ptr<FlattenLayer>(new FlattenLayer());
        return mMbox_conf_flat_layer.get();
    }

    //reshape
    else if (!strcmp(layerName, "mbox_conf_reshape"))
    {
        assert(mMbox_conf_reshape.get() == nullptr);
        assert(nbWeights == 0 && weights == nullptr);
        mMbox_conf_reshape = std::unique_ptr<Reshape<2>>(new Reshape<2>());
        return mMbox_conf_reshape.get();
    }
    //softmax layer
    else if (!strcmp(layerName, "mbox_conf_softmax"))
    {
        assert(mPluginSoftmax == nullptr);
        assert(nbWeights == 0 && weights == nullptr);
        mPluginSoftmax = std::unique_ptr<SoftmaxPlugin>(new SoftmaxPlugin());
        return mPluginSoftmax.get();
    }
    else if (!strcmp(layerName, "detection_out"))
    {
        assert(mDetection_out.get() == nullptr);
        DetectionOutputParameters params;

        params.backgroundLabelId = 0;
        params.codeType = CodeTypeSSD::CENTER_SIZE;
        //params.codeType = CodeType_t::CENTER_SIZE;
        params.keepTopK = 100;
        params.shareLocation = true;
        params.varianceEncodedInTarget = false;
        params.topK = 100;
        params.nmsThreshold = 0.5;
        params.numClasses = 2;
        params.inputOrder[0] = 0;
        params.inputOrder[1] = 1;
        params.inputOrder[2] = 2;
        params.confidenceThreshold = 0.5;
	params.confSigmoid = false;
	params.isNormalized = true;

        mDetection_out = std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)>
                (createSSDDetectionOutputPlugin(params), nvPluginDeleter);

        return mDetection_out.get();
    }

    else
    {
        std::cout << "not found  " << layerName << std::endl;
        assert(0);
        return nullptr;
    }
}

IPlugin* PluginFactory::createPlugin(const char* layerName, const void* serialData, size_t serialLength)
{
    assert(isPlugin(layerName));
    if (!strcmp(layerName, "conv6_mbox_loc_perm"))
    {
        assert(mConv6_mbox_loc_perm_layer.get() == nullptr);
        mConv6_mbox_loc_perm_layer = std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)>
                (createSSDPermutePlugin(serialData,serialLength), nvPluginDeleter);
        return mConv6_mbox_loc_perm_layer.get();
    }
    else if (!strcmp(layerName, "conv6_mbox_conf_perm"))
    {
        assert(mConv6_mbox_conf_perm_layer.get() == nullptr);
        mConv6_mbox_conf_perm_layer = std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)>
                (createSSDPermutePlugin(serialData,serialLength), nvPluginDeleter);
        return mConv6_mbox_conf_perm_layer.get();
    }
    else if (!strcmp(layerName, "conv10_mbox_loc_perm"))
    {
        assert(mConv10_mbox_loc_perm_layer.get() == nullptr);
        mConv10_mbox_loc_perm_layer = std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)>
                (createSSDPermutePlugin(serialData,serialLength), nvPluginDeleter);
        return mConv10_mbox_loc_perm_layer.get();
    }
    else if (!strcmp(layerName, "conv10_mbox_conf_perm"))
    {
        assert(mConv10_mbox_conf_perm_layer.get() == nullptr);
        mConv10_mbox_conf_perm_layer = std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)>
                (createSSDPermutePlugin(serialData,serialLength), nvPluginDeleter);
        return mConv10_mbox_conf_perm_layer.get();
    }
    else if (!strcmp(layerName, "conv12_mbox_loc_perm"))
    {
        assert(mConv12_mbox_loc_perm_layer.get() == nullptr);
        mConv12_mbox_loc_perm_layer = std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)>
                (createSSDPermutePlugin(serialData,serialLength), nvPluginDeleter);
        return mConv12_mbox_loc_perm_layer.get();
    }
    else if (!strcmp(layerName, "conv12_mbox_conf_perm"))
    {
        assert(mConv12_mbox_conf_perm_layer.get() == nullptr);
        mConv12_mbox_conf_perm_layer = std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)>
                (createSSDPermutePlugin(serialData,serialLength), nvPluginDeleter);
        return mConv12_mbox_conf_perm_layer.get();
    }

    else if (!strcmp(layerName, "conv6_mbox_priorbox"))
    {
        assert(mConv6_mbox_priorbox_layer.get() == nullptr);

        mConv6_mbox_priorbox_layer = std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)>
                (createSSDPriorBoxPlugin(serialData,serialLength), nvPluginDeleter);
        return mConv6_mbox_priorbox_layer.get();
    }
    else if (!strcmp(layerName, "conv10_mbox_priorbox"))
    {
        assert(mConv10_mbox_priorbox_layer.get() == nullptr);
        mConv10_mbox_priorbox_layer = std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)>
                (createSSDPriorBoxPlugin(serialData,serialLength), nvPluginDeleter);
        return mConv10_mbox_priorbox_layer.get();
    }
    else if (!strcmp(layerName, "conv12_mbox_priorbox"))
    {
        assert(mConv12_mbox_priorbox_layer.get() == nullptr);
        mConv12_mbox_priorbox_layer = std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)>
                (createSSDPriorBoxPlugin(serialData,serialLength), nvPluginDeleter);
        return mConv12_mbox_priorbox_layer.get();
    }

    else if (!strcmp(layerName, "mbox_loc"))
    {
        assert(mBox_loc_layer.get() == nullptr);
        mBox_loc_layer = std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)>
                (createConcatPlugin(serialData,serialLength), nvPluginDeleter);
        return mBox_loc_layer.get();
    }
    else if (!strcmp(layerName, "mbox_conf"))
    {
        assert(mBox_conf_layer.get() == nullptr);
        mBox_conf_layer = std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)>
                (createConcatPlugin(serialData,serialLength), nvPluginDeleter);
        return mBox_conf_layer.get();
    }
    else if (!strcmp(layerName, "mbox_priorbox"))
    {
        assert(mBox_priorbox_layer.get() == nullptr);
        mBox_priorbox_layer = std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)>
                (createConcatPlugin(serialData,serialLength), nvPluginDeleter);
        return mBox_priorbox_layer.get();
    }
        //flatten
    else if (!strcmp(layerName, "conv6_mbox_conf_flat"))
    {
        assert(mConv6_mbox_conf_flat_layer.get() == nullptr);
        mConv6_mbox_conf_flat_layer = std::unique_ptr<FlattenLayer>(new FlattenLayer(serialData,serialLength));
        return mConv6_mbox_conf_flat_layer.get();
    }
    else if (!strcmp(layerName, "conv10_mbox_conf_flat"))
    {
        assert(mConv10_mbox_conf_flat_layer.get() == nullptr);
        mConv10_mbox_conf_flat_layer = std::unique_ptr<FlattenLayer>(new FlattenLayer(serialData,serialLength));
        return mConv10_mbox_conf_flat_layer.get();
    }
    else if (!strcmp(layerName, "conv12_mbox_conf_flat"))
    {
        assert(mConv12_mbox_conf_flat_layer.get() == nullptr);
        mConv12_mbox_conf_flat_layer = std::unique_ptr<FlattenLayer>(new FlattenLayer(serialData,serialLength));
        return mConv12_mbox_conf_flat_layer.get();
    }

    else if (!strcmp(layerName, "conv6_mbox_loc_flat"))
    {
        assert(mConv6_mbox_loc_flat_layer.get() == nullptr);
        mConv6_mbox_loc_flat_layer = std::unique_ptr<FlattenLayer>(new FlattenLayer(serialData,serialLength));
        return mConv6_mbox_loc_flat_layer.get();
    }
    else if (!strcmp(layerName, "conv10_mbox_loc_flat"))
    {
        assert(mConv10_mbox_loc_flat_layer.get() == nullptr);
        mConv10_mbox_loc_flat_layer = std::unique_ptr<FlattenLayer>(new FlattenLayer(serialData,serialLength));
        return mConv10_mbox_loc_flat_layer.get();
    }
    else if (!strcmp(layerName, "conv12_mbox_loc_flat"))
    {
        assert(mConv12_mbox_loc_flat_layer.get() == nullptr);
        mConv12_mbox_loc_flat_layer = std::unique_ptr<FlattenLayer>(new FlattenLayer(serialData,serialLength));
        return mConv12_mbox_loc_flat_layer.get();
    }
 
    else if (!strcmp(layerName, "mbox_conf_flatten"))
    {
        assert(mMbox_conf_flat_layer.get() == nullptr);
        mMbox_conf_flat_layer = std::unique_ptr<FlattenLayer>(new FlattenLayer(serialData,serialLength));
        return mMbox_conf_flat_layer.get();
    }
    //reshape
    else if (!strcmp(layerName, "mbox_conf_reshape"))
    {
        assert(mMbox_conf_reshape == nullptr);
        //num of class,by lcg
        mMbox_conf_reshape = std::unique_ptr<Reshape<2>>(new Reshape<2>(serialData, serialLength));
        return mMbox_conf_reshape.get();
    }
    //softmax
    else if (!strcmp(layerName, "mbox_conf_softmax"))
    {
        std::cout << "2_softmax" << std::endl;
        assert(mPluginSoftmax == nullptr);
        mPluginSoftmax = std::unique_ptr<SoftmaxPlugin>(new SoftmaxPlugin(serialData, serialLength));
        return mPluginSoftmax.get();
    }

    else if (!strcmp(layerName, "detection_out"))
    {
        std::cout << "2_detection_out" << std::endl;
        assert(mDetection_out.get() == nullptr);
        mDetection_out = std::unique_ptr<INvPlugin, decltype(nvPluginDeleter)>(createSSDDetectionOutputPlugin(serialData, serialLength), nvPluginDeleter);
        return mDetection_out.get();
    }
    else
    {
        std::cout << "else" << std::endl;
        assert(0);
        return nullptr;
    }
}

bool PluginFactory::isPlugin(const char* name)
{
    return (!strcmp(name, "conv6_mbox_loc_perm")
            || !strcmp(name, "conv6_mbox_loc_flat")
            || !strcmp(name, "conv6_mbox_conf_perm")
            || !strcmp(name, "conv6_mbox_conf_flat")
            || !strcmp(name, "conv6_mbox_priorbox")
			
            || !strcmp(name, "conv10_mbox_loc_perm")
            || !strcmp(name, "conv10_mbox_loc_flat")
            || !strcmp(name, "conv10_mbox_conf_perm")
            || !strcmp(name, "conv10_mbox_conf_flat")
            || !strcmp(name, "conv10_mbox_priorbox")
			
            || !strcmp(name, "conv12_mbox_loc_perm")
            || !strcmp(name, "conv12_mbox_loc_flat")
            || !strcmp(name, "conv12_mbox_conf_perm")
            || !strcmp(name, "conv12_mbox_conf_flat")
            || !strcmp(name, "conv12_mbox_priorbox")
			
            || !strcmp(name, "mbox_conf_reshape")
            || !strcmp(name, "mbox_conf_flatten")
            || !strcmp(name, "mbox_loc")
            || !strcmp(name, "mbox_conf")
            || !strcmp(name, "mbox_priorbox")
            || !strcmp(name, "mbox_conf_softmax")
            || !strcmp(name, "detection_out"));
}

void PluginFactory::destroyPlugin()
{
    std::cout << "distroyPlugin" << std::endl;
    //mNormalizeLayer.release();
    //mNormalizeLayer = nullptr;

    mConv6_mbox_conf_perm_layer.release();
    mConv6_mbox_conf_perm_layer = nullptr;
    mConv6_mbox_loc_perm_layer.release();
    mConv6_mbox_loc_perm_layer = nullptr;
	
    mConv10_mbox_conf_perm_layer.release();
    mConv10_mbox_conf_perm_layer = nullptr;
    mConv10_mbox_loc_perm_layer.release();
    mConv10_mbox_loc_perm_layer = nullptr;
	
    mConv12_mbox_conf_perm_layer.release();
    mConv12_mbox_conf_perm_layer = nullptr;
    mConv12_mbox_loc_perm_layer.release();
    mConv12_mbox_loc_perm_layer = nullptr;

    mConv6_mbox_priorbox_layer.release();
    mConv6_mbox_priorbox_layer = nullptr;
    mConv10_mbox_priorbox_layer.release();
    mConv10_mbox_priorbox_layer = nullptr;
    mConv12_mbox_priorbox_layer.release();
    mConv12_mbox_priorbox_layer = nullptr;

    mBox_loc_layer.release();
    mBox_loc_layer = nullptr;
    mBox_conf_layer.release();
    mBox_conf_layer = nullptr;
    mBox_priorbox_layer.release();
    mBox_priorbox_layer = nullptr;

    mConv6_mbox_conf_flat_layer.release();
    mConv6_mbox_conf_flat_layer = nullptr;
    mConv10_mbox_conf_flat_layer.release();
    mConv10_mbox_conf_flat_layer = nullptr;
    mConv12_mbox_conf_flat_layer.release();
    mConv12_mbox_conf_flat_layer = nullptr;
	
    mConv6_mbox_loc_flat_layer.release();
    mConv6_mbox_loc_flat_layer = nullptr;
    mConv10_mbox_loc_flat_layer.release();
    mConv10_mbox_loc_flat_layer = nullptr;
    mConv12_mbox_loc_flat_layer.release();
    mConv12_mbox_loc_flat_layer = nullptr;

    mMbox_conf_flat_layer.release();
    mMbox_conf_flat_layer = nullptr;

    mMbox_conf_reshape.release();
    mMbox_conf_reshape = nullptr;
    mPluginSoftmax.release();
    mPluginSoftmax = nullptr;
    mDetection_out.release();
    mDetection_out = nullptr;

}
