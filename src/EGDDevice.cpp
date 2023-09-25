#ifndef ROS2NEURO_ACQUISITION_PLUGIN_EEGDEV_CPP
#define ROS2NEURO_ACQUISITION_PLUGIN_EEGDEV_CPP

#include "ros2neuro_acquisition_eegdev/EGDDevice.hpp"

namespace rosneuro {

EGDDevice::EGDDevice(void) : Device() {
    this->name_ = "egddev";

    this->grp_     = nullptr;
    this->strides_ = nullptr;
}

EGDDevice::EGDDevice(NeuroFrame* frame) : Device(frame) {
    
    this->name_ = "egddev";

    this->grp_     = nullptr;
    this->strides_ = nullptr;

}

EGDDevice::~EGDDevice(void) {
    this->destroy_egd_structures();
}

bool EGDDevice::configure(NeuroFrame* frame, unsigned int framerate) {

    // declare ros parameters
    this->node_->declare_parameter("devarg", ""); 
    this->node_->declare_parameter("devext", ""); 
    this->node_->declare_parameter("samplerate", 0); 

    int samplerate;

    // Initialize the frame
    this->frame_     = frame;
    this->framerate_ = framerate;

    // Getting mandatory parameters from the server
    this->devarg_ = this->node_->get_parameter("devarg").as_string();
    if(this->devarg_.empty()){
        RCLCPP_ERROR(this->node_->get_logger(), "Missing 'devarg' in the server. 'devarg' is a mandatory parameter");
        return false;
    }

    // Getting optional parameters from the server
    this->devext_ = this->node_->get_parameter("devext").as_string();
    samplerate = this->node_->get_parameter("samplerate").as_int();
    this->samplerate_ = (unsigned int)samplerate;

    return true;
}

bool EGDDevice::setup(void) {

    // Setup structures
    if(this->setup_dev_capabilities() == false) {
        RCLCPP_ERROR(this->node_->get_logger(), "Cannot setup device capabilities"); 
        return false;
    }
    
    if(this->setup_neuro_data() == false) {
        RCLCPP_ERROR(this->node_->get_logger(), "Cannot setup data"); 
        return false;
    }
    
    if(this->setup_egd_structures() == false) {
        RCLCPP_ERROR(this->node_->get_logger(), "Cannot setup data structures"); 
        return false;
    }

    // Setup egd device
    if(egd_acq_setup(this->egddev_, EGD_DATA_GROUPS, this->strides_, EGD_DATA_GROUPS, this->grp_) == -1) {
        RCLCPP_ERROR(this->node_->get_logger(), "Cannot setup the device");
        return false;
    }
    
    RCLCPP_INFO(this->node_->get_logger(), "'%s' device correctly configured with samplerate=%d Hz", this->getName().c_str(), this->samplerate_);
    
    return true;
}

bool EGDDevice::open() {

    std::string devnamearg;
    bool isfile = false;

    if(this->devarg_.find(".bdf") != std::string::npos) {
        devnamearg.assign("datafile|path|");
        isfile = true;
    } else if(this->devarg_.find(".gdf") != std::string::npos) {
        devnamearg.assign("datafile|path|");
        isfile = true;
    }
    devnamearg.append(this->devarg_);

    if(isfile == false & this->samplerate_ > 0) {
        if(this->devarg_.compare("gtec") == 0) {
            devnamearg += "|samplerate|" + std::to_string(this->samplerate_);
        } else if(this->devarg_.compare("eego") == 0) {
            devnamearg += "|SR|" + std::to_string(this->samplerate_);
        }
    }
    
    devnamearg += this->devext_;

    if(!(this->egddev_ = egd_open(devnamearg.c_str()))) {
        RCLCPP_ERROR(this->node_->get_logger(), "Cannot open the '%s' device with arg=%s and samplerate=%d Hz", 
                  this->getName().c_str(), devnamearg.c_str(), this->samplerate_);
        return false;
    }

    RCLCPP_INFO(this->node_->get_logger(), "'%s' device correctly opened with arg=%s", 
             this->getName().c_str(), devnamearg.c_str());
    
    return true;
}

bool EGDDevice::close(void) {
    if(egd_close(this->egddev_) == -1) {
        std::cerr<<"[Error] - Cannot close the device '" << this->getName() <<"': "
                 << std::strerror(errno)<<std::endl;
        return false;
    }
    
    this->destroy_egd_structures();
    
    return true;
}
        
bool EGDDevice::start(void) {
    if(egd_start(this->egddev_) == -1) { 
        std::cerr<<"[Error] - Cannot start the device '" << this->getName() <<"': "
                 << std::strerror(errno) << std::endl;
        return false;
    }
    return true;
}

bool EGDDevice::stop(void) {
    if(egd_stop(this->egddev_) == -1) { 
        std::cerr<<"[Error] - Cannot stop the device '" << this->getName() << "': "
                 << std::strerror(errno) <<std::endl;
        return false;
    }
    return true;
}

size_t EGDDevice::get(void) {
    size_t size;

    size = egd_get_data(this->egddev_, this->frame_->eeg.nsamples(), 
                        (void*)this->frame_->eeg.data(), 
                        (void*)this->frame_->exg.data(), 
                        (void*)this->frame_->tri.data());
    if (size == -1) {
        std::cerr<<"Error reading data: " << std::strerror(errno) << std::endl;
    }

    return size;
}
        
size_t EGDDevice::getAvailable(void) {
    return egd_get_available(this->egddev_);
}





/*********************************************************************/
/*                    Private/protected methods                         */
/*********************************************************************/

void EGDDevice::init_dev_capabilities(void) {
    this->devinfo.model   = "";
    this->devinfo.id      = "";
}

void EGDDevice::init_egd_structures(void) {

    // Group structure
    this->grp_ = (grpconf*)malloc(EGD_DATA_GROUPS * sizeof(grpconf));
    memset(this->grp_, 0, EGD_DATA_GROUPS*sizeof(struct grpconf));
    
    // Strides array
    this->strides_ = (size_t*)malloc(EGD_DATA_GROUPS * sizeof(size_t));
    memset(this->strides_, 0, EGD_DATA_GROUPS*sizeof(size_t));
}


bool EGDDevice::setup_dev_capabilities(void) {

    char* model            = nullptr;
    char* id            = nullptr;

    // Initialize device capabilities structure
    this->init_dev_capabilities();
    
    // Getting devtype
    if(egd_get_cap(this->egddev_, EGD_CAP_DEVTYPE, &model) == -1) {
        std::cerr<<"[Error] - Cannot get device type: "<<strerror(errno)<<std::endl;
        return false;
    }

    // Getting devid
    if(egd_get_cap(this->egddev_, EGD_CAP_DEVID, &id) == -1) {
        std::cerr<<"[Error] - Cannot get device id: "<<strerror(errno)<<std::endl;
        return false;
    }


    // Populating device capabilities and data structure
    this->devinfo.model   = std::string(model);
    this->devinfo.id      = std::string(id);

    return true;
}

bool EGDDevice::setup_neuro_data(void) {

    size_t ns;
    size_t neeg, nexg, ntri;
    NeuroDataInfo *ieeg, *iexg, *itri;

    // Getting sampling rate and number of samples
    if(egd_get_cap(this->egddev_, EGD_CAP_FS, &this->samplerate_) == -1) {
        std::cerr<<"[Error] - Cannot get device sampling rate: "<<strerror(errno)<<std::endl;
        return false;
    }
    ns = (size_t)(this->samplerate_/this->framerate_);
    this->frame_->sr = this->samplerate_;

    // Getting EEG number of channels
    if( (neeg = egd_get_numch(this->egddev_, EGD_EEG)) == -1) {
        std::cerr<<"[Error] - Cannot get number EEG channels: "<<strerror(errno)<<std::endl;
        return false;
    }
    
    // Getting EXG number of channels
    if( (nexg = egd_get_numch(this->egddev_, EGD_SENSOR)) == -1) {
        std::cerr<<"[Error] - Cannot get number EXG channels: "<<strerror(errno)<<std::endl;
        return false;
    }

    // Getting TRI number of channels
    if( (ntri = egd_get_numch(this->egddev_, EGD_TRIGGER)) == -1) {
        std::cerr<<"[Error] - Cannot get number TRIGGER channels: "<<strerror(errno)<<std::endl;
        return false;
    }

    // Setup NeuroData groups
    this->frame_->eeg.reserve(ns, neeg);
    this->frame_->exg.reserve(ns, nexg);
    this->frame_->tri.reserve(ns, ntri);

    // Fill NeuroData Info
    this->setup_neuro_info(this->frame_->eeg.info(), this->frame_->eeg.nchannels(), EGD_EEG);
    this->setup_neuro_info(this->frame_->exg.info(), this->frame_->exg.nchannels(), EGD_SENSOR);
    this->setup_neuro_info(this->frame_->tri.info(), this->frame_->tri.nchannels(), EGD_TRIGGER);


    return true;
}

void EGDDevice::setup_neuro_info(NeuroDataInfo* info, size_t nch, unsigned int index) {

    char unit[16] = {0};
    char transducter[128] = {0};
    char filtering[128] = {0};
    char label[32] = {0};
    double mm[2] = {0.0f, 1.0f};
    int isint = 0;

    if(info == nullptr)
        return;

    egd_channel_info(this->egddev_, index, 0, EGD_UNIT, unit, 
                    EGD_TRANSDUCTER, transducter, EGD_PREFILTERING, filtering,
                    EGD_MM_D, mm, EGD_ISINT, &isint, EGD_EOL);
    info->unit            = std::string(unit);
    info->transducter     = std::string(transducter);
    info->prefiltering    = std::string(filtering);
    info->minmax[0]       = mm[0];
    info->minmax[1]       = mm[1];
    info->isint           = isint;

    info->labels.clear();

    for(auto i = 0; i<nch; i++) {
        egd_channel_info(this->egddev_, index, i,
                         EGD_LABEL, label, EGD_EOL);
        info->labels.push_back(std::string(label));
    }
}


bool EGDDevice::setup_egd_structures(void) {



    // Initialize eegdev structures
    this->init_egd_structures();
    
    // Configure 
    this->grp_[0].sensortype = EGD_EEG;
    this->grp_[0].index      = 0;
    this->grp_[0].iarray     = 0;
    this->grp_[0].datatype   = EGD_FLOAT;
    this->grp_[0].arr_offset = 0;
    this->grp_[0].nch        = this->frame_->eeg.nchannels();
    
    this->grp_[1].sensortype = EGD_SENSOR;
    this->grp_[1].index      = 0; 
    this->grp_[1].iarray     = 1; 
    this->grp_[1].datatype   = EGD_FLOAT;
    this->grp_[1].arr_offset = 0;
    this->grp_[1].nch        = this->frame_->exg.nchannels();
    
    this->grp_[2].sensortype = EGD_TRIGGER;
    this->grp_[2].index      = 0; 
    this->grp_[2].iarray     = 2;
    this->grp_[2].datatype   = EGD_INT32;
    this->grp_[2].arr_offset = 0;
    this->grp_[2].nch        = this->frame_->tri.nchannels();

    this->strides_[0] = this->frame_->eeg.stride();
    this->strides_[1] = this->frame_->exg.stride();
    this->strides_[2] = this->frame_->tri.stride();

    return true;
}



void EGDDevice::destroy_egd_structures(void) {
    
    if(this->grp_ != nullptr)
        free(this->grp_);
    this->grp_ = nullptr;

    if(this->strides_ != nullptr)
        free(this->strides_);
    this->strides_ = nullptr;
}


}

#endif
