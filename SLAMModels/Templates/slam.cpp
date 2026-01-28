#include <vector>
#include <mutex>
#include <memory>

#include "slam.h"

SLAMModule::SLAMModule() {

}

SLAMModule::~SLAMModule() {
    
}

std::vector<float>** SLAMModule::GetRenderMapAddress() {
    return &(this->renderMap);
}

std::shared_ptr<std::mutex> SLAMModule::GetRenderMapGuard() {
    return this->guardRenderMap;
}
