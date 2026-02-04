#include <iostream>
#include <vector>
#include <mutex>
#include <memory>

#include "slam.h"
#include "..\MapRepresentation\poseRenderPacket.h"

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

PoseRenderPacket* SLAMModule::GetPoses() {
    std::lock_guard<std::mutex> lock(this->guardPoses);
    PoseRenderPacket* packet = this->poses->Copy();
    return packet;
}

void SLAMModule::ReplacePoses(PoseRenderPacket* newPacket) {
    delete this->poses;
    this->poses = newPacket;
}