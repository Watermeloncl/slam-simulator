#include "ekf.h"
#include "..\Templates\slam.h"
#include "landmarkExtractor.h"

EKF::EKF() : SLAMModule() {
    this->landmarkExtractor = new LandmarkExtractor();
}

EKF::~EKF() {

}
