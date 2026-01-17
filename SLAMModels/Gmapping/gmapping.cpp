#include "gmapping.h"
#include "..\Templates\slam.h"
#include "particle.h"
#include "scanMatcher.h"

Gmapping::Gmapping() : SLAMModule() {
    this->scanMatcher = new ScanMatcher();
}

Gmapping::~Gmapping() {
    
}