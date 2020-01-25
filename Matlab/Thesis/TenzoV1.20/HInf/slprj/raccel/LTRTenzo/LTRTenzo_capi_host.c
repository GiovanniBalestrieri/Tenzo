#include "LTRTenzo_capi_host.h"
static LTRTenzo_host_DataMapInfo_T root;
static int initialized = 0;
rtwCAPI_ModelMappingInfo *getRootMappingInfo()
{
    if (initialized == 0) {
        initialized = 1;
        LTRTenzo_host_InitializeDataMapInfo(&(root), "LTRTenzo");
    }
    return &root.mmi;
}

rtwCAPI_ModelMappingInfo *mexFunction() {return(getRootMappingInfo());}
