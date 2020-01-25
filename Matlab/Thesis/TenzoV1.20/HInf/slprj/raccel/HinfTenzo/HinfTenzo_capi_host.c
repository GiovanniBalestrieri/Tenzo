#include "HinfTenzo_capi_host.h"
static HinfTenzo_host_DataMapInfo_T root;
static int initialized = 0;
rtwCAPI_ModelMappingInfo *getRootMappingInfo()
{
    if (initialized == 0) {
        initialized = 1;
        HinfTenzo_host_InitializeDataMapInfo(&(root), "HinfTenzo");
    }
    return &root.mmi;
}

rtwCAPI_ModelMappingInfo *mexFunction() {return(getRootMappingInfo());}
