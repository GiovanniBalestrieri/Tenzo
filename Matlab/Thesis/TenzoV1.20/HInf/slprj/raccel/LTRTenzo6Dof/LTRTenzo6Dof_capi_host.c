#include "LTRTenzo6Dof_capi_host.h"
static LTRTenzo6Dof_host_DataMapInfo_T root;
static int initialized = 0;
rtwCAPI_ModelMappingInfo *getRootMappingInfo()
{
    if (initialized == 0) {
        initialized = 1;
        LTRTenzo6Dof_host_InitializeDataMapInfo(&(root), "LTRTenzo6Dof");
    }
    return &root.mmi;
}

rtwCAPI_ModelMappingInfo *mexFunction() {return(getRootMappingInfo());}
