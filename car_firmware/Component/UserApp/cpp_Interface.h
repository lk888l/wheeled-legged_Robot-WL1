#ifndef WL1_CPP_INTERFACE_H
#define WL1_CPP_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

// Called once by AppBootstrap after the FreeRTOS scheduler has started.
void CPP_Main(void);

#ifdef __cplusplus
}
#endif

#endif // WL1_CPP_INTERFACE_H
