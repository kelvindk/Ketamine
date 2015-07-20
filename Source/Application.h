#include "att.h"

bool writeTestPaperId(uint8* buf, uint8 length);
bool sendReadBuf(attHandleValueNoti_t* ptrNoti, uint8* buf, uint8 length, uint8 preamble);