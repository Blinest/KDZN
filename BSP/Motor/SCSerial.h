//
// Created by blin on 2026/5/2.
//

#ifndef CONTROLSYSTEM_SCSERIAL_H
#define CONTROLSYSTEM_SCSERIAL_H
int readSCS(unsigned char *nDat, int nLen);
int writeSCS(unsigned char *nDat, int nLen);
int writeByteSCS(unsigned char bDat);
void rFlushSCS();
void wFlushSCS();
#endif //CONTROLSYSTEM_SCSERIAL_H
