#ifndef DEBUG_H
#define DEBUG_H

void initalizeDebug();

void debugImuStatuses(const ImuDevice& hand, const ImuDevice& device wrist);

void debugImuAttitudes(const ImuAttitude& hand, const ImuAttitude& wrist, const ImuAttitude& relative);

#endif
