#include <cstdio>
#include "VcuApp.h"
#include "VcuSelfTest.h"
static void sink(int i, const VcuTestStep& in, const VcuOutputs& out){
  printf("%d,%u,%u,%u,%u,%u,%.3f,%.3f,%d,%d,%d,%.3f,%d\n",
    i, in.apps, in.bps, in.fault, in.ageMs,
    out.appsADC, out.pedalPct, out.brakePct,
    out.bppcCut?1:0, out.canStale?1:0, out.invFault?1:0,
    out.torqueCmd, out.sendBrake?1:0);
}
int main(){
  printf("# selftest begin step,apps,bps,fault,age,appsADC,pedalPct,brakePct,bppc,stale,inv,torque,sendBrake\n");
  VcuApp t; vcuSelfTest(t, sink);
  printf("# selftest end\n");
}
