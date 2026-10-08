#pragma once
// Solo per i test PC: esegue il blocco una volta, senza simulare interrupt.
// Su Nano Every si usa invece util/atomic.h della toolchain AVR.
#define ATOMIC_RESTORESTATE 0
#define ATOMIC_BLOCK(type) for (bool atomicOnce = true; atomicOnce; atomicOnce = false)
