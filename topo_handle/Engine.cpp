#include <iostream>
#include <omp.h>
#include "Engine.h"

void Address_Init(struct Engine *p_E) {
    if(p_E->m_LowTurbo.pFan != &p_E->m_Fan)
        p_E->m_LowTurbo.pFan = &p_E->m_Fan;
}
void Part_Init(struct Engine *p_E) {
    p_E->m_Fan.D_nFan = 11000.0;
    p_E->m_LowTurbo.D_n_LowTurbo = 12345.0;
}


void Engine_Init(struct Engine *p_E) {
    Part_Init(p_E);
    Address_Init(p_E);
    p_E->a = 100;
    p_E->b = 1;
    p_E->c = p_E->a * p_E->b;
}
// void EngineBegine(struct Engine *p_E);
void Engine_Go(struct Engine *p_E,struct EngineInput m_EngineInput) {
    Address_Init(p_E);
    p_E->a = m_EngineInput.Input_a;
    p_E->b = m_EngineInput.Input_b;
    p_E->c = p_E->a * p_E->b;
    // printf("p_E->a = %f ,p_E-b = %d ,p_E-c=%f\n ",p_E->a,p_E->b,p_E->c);
}