#include <iostream>
#include <vector>
#include <unistd.h>
#include <omp.h>
#include "Engine.h"
using namespace std;

static omp_lock_t lock;

class MemTest{
public:
    double EOutPut;

private:
    size_t EngineSize = 0;
    struct Engine* m_Engine = nullptr;
    struct Engine* m_EngineBuf = nullptr;
    struct EngineInput a_EngineInput;
    double a_EngineOutput;

public:
    // Deep copy construct func
    MemTest(const MemTest& sm) {
        this->EngineSize = sm.EngineSize;
        this->m_Engine = new struct Engine(*(sm.m_Engine));
        Address_Init(this->m_Engine);
        this->m_EngineBuf = new struct Engine(*(sm.m_EngineBuf));
        Address_Init(this->m_EngineBuf);
        this->a_EngineInput = sm.a_EngineInput;
        this->a_EngineOutput = sm.a_EngineOutput;
        this->EOutPut = sm.EOutPut;
        printf("Deep Copy complete\n");

    }

    MemTest() {

    }

    ~MemTest() {
        if(m_Engine)
            delete m_Engine;
        m_Engine = nullptr;
        if(m_EngineBuf)
            delete m_EngineBuf;
        m_EngineBuf = nullptr;
    }


    bool Init() {
        if(m_Engine) {delete m_Engine; m_Engine = nullptr; }
        if(m_EngineBuf) {delete m_EngineBuf; m_EngineBuf = nullptr; }
        m_Engine = new struct Engine;
        printf("calloc m_Engine success\n");
        Engine_Init(m_Engine);

        return true;
    }


    void f_update(struct EngineInput& u) {
        if(m_Engine->m_LowTurbo.pFan == &m_Engine->m_Fan) {
            Engine_Go(m_Engine, u);
            sleep(1);
        }
        else
            printf("Error PtrAdress\n");
        a_EngineOutput = m_Engine->c;

        EOutPut = a_EngineOutput;
        // printf("Eoutput = %f",EOutPut);

    }

    void f_Noupdate(struct EngineInput u,int cnt) {

        if(m_EngineBuf) {
            delete(m_EngineBuf);
            m_EngineBuf = nullptr;
            //printf("------delete m_EngineBuf-----\n");
        }
        m_EngineBuf = new struct Engine(*m_Engine); // default copy construct func
        Address_Init(m_EngineBuf);

        // update
        for(int i = 0;i<cnt;i++)    f_update(u);

        if(m_Engine) {
            delete(m_Engine);
            m_Engine = nullptr;
            // printf("------delete m_Engine-----\n");
        }
        m_Engine = new struct Engine(*m_EngineBuf);
        Address_Init(m_Engine);

    }

    double hfun(struct EngineInput& x) {
        f_Noupdate(x,1);
        return EOutPut;
    }

};


int main() {
#ifndef _OPENMP
    printf("OpenMP not supported");
#endif

    cout << sizeof(double ) << endl;
    //omp_init_lock(&lock);

    MemTest *SM = new MemTest();
    SM->Init();
    double* MultiOutput = new double[1000];
    struct EngineInput MultiInput[6] = {{10,1},{12,2},{13,3},{14,4},{15,5},{16,6}};
    struct EngineInput MultiInput2[1000];
    omp_set_num_threads(6);
#pragma omp parallel for firstprivate(SM)
    for(int i = 0;i<1000;i++) {
        //omp_set_lock(&lock);
        printf(" i = %d , I am Thread %d , MultiOutPut_i = %f\n",i,omp_get_thread_num(),MultiOutput[i]);
        MemTest *tmpSM = new MemTest(SM);
        MultiOutput[i] = tmpSM->hfun(MultiInput2[i]);
        delete tmpSM;
        //omp_unset_lock(&lock);
    }
    //omp_destroy_lock(&lock);
    delete [] MultiOutput;
    return 0;

}