


struct Fan {
    double D_nFan;
};
struct LowTurbo {
    struct Fan* pFan;
    double D_n_LowTurbo;
};

struct EngineInput {
    double Input_a;
    double Input_b;

};

struct Engine {
    double a;
    int b;
    double c;
    struct Fan m_Fan;
    struct LowTurbo m_LowTurbo;
    // TO Test firstprivate openmp
    // Need a User-defined copy construct func!!
};
void Address_Init(struct Engine *p_E);
void Part_Init(struct Engine *p_E);


void Engine_Init(struct Engine *p_E);
// void EngineBegine(struct Engine *p_E);
void Engine_Go(struct Engine *p_E,struct EngineInput m_EngineInput);