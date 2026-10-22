/* Host tests exercise the actual RL rejection branches without hardware. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "../../Identification/Rs_Ls.c"

Motor_Run_T Motor_Run;
Motor_Config_T Motor_Config;
Ident_Envelope_T Ident_Envelope;
PID_T Id_Ctrl, Iq_Ctrl;
float Control_Current_Bw_Hz = 1250;
volatile uint8_t Ident_Fail_Reason;
static unsigned Reports;
static float Out[5];

void Current_Tuning_Update(void) {}
void Current_Loop_State_Reset(void) {}
void Identification_Report(bool Valid) { (void)Valid; Reports++; }

static void Setup(float Hz)
{
    Rs_Ls_Stop();
    Motor_Config.RL_I_Peak_A = 1;
    Ident_Envelope = (Ident_Envelope_T){ .I_Max = 2, .U_Max = 12 };
    Motor_Run.Ud = .25f;
    Ident_Fail_Reason = IDENT_FAIL_NONE;
    Reports = 0;
    Rs_Ls_Start();
    Frequency_Set(Hz);
}

static int Run(void)
{
    return Rs_Ls_Run(0, Out, Out + 1, Out + 2, Out + 3, Out + 4);
}

static void Window(Rs_Ls_State_e Stage, float Rs, float Ls, float Amp)
{
    unsigned Total = Sample_Per_Cycle * ((Stage == RS_LS_PROBE_MEASURE) ? RS_LS_PROBE_MEASURE_CYCLE : RS_LS_MEASURE_CYCLE);
    unsigned N = Total - ((Stage == RS_LS_PROBE_MEASURE) ? 0 : Sample_Per_Cycle * RS_LS_MEASURE_SKIP_CYCLE);
    State = Stage;
    Phase = 0;
    U_DC = .3f;
    U_Hold = .3f;
    U_AC = .1f;
    Cnt = Total - 1;
    I_DC_Sum = N * .7f;
    I_Re = N * Amp * .5f;
    I_Im = 0;
    U_Re = Rs * I_Re - .3f;
    U_Im = TWO_PI_F * (CUR_FREQ_HZ_DEFAULT / Sample_Per_Cycle) * Ls * I_Re;
    Ramp_Cnt = 0;
    Ramp_I_Re = Ramp_I_Im = 0;
    Coh_Cnt = RS_LS_PROBE_MEASURE_CYCLE;
    Coh_I_Pow = RS_LS_PROBE_MEASURE_CYCLE * Amp * Amp;
    I_Noise = .001f;
}

static void Failed(void)
{
    assert(Reports == 1);
    assert(!Rs_Ls_Result.Valid && Out[3] == 0);
    Rs_Ls_Stop();
    Rs_Ls_Start();
}

int main(void)
{
    /* Complete alignment first, then average five periods of completed Ud.
     * A different voltage in each period detects an early exit or wrong divisor. */
    const float Hold_Hz[] = {20, 150, 500};
    for (unsigned H = 0; H < sizeof(Hold_Hz) / sizeof(Hold_Hz[0]); H++)
    {
        Setup(Hold_Hz[H]); State = RS_LS_ALIGN; RL_Temporary = true;
        Rs_Ls_Align.State.Cnt = Rs_Ls_Align.Para.Hold_Cnt - 1;
        Cnt = 99; U_Hold = 99;
        assert(Run() == FAST_CURRENT && Cnt == 0 && U_Hold == 0);
        unsigned Hold_N = Sample_Per_Cycle * 5;
        for (unsigned K = 0; K < Hold_N; K++)
        {
            Motor_Run.Ud = .1f * (float)(K / Sample_Per_Cycle + 1);
            int Mode = Run();
            if (K + 1 < Hold_N)
            {
                assert(Mode == FAST_CURRENT && State == RS_LS_ALIGN && RL_Temporary);
                assert(Out[1] == .65f && Out[2] == 0 && Cnt == K + 1);
            }
            else
            {
                assert(Mode == FAST_VOLTAGE && State == RS_LS_RAMP && !RL_Temporary);
                assert(fabsf(U_Hold - .3f) < .0001f && fabsf(Out[3] - .3f) < .0001f);
            }
        }
    }

    /* Noise lasts exactly 30 cycles, including each fresh retry measurement. */
    Setup(150);
    unsigned Noise_N = Sample_Per_Cycle * 30;
    for (unsigned K = 0; K < Noise_N; K++)
    {
        assert(State == RS_LS_NOISE && Run() == FAST_VOLTAGE);
    }
    assert(State == RS_LS_PROBE_RAMP && I_Noise == 0);

    /* Both ramps hold voltage for two full cycles; their DC/AC normalization
     * must not mistake a sub-target AC current for twice its true amplitude. */
    for (unsigned S = 0; S < 2; S++)
    {
        Setup(150); State = S ? RS_LS_RAMP : RS_LS_PROBE_RAMP;
        U_Hold = U_DC = .3f; U_AC = .1f;
        unsigned Ramp_N = Sample_Per_Cycle * 2;
        for (unsigned K = 0; K < Ramp_N; K++)
        {
            float I = .7f + .2f * sinf(Phase);
            assert(Rs_Ls_Run(I, Out, Out + 1, Out + 2, Out + 3, Out + 4) == FAST_VOLTAGE);
            if (K + 1 < Ramp_N) assert(U_AC == .1f);
        }
        assert(U_AC > .1f && U_AC <= .120001f && Ramp_Cnt == 0);
        assert(State == (S ? RS_LS_RAMP : RS_LS_PROBE_RAMP));
        if (!S) assert(U_DC == .3f);
    }

    /* Discarded samples must never reach any integral, even if very large.
     * Check all three windows and the exact first accepted sample. */
    for (unsigned S = RS_LS_MEASURE_A; S <= RS_LS_MEASURE_C; S++)
    {
        Setup(150); State = (Rs_Ls_State_e)S; U_Hold = .3f; U_AC = .1f;
        unsigned Skip_N = Sample_Per_Cycle * 5;
        for (unsigned K = 0; K < Skip_N; K++)
        {
            assert(Rs_Ls_Run(100, Out, Out + 1, Out + 2, Out + 3, Out + 4) == FAST_VOLTAGE);
            assert(U_Re == 0 && U_Im == 0 && I_Re == 0 && I_Im == 0);
        }
        assert(Cnt == Skip_N);
        assert(Run() == FAST_VOLTAGE && Cnt == Skip_N + 1);
        assert(U_Re != 0 || U_Im != 0);
    }

    Setup(150); Window(RS_LS_MEASURE_A, .3f, .00008f, .4f);
    assert(Run() == FAST_VOLTAGE && State == RS_LS_RAMP);
    assert(Out[3] == 0 && Freq_Refined);
    assert(fabsf(Rs_A - .3f) < .00001f && fabsf(Ls_A - .00008f) < 1e-8f);
    Window(RS_LS_MEASURE_A, .3f, .00008f, .4f);
    assert(Run() == FAST_VOLTAGE && State == RS_LS_MEASURE_B);
    Window(RS_LS_MEASURE_B, .3f, .00008f, .4f);
    assert(Run() == FAST_OFF && Reports == 1 && Rs_Ls_Result.Valid);
    assert(fabsf(Rs_Ls_Result.Rs_Ohm - .3f) < .00001f);
    assert(fabsf(Rs_Ls_Result.Ls_H - .00008f) < 1e-8f);

    Setup(150); Ident_Envelope.U_Max = 0;
    assert(Run() == FAST_OFF); Failed();
    Setup(150); Motor_Config.RL_I_Peak_A = 0;
    assert(Run() == FAST_OFF); Failed();
    Setup(150); State = RS_LS_RAMP; U_Hold = 12;
    assert(Run() == FAST_OFF); Failed();

    for(unsigned S = 0; S < 3; S++)
    {
        Setup(150);
        State = S == 0 ? RS_LS_PROBE_RAMP : (S == 1 ? RS_LS_RAMP : RS_LS_PROBE_MEASURE);
        U_DC = 12; U_AC = S == 1 ? 12 : 0; U_Hold = 0;
        Ramp_Cnt = Sample_Per_Cycle * (S == 2 ? 1 : RS_LS_RAMP_CYCLE) - 1; Ramp_I_Re = Ramp_I_Im = 0;
        Cnt = Sample_Per_Cycle * RS_LS_PROBE_MEASURE_CYCLE - 1; I_DC_Sum = I_Re = I_Im = 0;
        assert(Run() == FAST_OFF);
        Failed();
    }
    for(unsigned Q = 0; Q < 3; Q++)
    {
        Setup(300); Window(RS_LS_PROBE_MEASURE, .3f, .00008f, .4f);
        if(Q != 1) I_Noise = .1f;
        if(Q != 0) Coh_I_Pow *= 2;
        assert(Run() == FAST_OFF);
        Failed();
    }
    for(unsigned K = 0; K < 2; K++)
    {
        Setup(300); Window(RS_LS_PROBE_MEASURE, K ? .3f : -1, K ? -.00008f : .00008f, .4f);
        assert(Run() == FAST_OFF);
        Failed();
    }
    for(unsigned S = RS_LS_MEASURE_A; S <= RS_LS_MEASURE_C; S++)
    for(unsigned K = 0; K < 4; K++)
    {
        Setup(150); Rs_A = .3f; Ls_A = .00008f; Rs_B = .4f; Ls_B = .00009f;
        Window((Rs_Ls_State_e)S, K == 2 ? -1 : .3f, K == 3 ? -.00008f : .00008f, K == 0 ? 0 : (K == 1 ? .01f : .4f));
        assert(Run() == FAST_OFF);
        Failed();
    }
    Setup(150); Freq_Refined = true; Rs_A = .3f; Ls_A = .00008f;
    Window(RS_LS_MEASURE_B, .5f, .00008f, .4f);
    assert(Run() == FAST_VOLTAGE && State == RS_LS_MEASURE_C);
    Window(RS_LS_MEASURE_C, .8f, .00008f, .4f);
    assert(Run() == FAST_OFF);
    Failed();

    /* Initial rejection retries at 300 Hz, then resumes alignment. */
    Setup(150); Window(RS_LS_PROBE_MEASURE, .3f, .00008f, .4f); I_Noise = .1f;
    assert(Run() == FAST_VOLTAGE && State == RS_LS_NOISE);
    assert(Reports == 0 && Ident_Fail_Reason == IDENT_FAIL_NONE);
    Window(RS_LS_PROBE_MEASURE, .3f, .00008f, .4f);
    assert(Run() == FAST_VOLTAGE && State == RS_LS_ALIGN);
    assert(Sample_Per_Cycle == (uint32_t)(CUR_FREQ_HZ_DEFAULT / 300.0f + 0.5f));
    /* A second rejection ends identification. */
    Window(RS_LS_PROBE_MEASURE, .3f, -.00008f, .4f);
    assert(Run() == FAST_OFF);
    Failed();
    puts("PASS: RL timing, voltage ramps, A/B/C rejection branches and probe retry");
}
