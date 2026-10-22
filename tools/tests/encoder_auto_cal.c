/* Host fixture: execute the actual calibration, VF and motor-control code.
 * Include Enc_Cal.c to exercise its private fit without adding a firmware API. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "Enc_Cal.c"
#include "Enc_Control.h"
#include "Servo_Phase.c"
#include "Current_Loop.h"
#include "Motor_Config.h"
#include "Motor_PWM.h"
#include "Motion_Loop.h"
#include "Open_Loop.h"
#include "Sensorless.h"
#include "Signal_Injection.h"
#include "motor_thread.h"
#include "Protocol.h"
#include "Protection.h"

TIM_TypeDef Test_TIM;
Motor_Para_T Motor_Para;
volatile ADC_T ADC;
volatile Encoder_T Encoder;
Encoder_Harmonic_T Encoder_Harmonic;
Mechanical_ESO_T Mechanical_ESO;
uint8_t Current_FF_Enable;
uint8_t Open_Loop_Method;
volatile float Motor_Plot_Iq_Ref;
volatile Motor_Work_State_e Motor_Work_State;
static int Fit_Requests;
static int Save_Requests;
static bool Work_Reject;
static Motor_Work_Job_e Work_Job;
static uint16_t Saved_Ids[10];
static uint16_t Saved_Count;
static uint8_t Saved_Valid;
static int Storage_Error;
static bool Enable_Allowed = true;
static unsigned PWM_Starts;
TX_QUEUE Motor_Cmd_Q;
static Motor_Cmd_Msg_T Queue[8];
static unsigned Wr, Rd;
static struct
{
    unsigned Kind, Id, Status, Txn;
} Log[16];
static unsigned Log_Cnt;

static UINT tx_queue_receive(TX_QUEUE *Q, void *Msg, ULONG Wait)
{
    (void)Q;
    (void)Wait;
    if (Rd == Wr)
        return 1U;
    *(Motor_Cmd_Msg_T *)Msg = Queue[Rd++];
    return TX_SUCCESS;
}
void Protocol_Write_Response(uint8_t Txn, uint16_t Id, AxDr_Status_e Status)
{
    assert(Log_Cnt < 16);
    Log[Log_Cnt].Txn = Txn;
    Log[Log_Cnt].Kind = 0;
    Log[Log_Cnt].Id = Id;
    Log[Log_Cnt++].Status = Status;
}
void Protocol_Action_Complete(uint8_t Txn, uint16_t Id, AxDr_Status_e Status)
{
    Protocol_Write_Response(Txn, Id, Status);
    Log[Log_Cnt - 1].Kind = 1;
}
void Protocol_Parameter_Write_Response(uint8_t Txn, uint16_t Id, Parameter_Status_e Status)
{
    (void)Txn;
    (void)Id;
    (void)Status;
    abort();
}
Parameter_Status_e Parameter_Write(uint16_t Id, Parameter_Type_e Type, Parameter_Value_T Value)
{
    (void)Id;
    (void)Type;
    (void)Value;
    abort();
}
bool Protection_Enable_Allowed(void)
{
    return Enable_Allowed;
}
bool Protection_Clear(void)
{
    return true;
}
#include "commands.inc"

void Encoder_Position_Read(int32_t *Turn, float *Theta)
{
    *Turn = 0;
    *Theta = 0;
}
void PWM_Enable(void)
{
    if (Enc_Step == ENC_PHASE_RUN)
        assert(Motor_State_Get() == ENABLED && Servo_Phase_Active());
    PWM_Starts++;
    TIM1->BDTR = TIM_BDTR_MOE;
}
void PWM_Disable(void)
{
    TIM1->BDTR = 0;
}
float Mechanical_ESO_Wm_Get(void)
{
    return Mechanical_ESO.State.Wm;
}
void Mechanical_ESO_Run(float Theta, bool Valid, float Iq)
{
    (void)Theta;
    (void)Valid;
    (void)Iq;
}
bool Protection_Current_Fast(float Ia, float Ib, float Ic)
{
    (void)Ia;
    (void)Ib;
    (void)Ic;
    return true;
}
bool Motor_Work_Request(Motor_Work_Job_e Job)
{
    assert(Motor_Work_State == MOTOR_WORK_IDLE);
    assert(Motor_State_Get() == DISABLED && (TIM1->BDTR & TIM_BDTR_MOE) == 0);
    if (Job == MOTOR_WORK_FIT)
        Fit_Requests++;
    else
        Save_Requests++;
    if (Work_Reject)
        return false;
    Work_Job = Job;
    Motor_Work_State = MOTOR_WORK_BUSY;
    return true;
}
void Motor_Work_Ack(void)
{
    Motor_Work_State = MOTOR_WORK_IDLE;
}
void Encoder_DMA_Config(void)
{
    /* Hardware rebinding is outside this host fixture. */
}
#include "encoder_angle.inc"

int NVS_Storage_Save_Ids(const uint16_t *Ids, uint16_t Count)
{
    assert(Motor_State_Get() == DISABLED && TIM1->BDTR == 0);
    assert(Count <= 10);
    memcpy(Saved_Ids, Ids, Count * sizeof(*Ids));
    Saved_Count = Count;
    Saved_Valid = Motor_Cal.Valid;
    return Storage_Error;
}

static void Worker_Run(void)
{
    assert(Motor_Work_State == MOTOR_WORK_BUSY);
    assert(Motor_State_Get() == DISABLED);
    bool Passed = false;
    if (Work_Job == MOTOR_WORK_FIT)
        Passed = Enc_Cal_Fit_Apply();
    else if (Work_Job == MOTOR_WORK_SAVE_ENC)
        Passed = Enc_Cal_Save();
    else if (Work_Job == MOTOR_WORK_SAVE_PHASE)
        Passed = Servo_Phase_Save();
    else
        abort();
    Motor_Work_State = Passed ? MOTOR_WORK_DONE : MOTOR_WORK_FAILED;
}

/* Unrelated run modes must not execute during calibration from any user mode. */
#pragma GCC diagnostic ignored "-Wunused-parameter"
bool Sensorless_Begin(void)
{
    abort();
}
void Sensorless_Stop_Request(void)
{
    abort();
}
void Sensorless_Stop(void)
{
}
bool Sensorless_Speed_Control_Active(void)
{
    abort();
}
float Sensorless_Wm_Get(void)
{
    return 0;
}
Sensorless_State_e Sensorless_State_Get(void)
{
    abort();
}
void Sensorless_Control(float We, float Min, float Max)
{
    abort();
}
void Sensorless_Run(float Ia, float Ib, float We, float *Theta, float *Id, float *Iq)
{
    abort();
}
void Open_Loop_Reset(void)
{
    abort();
}
Motor_Fast_Mode_e Open_Loop(float We, float *Theta, float *Id, float *Iq, float *Ua, float *Ub)
{
    abort();
}
bool Identification_Start(Ident_Mode_e Mode, float Wm)
{
    abort();
}
volatile Ident_State_e Ident_State = IDENT_IDLE;
void Identification_Stop(void)
{
    abort();
}
void Identification_Report(bool Valid)
{
    (void)Valid;
    abort();
}
void Identification_Control(void)
{
    abort();
}
bool Identification_Apply(void)
{
    abort();
}
Motor_Fast_Mode_e Identification_Fast_Run(float Ia, float Ib, float Ic, float *Theta, float *Id,
                                          float *Iq, float *Ua, float *Ub)
{
    abort();
}

static void Setup(int Mode)
{
    Motor_Disable();
    Enc_Step = ENC_IDLE;
    Enc_Cal.Step = CAL_IDLE;
    Motor_Work_State = MOTOR_WORK_IDLE;
    Servo_Phase_Abort();
    Fit_Requests = 0;
    Save_Requests = 0;
    Work_Reject = false;
    Storage_Error = 0;
    Saved_Count = 0;
    Enable_Allowed = true;
    PWM_Starts = 0;
    Wr = Rd = Log_Cnt = 0;
    Enc_Action_Pending = Ident_Action_Pending = Save_Action_Pending = false;
    Motor_Mode = Mode;
    Motor_Para = (Motor_Para_T){
        .Pp = 7, .Rs = .28252554f, .Ld = .0000713822f, .Lq = .0000713822f, .Flux = .002281195f};
    Motor_Cal.Valid = 1;
    Motor_Cal.Enc_Dir = -1;
    Motor_Cal.Theta_Off = .2f;
    Motor_Run = (Motor_Run_T){0};
    Motor_Cmd = (Motor_Cmd_T){.Te_Target = .05f, .Wm_Target = 30.0f};
    Motor_Config.Dir = -1;
    User_Lim.I_Max = 10.0f;
    User_Lim.Wm_Max = 1200.0f;
    ADC = (ADC_T){.Vbus_V = 24.0f};
    Encoder = (Encoder_T){.Ready = 1, .Valid = 1};
    Encoder_Harmonic = (Encoder_Harmonic_T){0};
    Mechanical_ESO = (Mechanical_ESO_T){.Para.Valid = 1};
    Motion_Config = (Motion_Config_T){.Wm_Acc = 100.0f, .Wm_Dec = 100.0f};
    Speed_Ctrl = (PID_T){.Para.Kp = .04f, .Para.Ki = .3f};
    Id_Ctrl = (PID_T){.Para.Kp = .9f, .Para.Ki = 3500.0f};
    Iq_Ctrl = Id_Ctrl;
    Id_LADRC = (LADRC_T){.Para.B0 = 14009.0f, .Para.Kp = 4700.0f, .Para.Wo = 18000.0f};
    Iq_LADRC = Id_LADRC;
    Current_Controller = CURRENT_CTRL_PI;
    assert(Motor_Enc_Cal_Start() == MOTOR_OK);
    assert(Motor_Mode == Mode && Motor_State_Get() == RUN);
}

static Motor_Fast_Mode_e Fast(float *Theta, float *Id, float *Iq, float *Ua, float *Ub)
{
#include "adc_control.inc"
finish:
    *Theta = Theta_e;
    *Id = Id_Ref;
    *Iq = Iq_Ref;
    *Ua = Motor_Run.Ualpha;
    *Ub = Motor_Run.Ubeta;
    return Fast_Mode;
}

static void Modes(void)
{
    for (int Mode = TORQUE; Mode <= PHASE_SEARCH; Mode++)
    {
        Setup(Mode);
        for (int n = 0; n < 100; n++)
            Motor_Control();
        assert(fabsf(Motor_Wm_Ref_Get() + 5.0f) < .001f);
        assert(Motor_Cmd.Te_Target == .05f && Motor_Cmd.Wm_Target == 30.0f);
        float Theta, Id, Iq, Ua, Ub;
        assert(Fast(&Theta, &Id, &Iq, &Ua, &Ub) == FAST_CURRENT);
        assert(Iq > 0);
        Mechanical_ESO.State.Wm = ENC_CAL_WM;
        Motor_Run.Iq = .3f;
        Motor_Cal_Decel_Begin();
        Enc_Cal.Step = CAL_DECEL;
        for (int n = 0; n < 100; n++)
            Motor_Control();
        assert(fabsf(Motor_Wm_Ref_Get() + (ENC_CAL_WM - 5.0f)) < .002f);
        assert(Motor_Mode == Mode && User_Lim.I_Max == 10.0f);
    }
    Motor_Disable();
    Motor_Cal.Valid = 0;
    Motor_Control();
    assert(Motor_Enc_Cal_Start() == MOTOR_ERR_CONFIG);
    assert(Motor_State_Get() == DISABLED && TIM1->BDTR == 0);
}

static void Transfer(void)
{
    for (int Mode = TORQUE; Mode <= PHASE_SEARCH; Mode++)
      for (int Ctrl = 0; Ctrl < 2; Ctrl++)
        for (int FF = 0; FF < 2; FF++)
        {
            Setup(Mode);
            Current_Controller = Ctrl;
            Current_FF_Enable = FF;
            Mechanical_ESO.State.Wm = ENC_CAL_WM;
            Motor_Run.Ualpha = 1.8f;
            Motor_Run.Ubeta = -.98f;
            Enc_Cal.Step = CAL_VF;
            float Theta, Id, Iq, Ua, Ub;
            float Mag = hypotf(Motor_Run.Ualpha, Motor_Run.Ubeta);
            float Prev = atan2f(Motor_Run.Ubeta, Motor_Run.Ualpha);
            float Delta = 7 * ENC_CAL_WM * CUR_TS;
            /* Production SinCos uses a 1024-entry, non-interpolated LUT:
             * phase-step error may reach one bin (2pi/1024), not zero. */
            for (int n = 0; n < 2000; n++)
            {
                assert(Fast(&Theta, &Id, &Iq, &Ua, &Ub) == FAST_VOLTAGE);
                float Phase = atan2f(Ub, Ua);
                assert(fabsf(remainderf(Phase - Prev, TWO_PI_F) - Delta) < .0062f);
                assert(fabsf(hypotf(Ua, Ub) - Mag) < .00003f);
                Prev = Phase;
            }
            Enc_Cal.Step = CAL_HANDOFF;
            for (int n = 0; n < 30; n++) /* Delay the slow owner by 1.5 ms. */
            {
                Encoder.Theta_m = Angle_Wrap(Encoder.Theta_m + ENC_CAL_WM * CUR_TS);
                ADC.Ia_A = .1f + .002f * n;
                ADC.Ib_A = .2f - .003f * n;
                assert(Fast(&Theta, &Id, &Iq, &Ua, &Ub) == FAST_VOLTAGE);
                float Phase = atan2f(Ub, Ua);
                assert(fabsf(remainderf(Phase - Prev, TWO_PI_F) - Delta) < .0062f);
                Prev = Phase;
            }
            Motor_Control();
            assert(Enc_Cal.Step == CAL_DECEL);
            assert(fabsf(Motor_Wm_Ref_Get() + ENC_CAL_WM) < .0001f);
            Encoder.Theta_m = Angle_Wrap(Encoder.Theta_m + ENC_CAL_WM * CUR_TS);
            ADC.Ia_A = .35f;
            ADC.Ib_A = -.15f;
            float S, C;
            SinCos(Cal_VF.State.Theta_e, &S, &C);
            float ExpectedA = -Mag * S;
            float ExpectedB = Mag * C;
            assert(Fast(&Theta, &Id, &Iq, &Ua, &Ub) == FAST_CURRENT);
            assert(!Enc_Cal_Handoff_Ready);
            SinCos(Theta, &S, &C);
            assert(fabsf(Id - (ADC.Ia_A * C + (ADC.Ia_A + 2 * ADC.Ib_A) * INV_SQRT3_F * S)) < .00001f);
            assert(fabsf(Iq - (-ADC.Ia_A * S + (ADC.Ia_A + 2 * ADC.Ib_A) * INV_SQRT3_F * C)) < .00001f);
            assert(fabsf(Ua - ExpectedA) < .00005f && fabsf(Ub - ExpectedB) < .00005f);
            assert(fabsf(remainderf(atan2f(Ub, Ua) - Prev, TWO_PI_F) - Delta) < .0062f);
            /* Reference holds until the slow loop runs; a second ADC sample
             * must not track the measured current again. */
            float Id_Hold = Id, Iq_Hold = Iq;
            float VF_Theta = Cal_VF.State.Theta_e;
            ADC.Ia_A += .2f;
            assert(Fast(&Theta, &Id, &Iq, &Ua, &Ub) == FAST_CURRENT);
            assert(Id == Id_Hold && Iq == Iq_Hold);
            assert(Cal_VF.State.Theta_e == VF_Theta);
        }
}

static void Capture(int Dir, float Noise)
{
    Setup(SPEED);
    Motor_Cal.Enc_Dir = Dir;
    Enc_Cal.Step = CAL_SAMPLE;
    const double Coeff[4] = {.0000587322, -.000600221, -.001006767, -.001470696};
    for (unsigned n = 0; n < ENC_CAL_SAMPLES; n++)
    {
        double True = .83 + Dir * (double)ENC_CAL_WM * (double)ENC_CAL_SAMPLE_TS * n;
        double Native = True;
        for (int i = 0; i < 8; i++)
            Native = True + Coeff[0] * cos(Native) + Coeff[1] * sin(Native) +
                     Coeff[2] * cos(2 * Native) + Coeff[3] * sin(2 * Native);
        Native += Noise * sin(n * .37);
        float Angle = (float)(Native - floor(Native / (2 * M_PI)) * (2 * M_PI));
        for (unsigned k = 0; k < ENC_CAL_SAMPLE_DIV; k++)
            Enc_Cal_Sample(Angle);
    }
    assert(Sample_Cnt == ENC_CAL_SAMPLES);
}

static void Phase_Complete(void)
{
    assert(Enc_Step == ENC_PHASE_RUN && Motor_State_Get() == RUN);
    for (unsigned n = 0; n < 16000; n++)
    {
        float T, D, Q, A, B;
        Encoder.Theta_Native = (n < 2100) ? 1.0f : 1.02f;
        (void)Fast(&T, &D, &Q, &A, &B);
    }
    assert(!Servo_Phase_Active());
    Motor_Control();
}

static void Fit(void)
{
    for (int Dir = -1; Dir <= 1; Dir += 2)
    {
        Capture(Dir, 0);
        float C[4];
        assert(Enc_Cal_Fit(C));
        const float Expected[4] = {.0000587322f, -.000600221f, -.001006767f, -.001470696f};
        /* Include the production LUT's phase quantization in this bound. */
        for (int i = 0; i < 4; i++)
            assert(fabsf(C[i] - Expected[i]) < .00002f);
        Enc_Cal.Step = CAL_FIT_WAIT;
        /* PWM off alone must no longer authorize applying new coefficients. */
        PWM_Disable();
        assert(!Enc_Cal_Fit_Apply());
        Enc_Cal.Step = CAL_FIT;
        Motor_Control();
        assert(Enc_Cal.Step == CAL_FIT_WAIT && Motor_State_Get() == DISABLED);
        assert(Motor_Work_State == MOTOR_WORK_BUSY && Work_Job == MOTOR_WORK_FIT);
        Worker_Run();
        assert(Motor_Cal.Valid == 0 && fabsf(Encoder_Harmonic.C2 - C[2]) < 1e-9f);
        Motor_Control();
        assert(Enc_Cal.Step == CAL_SAVE_WAIT && Work_Job == MOTOR_WORK_SAVE_ENC);
        assert(Motor_State_Get() == DISABLED && !Servo_Phase_Active() && TIM1->BDTR == 0);
        Worker_Run();
        const uint16_t Enc_Ids[] = {PARAM_CAL_VALID, PARAM_ENCODER_HARMONIC_C1,
                                   PARAM_ENCODER_HARMONIC_S1, PARAM_ENCODER_HARMONIC_C2,
                                   PARAM_ENCODER_HARMONIC_S2};
        assert(Saved_Count == 5 && !memcmp(Saved_Ids, Enc_Ids, sizeof Enc_Ids));
        assert(Saved_Valid == 0);
        Motor_Control();
        assert(Enc_Cal.Step == CAL_DONE && Enc_Step == ENC_PHASE_RUN);
        assert(Servo_Phase_Active() && Motor_State_Get() == RUN && PWM_Starts == 2);
        Phase_Complete();
        assert(Enc_Step == ENC_PHASE_SAVE && Motor_Cal.Valid == 1);
        assert(Motor_Work_State == MOTOR_WORK_BUSY);
        assert(!Enc_Action_Pending && Save_Requests == 2);
        Worker_Run();
        const uint16_t Phase_Ids[] = {PARAM_CAL_ENC_DIR, PARAM_CAL_THETA_OFF, PARAM_CAL_VALID};
        assert(Saved_Count == 3 && !memcmp(Saved_Ids, Phase_Ids, sizeof Phase_Ids));
        assert(Saved_Valid == 1);
        Motor_Control();
        assert(Enc_Step == ENC_DONE && Motor_Work_State == MOTOR_WORK_IDLE);
        assert(Motor_State_Get() == DISABLED && TIM1->BDTR == 0);
        assert(fabsf(Encoder_Harmonic.C2 - C[2]) < 1e-9f);
    }
    Capture(-1, .02f);
    float C[4];
    assert(!Enc_Cal_Fit(C));
    assert(Encoder_Harmonic.C2 == 0 && Motor_Cal.Valid == 1);
    Capture(-1, 0);
    Sample_Cnt--;
    assert(!Enc_Cal_Fit(C));
}

/* Known coefficients and residual energy, with correlated columns. Construct
 * moments independently of the sampler and solver to test acceptance limits. */
static void Moments(const double Beta[6], double Rms)
{
    double Energy = ENC_CAL_SAMPLES * Rms * Rms;
    Sample_Cnt = ENC_CAL_SAMPLES;
    for (unsigned i = 0; i < ENC_CAL_FIT_N; i++)
    {
        double Sum = 0;
        for (unsigned j = 0; j < ENC_CAL_FIT_N; j++)
        {
            double Gram = ENC_CAL_SAMPLES * pow(.2, abs((int)i - (int)j));
            if (j >= i)
                A[i][j] = (float)Gram;
            Sum += Gram * Beta[j];
        }
        B[i] = (float)Sum;
        Energy += Beta[i] * Sum;
    }
    Y2 = (float)Energy;
}

static void Fit_Bounds(void)
{
    double Beta[6] = {.02, .001, .001, -.002, .003, -.004};
    float C[4];
    Moments(Beta, ENC_CAL_RMS_MAX * .999);
    assert(Enc_Cal_Fit(C));
    for (unsigned i = 0; i < 4; i++)
        assert(fabs(C[i] - Beta[i + 2]) < 1e-7);
    Moments(Beta, ENC_CAL_RMS_MAX * 1.001);
    assert(!Enc_Cal_Fit(C));

    for (unsigned k = 1; k < ENC_CAL_FIT_N; k++)
        for (int Sign = -1; Sign <= 1; Sign += 2)
        {
            double Edge[6] = {0};
            double Limit = (k == 1) ? .1f * ENC_CAL_WM : ENC_CAL_COEF_MAX;
            Edge[k] = Sign * Limit * .999;
            Moments(Edge, 0);
            assert(Enc_Cal_Fit(C));
            Edge[k] = Sign * Limit * 1.001;
            Moments(Edge, 0);
            assert(!Enc_Cal_Fit(C));
        }

    for (unsigned i = 0; i < ENC_CAL_FIT_N; i++)
    {
        Moments(Beta, 0);
        A[i][i] = NAN;
        assert(!Enc_Cal_Fit(C));
        A[i][i] = INFINITY;
        assert(!Enc_Cal_Fit(C));
        Moments(Beta, 0);
        B[i] = NAN;
        assert(!Enc_Cal_Fit(C));
    }
    Moments(Beta, 0);
    Y2 = INFINITY;
    assert(!Enc_Cal_Fit(C));
    Y2 = -1;
    assert(!Enc_Cal_Fit(C));
    memset(A, 0, sizeof A);
    memset(B, 0, sizeof B);
    Y2 = 0;
    for (unsigned i = 0; i < ENC_CAL_FIT_N; i++)
        A[i][i] = ENC_CAL_SAMPLES;
    A[5][5] = .000099f * ENC_CAL_SAMPLES;
    assert(!Enc_Cal_Fit(C));
    A[5][5] = .000101f * ENC_CAL_SAMPLES;
    assert(Enc_Cal_Fit(C));
}

static void Fit_Capture_Quality(void)
{
    float C[4];
    for (int Dir = -1; Dir <= 1; Dir += 2)
    {
        Capture(Dir, sqrtf(2) * ENC_CAL_RMS_MAX * .99f);
        assert(Enc_Cal_Fit(C));
        Capture(Dir, sqrtf(2) * ENC_CAL_RMS_MAX * 1.01f);
        assert(!Enc_Cal_Fit(C));
    }
    /* A full-length but stationary capture has no independent harmonic basis. */
    Setup(SPEED);
    Enc_Cal.Step = CAL_SAMPLE;
    for (unsigned n = 0; n < ENC_CAL_SAMPLES * ENC_CAL_SAMPLE_DIV; n++)
        Enc_Cal_Sample(.2f);
    assert(Sample_Cnt == ENC_CAL_SAMPLES && !Enc_Cal_Fit(C));
}

static void Quality(void)
{
    Setup(SPEED);
    Enc_Cal.Step = CAL_SAMPLE;
    Encoder.Valid = 0;
    Enc_Cal_Sample(.2f);
    Encoder.Valid = 1;
    assert(Sample_Cnt > ENC_CAL_SAMPLES);
    Motor_Control();
    assert(Enc_Cal.Step == CAL_FAILED && Motor_State_Get() == DISABLED);
    Setup(SPEED);
    Enc_Cal.Step = CAL_SAMPLE;
    Mechanical_ESO.State.Wm = .7f * ENC_CAL_WM;
    Motor_Control();
    assert(Enc_Cal.Step == CAL_FAILED);
    Setup(SPEED);
    Enc_Cal.Step = CAL_VF;
    VF_Ready = true;
    Mechanical_ESO.State.Wm = NAN;
    for (unsigned n = 0; n < ENC_CAL_VF_SETTLE_CNT; n++)
        Motor_Control();
    assert(Enc_Cal.Step == CAL_FAILED);
}

static void Decel(void)
{
    Setup(SPEED);
    Enc_Cal.Step = CAL_DECEL;
    Mechanical_ESO.State.Wm = 20 * ENC_CAL_STOP_WM;
    for (unsigned n = 0; n < ENC_CAL_STOP_WAIT_CNT + 10; n++)
        Motor_Control();
    assert(Enc_Cal.Step == CAL_DECEL && Fit_Requests == 0);
    Mechanical_ESO.State.Wm = 0;
    for (unsigned n = 0; n < ENC_CAL_STOP_WAIT_CNT; n++)
        Motor_Control();
    assert(Enc_Cal.Step == CAL_FIT);
    float T, D, Q, A, B;
    assert(Fast(&T, &D, &Q, &A, &B) == FAST_OFF);
    Motor_Control();
    assert(Enc_Cal.Step == CAL_FIT_WAIT && Fit_Requests == 1);
    Setup(SPEED);
    Enc_Cal.Step = CAL_DECEL;
    Mechanical_ESO.State.Wm = 50;
    for (int n = 0; n < 20000 && Enc_Cal.Step == CAL_DECEL; n++)
        Motor_Control();
    assert(Enc_Cal.Step == CAL_FAILED && Fit_Requests == 0);
}

static void Abort(void)
{
    for (int Step = CAL_SPEED; Step <= CAL_SAVE_WAIT; Step++)
        for (int Disable = 0; Disable < 2; Disable++)
        {
            Setup(TORQUE);
            Enc_Cal.Step = Step;
            if (Disable)
                Motor_Disable();
            else
                Motor_Stop();
            Motor_Control();
            assert(TIM1->BDTR == 0);
            assert(Motor_State_Get() == DISABLED && Enc_Cal.Step == CAL_FAILED);
            assert(!Servo_Phase_Active() && Fit_Requests == 0 && TIM1->BDTR == 0);
            assert(!Enc_Cal_Fit_Apply());
        }
}

static void Replay(int Dir)
{
    Setup(SPEED);
    Motor_Cal.Enc_Dir = Dir;
    Enc_Cal.Step = CAL_SAMPLE;
    for (unsigned n = 0; n < ENC_CAL_SAMPLES; n++)
    {
        float Angle;
        assert(fread(&Angle, sizeof Angle, 1, stdin) == 1);
        for (unsigned k = 0; k < ENC_CAL_SAMPLE_DIV; k++)
            Enc_Cal_Sample(Angle);
    }
    float C[4];
    assert(Enc_Cal_Fit(C));
    printf("[%.10g, %.10g, %.10g, %.10g]\n", C[0], C[1], C[2], C[3]);
}

static void Enqueue(Motor_Cmd_e Cmd, uint16_t Id)
{
    assert(Wr < 8);
    Queue[Wr++] = (Motor_Cmd_Msg_T){.Cmd = Cmd, .Reserved = Id};
}

static void Commands(void)
{
    const Motor_Cmd_e Cancel[] = {MOTOR_CMD_STOP, MOTOR_CMD_DISABLE, MOTOR_CMD_EN_TOGGLE,
                                  MOTOR_CMD_RUN_TOGGLE};
    for (unsigned k = 0; k < 4; k++)
        for (int Step = CAL_SPEED; Step <= CAL_SAVE_WAIT; Step++)
        {
            Setup(TORQUE);
            Motor_Disable();
            Motor_Control();
            Enqueue(MOTOR_CMD_ENC_CAL_START, ACTION_ENC_CAL_START);
            Motor_Cmd_Run();
            assert(Enc_Action_Pending && Log_Cnt == 1 && Log[0].Status == AXDR_OK);
            Enc_Cal.Step = Step;
            if (Step >= CAL_FIT_WAIT)
                Motor_Work_State = MOTOR_WORK_BUSY;
            Enqueue(Cancel[k], 0x1000);
            Enqueue(MOTOR_CMD_RUN, 0x1001);
            Enqueue(MOTOR_CMD_ENABLE, 0x1002);
            Motor_Cmd_Run();
            assert(Motor_State_Get() == DISABLED);
            if (k < 2)
                assert(Log[1].Status == AXDR_OK);
            assert(Log[Log_Cnt - 1].Status == AXDR_ERR_STATE);
            Motor_Control();
            assert(TIM1->BDTR == 0);
            if (Step >= CAL_FIT_WAIT)
            {
                Motor_Async_Action_Poll();
                assert(Enc_Action_Pending && Enc_Step == ENC_STOP_WAIT);
                assert(!Enc_Cal_Fit_Apply());
                Motor_Work_State = MOTOR_WORK_FAILED;
                Motor_Control();
            }
            Motor_Async_Action_Poll();
            assert(!Enc_Action_Pending && !Enc_Control_Active() && !Servo_Phase_Active());
            assert(Log[Log_Cnt - 1].Kind == 1 && Log[Log_Cnt - 1].Id == ACTION_ENC_CAL_START);
            assert(Log[Log_Cnt - 1].Status == AXDR_ERR_CONFIG && Save_Requests == 0);
        }
    for (int Passed = -1; Passed < 2; Passed++)
    {
        Setup(SPEED);
        Enc_Action_Pending = true;
        Enc_Action = ACTION_ENC_CAL_START | (0xA5U << MOTOR_ACTION_TXN_SHIFT);
        Enc_Cal.Step = CAL_DONE;
        Encoder_Harmonic.C1 = .001f;
        Motor_Control();
        Work_Reject = (Passed < 0);
        Phase_Complete();
        assert(Motor_State_Get() == DISABLED && Save_Requests == 1 && Log_Cnt == 0);
        assert(Motor_Cal.Valid == 1 && Encoder_Harmonic.C1 == .001f);
        if (!Work_Reject)
        {
            Motor_Async_Action_Poll();
            assert(Log_Cnt == 0 && Enc_Action_Pending);
            Storage_Error = Passed ? 0 : -1;
            Worker_Run();
            Motor_Control();
        }
        Motor_Async_Action_Poll();
        assert(!Enc_Action_Pending && Log_Cnt == 1 && Log[0].Kind == 1);
        assert(Log[0].Id == ACTION_ENC_CAL_START && Log[0].Txn == 0xA5);
        assert(Log[0].Status == ((Passed == 1) ? AXDR_OK : AXDR_ERR_CONFIG));
        Motor_Async_Action_Poll();
        assert(Log_Cnt == 1 && Motor_Work_State == MOTOR_WORK_IDLE);
        assert(Save_Requests == 1 && Motor_Cal.Valid == 1 && Encoder_Harmonic.C1 == .001f);
    }
}

static void Phase_Sequence(void)
{
    for (int Mode = TORQUE; Mode <= PHASE_SEARCH; Mode++)
    {
        Setup(Mode);
        /* The encoder scheduler owns disable and restart without an outer
         * Motor_Control phase-transition branch. */
        Enc_Cal.Step = CAL_FIT;
        Enc_Control_Run();
        assert(Motor_State_Get() == DISABLED);
        Motor_Work_State = MOTOR_WORK_DONE;
        Enc_Control_Run();
        assert(Enc_Cal.Step == CAL_SAVE_WAIT && Motor_State_Get() == DISABLED);
        Motor_Work_State = MOTOR_WORK_DONE;
        Enc_Control_Run();
        assert(Enc_Step == ENC_PHASE_RUN && Servo_Phase_Active());
        assert(Motor_State_Get() == RUN && Motor_Mode == Mode);
        assert(Motor_Cmd.Wm_Target == 30 && Motor_Cmd.Te_Target == .05f);
        Phase_Complete();
        Worker_Run();
        Enc_Control_Run();
        assert(Enc_Step == ENC_DONE && Motor_State_Get() == DISABLED);
        assert(Motor_Mode == Mode && TIM1->BDTR == 0);
    }
    for (unsigned k = 0; k < 2; k++)
    {
        Setup(TORQUE);
        Motor_Disable();
        Motor_Control();
        Motor_Mode = PHASE_SEARCH;
        Motor_Cal.Valid = 0;
        Encoder_Harmonic = (Encoder_Harmonic_T){.C1 = .001f, .S1 = -.002f};
        assert(Motor_Enable() == MOTOR_OK);
        assert(Motor_Start() == MOTOR_OK);
        assert(Enc_Step == ENC_PHASE_RUN && Motor_State_Get() == RUN);
        unsigned Starts = PWM_Starts;
        if (k == 0)
        {
            Phase_Complete();
            Worker_Run();
            assert(Saved_Count == 3 && Save_Requests == 1 && Fit_Requests == 0);
            Motor_Control();
            assert(Enc_Step == ENC_DONE);
        }
        else
        {
            Motor_Stop();
            Motor_Control();
            assert(Enc_Step == ENC_FAILED && PWM_Starts == Starts);
            assert(Save_Requests == 0);
        }
        assert(Motor_State_Get() == DISABLED && TIM1->BDTR == 0);
        assert(Encoder_Harmonic.C1 == .001f && Encoder_Harmonic.S1 == -.002f);
    }
}

static void Failures(void)
{
    /* Fit, save submission, Flash write, re-enable and phasing failures. */
    for (unsigned Failure = 0; Failure < 5; Failure++)
    {
        Capture(-1, 0);
        Enc_Action_Pending = true;
        Enc_Action = ACTION_ENC_CAL_START;
        Enc_Cal.Step = CAL_FIT;
        Motor_Control();
        if (Failure == 0)
            Y2 = INFINITY;
        Worker_Run();
        Work_Reject = (Failure == 1);
        Motor_Control();
        if (Failure >= 2)
        {
            assert(Work_Job == MOTOR_WORK_SAVE_ENC && Motor_State_Get() == DISABLED);
            Storage_Error = (Failure == 2) ? -1 : 0;
            Worker_Run();
            Enable_Allowed = (Failure != 3);
            Motor_Control();
            if (Failure == 4)
            {
                float T, D, Q, A, B;
                assert(Enc_Step == ENC_PHASE_RUN);
                Encoder.Fault = 1;
                assert(Fast(&T, &D, &Q, &A, &B) == FAST_OFF);
                Motor_Control();
            }
        }
        Motor_Control();
        Motor_Async_Action_Poll();
        assert(Enc_Step == ENC_FAILED && Motor_State_Get() == DISABLED && TIM1->BDTR == 0);
        assert(Motor_Work_State == MOTOR_WORK_IDLE && !Enc_Action_Pending);
        assert(Log_Cnt == 1 && Log[0].Kind == 1 && Log[0].Status == AXDR_ERR_CONFIG);
        assert(Motor_Cal.Valid == (Failure == 0));
        assert(Save_Requests <= 1 && Saved_Count != 3);
        if (Failure != 0)
            assert(fabsf(Encoder_Harmonic.C2) > .0009f);
    }
}

static void Normal(void)
{
    const Enc_Cal_Step_e Steps[] = {CAL_IDLE, CAL_DONE, CAL_FAILED};
    for (unsigned k = 0; k < 3; k++)
    {
        Setup(TORQUE);
        Motor_Disable();
        Motor_Control();
        Enc_Cal.Step = Steps[k];
        Motor_Cal.Valid = 1;
        Motor_Enable();
        Motor_Start();
        Motor_Control();
        float Theta, Id, Iq, Ua, Ub;
        assert(Fast(&Theta, &Id, &Iq, &Ua, &Ub) == FAST_CURRENT);
        float Expected = -Motor_Cmd.Te_Target / (1.5f * 7.0f * Motor_Para.Flux);
        assert(fabsf(Iq - Expected) < .00001f);
        Motor_Stop();
        Motor_Control();
        assert(Motor_State_Get() == ENABLED && TIM1->BDTR != 0);
        assert(Fast(&Theta, &Id, &Iq, &Ua, &Ub) == FAST_CURRENT && Iq == 0);
        Motor_Disable();
        Motor_Mode = SPEED;
        Motor_Enable();
        Motor_Start();
        for (int n = 0; n < 100; n++)
            Motor_Control();
        assert(fabsf(Motor_Wm_Ref_Get() - 5.0f) < .001f);
        assert(Enc_Cal.Step == Steps[k]);
    }
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    if (!strcmp(argv[1], "modes"))
        Modes();
    else if (!strcmp(argv[1], "transfer"))
        Transfer();
    else if (!strcmp(argv[1], "fit"))
        Fit();
    else if (!strcmp(argv[1], "fit-bounds"))
        Fit_Bounds();
    else if (!strcmp(argv[1], "fit-capture-quality"))
        Fit_Capture_Quality();
    else if (!strcmp(argv[1], "quality"))
        Quality();
    else if (!strcmp(argv[1], "decel"))
        Decel();
    else if (!strcmp(argv[1], "abort"))
        Abort();
    else if (!strcmp(argv[1], "commands"))
        Commands();
    else if (!strcmp(argv[1], "phase-sequence"))
        Phase_Sequence();
    else if (!strcmp(argv[1], "failures"))
        Failures();
    else if (!strcmp(argv[1], "normal"))
        Normal();
    else if (!strcmp(argv[1], "replay-forward"))
        Replay(-1);
    else if (!strcmp(argv[1], "replay-reverse"))
        Replay(1);
    else
        return 2;
    return 0;
}
