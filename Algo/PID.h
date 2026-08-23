/**
  ==============================================================================
  * @file    PID.h
  * @author  ZHOUHENG
  * @brief   PID controller header file
  ==============================================================================

  ==============================================================================
*/

#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        struct
        {
            float Kp;
            float Ki;
            float Kd;

            float Out_Max;
            float Out_Min;

            float Int_Max; // Integral windup limit
            float Int_Min;
        } Para;

        struct
        {
            float Ref;
            float Fbk;
            float Err;
            float Out;
        } Sig;

        struct
        {
            float Int;
            float Fbk_Pre;
        } State;

    } PID_T;

    void PID_Run(PID_T *Pid, float Ts);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */