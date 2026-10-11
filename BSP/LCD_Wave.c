/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 ZHOUHENG
 */

#include "LCD_Wave.h"

#include <string.h>

#include "ST7789.h"
#include "Encoder.h"
#include "Mechanical_ESO.h"
#include "Math.h"
#include "Motor_ADC.h"
#include "Motor_Config.h"
#include "Motor_Control.h"
#include "Motor_Type.h"
#include "main.h"
#include "tx_api.h"

#define LCD_SAMPLE_TICKS 10U /* 100 Hz, ThreadX tick = 1 ms */
#define LCD_FRAME_TICKS  33U /* ~30 FPS */
#define LCD_WINDOW_COUNT 500U /* 5 s at 100 Hz */
#define LCD_HISTORY_LEN  512U

#define LCD_GRAPH_Y      32U
#define LCD_GRAPH_H      (ST7789_HEIGHT - LCD_GRAPH_Y)
#define LCD_TILE_W       16U
#define LCD_TILE_H       8U
#define LCD_TILE_COLS    (ST7789_WIDTH / LCD_TILE_W)
#define LCD_TILE_ROWS    ((ST7789_HEIGHT + LCD_TILE_H - 1U) / LCD_TILE_H)
#define LCD_ROW_BYTES    (ST7789_WIDTH / 4U)
#define LCD_FRAME_BYTES  (LCD_ROW_BYTES * ST7789_HEIGHT)

/* Two bits per pixel: ref, feedback, both, or background. */
#define LCD_REF 1U
#define LCD_FBK 2U

static uint8_t Frame_Old[LCD_FRAME_BYTES];
static uint8_t Frame_New[LCD_FRAME_BYTES];
static uint8_t Tx_Buffer[ST7789_WIDTH * LCD_TILE_H * 2U];
static uint16_t Dirty[LCD_TILE_ROWS];

static float History_Ref[LCD_HISTORY_LEN];
static float History_Fbk[LCD_HISTORY_LEN];
static uint16_t History_Head = 0U;
static uint16_t History_Count = 0U;

static Motor_Mode_e Display_Mode = TORQUE;
static float Scale_Min = -1.0f;
static float Scale_Max = 1.0f;
/* Display interval is latched per position move; the trajectory keeps no origin. */
static float Pos_Start = 0.0f;
static float Pos_End = 0.0f;
static bool Pos_Run = false;

static bool LCD_Active = false;
static bool Frame_Valid = false;
static bool Flush_Active = false;
static uint8_t Flush_Row = 0U;
static uint32_t Sample_Tick = 0U;
static uint32_t Frame_Tick = 0U;
static uint32_t Frame_Bytes = 0U;

volatile LCD_Time_T LCD_Time = { 0 };
volatile uint32_t LCD_Test = 0U;

/* 5x7 glyphs, columns with least-significant bit at the top. */
static const char Font_Chars[] = " 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ./-+:";
static const uint8_t Font_Cols[][5] =
{
    { 0x00, 0x00, 0x00, 0x00, 0x00 }, /* space */
    { 0x3E, 0x51, 0x49, 0x45, 0x3E }, /* 0 */
    { 0x00, 0x42, 0x7F, 0x40, 0x00 }, /* 1 */
    { 0x42, 0x61, 0x51, 0x49, 0x46 }, /* 2 */
    { 0x21, 0x41, 0x45, 0x4B, 0x31 }, /* 3 */
    { 0x18, 0x14, 0x12, 0x7F, 0x10 }, /* 4 */
    { 0x27, 0x45, 0x45, 0x45, 0x39 }, /* 5 */
    { 0x3C, 0x4A, 0x49, 0x49, 0x30 }, /* 6 */
    { 0x01, 0x71, 0x09, 0x05, 0x03 }, /* 7 */
    { 0x36, 0x49, 0x49, 0x49, 0x36 }, /* 8 */
    { 0x06, 0x49, 0x49, 0x29, 0x1E }, /* 9 */
    { 0x7E, 0x11, 0x11, 0x11, 0x7E }, /* A */
    { 0x7F, 0x49, 0x49, 0x49, 0x36 }, /* B */
    { 0x3E, 0x41, 0x41, 0x41, 0x22 }, /* C */
    { 0x7F, 0x41, 0x41, 0x22, 0x1C }, /* D */
    { 0x7F, 0x49, 0x49, 0x49, 0x41 }, /* E */
    { 0x7F, 0x09, 0x09, 0x09, 0x01 }, /* F */
    { 0x3E, 0x41, 0x49, 0x49, 0x7A }, /* G */
    { 0x7F, 0x08, 0x08, 0x08, 0x7F }, /* H */
    { 0x00, 0x41, 0x7F, 0x41, 0x00 }, /* I */
    { 0x20, 0x40, 0x41, 0x3F, 0x01 }, /* J */
    { 0x7F, 0x08, 0x14, 0x22, 0x41 }, /* K */
    { 0x7F, 0x40, 0x40, 0x40, 0x40 }, /* L */
    { 0x7F, 0x02, 0x0C, 0x02, 0x7F }, /* M */
    { 0x7F, 0x04, 0x08, 0x10, 0x7F }, /* N */
    { 0x3E, 0x41, 0x41, 0x41, 0x3E }, /* O */
    { 0x7F, 0x09, 0x09, 0x09, 0x06 }, /* P */
    { 0x3E, 0x41, 0x51, 0x21, 0x5E }, /* Q */
    { 0x7F, 0x09, 0x19, 0x29, 0x46 }, /* R */
    { 0x46, 0x49, 0x49, 0x49, 0x31 }, /* S */
    { 0x01, 0x01, 0x7F, 0x01, 0x01 }, /* T */
    { 0x3F, 0x40, 0x40, 0x40, 0x3F }, /* U */
    { 0x1F, 0x20, 0x40, 0x20, 0x1F }, /* V */
    { 0x3F, 0x40, 0x38, 0x40, 0x3F }, /* W */
    { 0x63, 0x14, 0x08, 0x14, 0x63 }, /* X */
    { 0x07, 0x08, 0x70, 0x08, 0x07 }, /* Y */
    { 0x61, 0x51, 0x49, 0x45, 0x43 }, /* Z */
    { 0x00, 0x60, 0x60, 0x00, 0x00 }, /* . */
    { 0x20, 0x10, 0x08, 0x04, 0x02 }, /* / */
    { 0x08, 0x08, 0x08, 0x08, 0x08 }, /* - */
    { 0x08, 0x08, 0x3E, 0x08, 0x08 }, /* + */
    { 0x00, 0x36, 0x36, 0x00, 0x00 }, /* : */
};

static void LCD_Pixel(uint16_t X, uint16_t Y, uint8_t Signal)
{
    uint32_t Index;
    uint32_t Shift;

    if ((X >= ST7789_WIDTH) || (Y >= ST7789_HEIGHT))
    {
        return;
    }

    Index = (uint32_t)Y * LCD_ROW_BYTES * 4U + X;
    Shift = (Index & 3U) * 2U;
    Frame_New[Index >> 2U] |= (uint8_t)(Signal << Shift);
}

static void LCD_Text(uint16_t X, uint16_t Y, const char *Text,
                     uint8_t Scale, uint8_t Signal)
{
    uint16_t Cursor = X;

    while (*Text != '\0')
    {
        const uint8_t *Glyph = Font_Cols[0];

        for (uint32_t n = 0U; n < sizeof(Font_Chars) - 1U; n++)
        {
            if (Font_Chars[n] == *Text)
            {
                Glyph = Font_Cols[n];
                break;
            }
        }

        for (uint8_t dx = 0U; dx < 5U; dx++)
        {
            for (uint8_t dy = 0U; dy < 7U; dy++)
            {
                if ((Glyph[dx] & (1U << dy)) == 0U)
                {
                    continue;
                }

                for (uint8_t sx = 0U; sx < Scale; sx++)
                {
                    for (uint8_t sy = 0U; sy < Scale; sy++)
                    {
                        LCD_Pixel(Cursor + (uint16_t)dx * Scale + sx,
                                  Y + (uint16_t)dy * Scale + sy,
                                  Signal);
                    }
                }
            }
        }

        Cursor += (uint16_t)6U * Scale;
        Text++;
    }
}

static void LCD_Number(float Value, uint8_t Decimals, char *Text)
{
    char Digits[6];
    uint32_t Scaled;
    uint32_t Whole;
    uint32_t Factor = (Decimals == 3U) ? 1000U : ((Decimals == 2U) ? 100U : 10U);
    uint8_t Count = 0U;
    uint8_t Offset = 0U;

    if (!__builtin_isfinite(Value))
    {
        strcpy(Text, "N/A");
        return;
    }

    if (Value < 0.0f)
    {
        Text[Offset++] = '-';
        Value = -Value;
    }

    if (Value >= 10000.0f)
    {
        strcpy(Text, "OVR");
        return;
    }

    Scaled = (uint32_t)(Value * (float)Factor + 0.5f);
    Whole = Scaled / Factor;

    do
    {
        Digits[Count++] = (char)('0' + (Whole % 10U));
        Whole /= 10U;
    } while ((Whole != 0U) && (Count < sizeof(Digits)));

    while (Count != 0U)
    {
        Text[Offset++] = Digits[--Count];
    }

    Text[Offset++] = '.';
    Scaled %= Factor;
    for (uint8_t n = Decimals; n > 0U; n--)
    {
        Factor /= 10U;
        Text[Offset++] = (char)('0' + (Scaled / Factor));
        Scaled %= Factor;
    }
    Text[Offset] = '\0';
}

static const char *LCD_Mode_Name(Motor_Mode_e Mode)
{
    switch (Mode)
    {
        case TORQUE:          return "TORQUE";
        case SPEED:           return "SPEED";
        case POSITION:        return "POSITION";
        case OPEN_LOOP:       return "OPEN LOOP";
        case IDENT:           return "IDENT";
        case SENSORLESS_SPEED:return "SENSORLESS";
        case SENSORLESS_SPEED_KNOWN_ANGLE: return "SL KNOWN";
        case PHASE_SEARCH:    return "PHASE";
        default:              return "UNKNOWN";
    }
}

static const char *LCD_Mode_Unit(Motor_Mode_e Mode)
{
    switch (Mode)
    {
        case TORQUE:          return "A";
        case POSITION:        return "TURNS";
        case SPEED:
        case SENSORLESS_SPEED:
        case SENSORLESS_SPEED_KNOWN_ANGLE:
        case OPEN_LOOP:       return "RAD/S";
        default:              return "";
    }
}

static const char *LCD_State_Name(Motor_State_e State)
{
    switch (State)
    {
        case DISABLED: return "OFF";
        case ENABLED:  return "READY";
        case RUN:      return "RUN";
        default:       return "OFF";
    }
}

static bool LCD_Feedback_Valid(void)
{
    return (Encoder.Ready != 0U) &&
           (Encoder.Fault == 0U) &&
           (Mechanical_ESO.Para.Valid != 0U);
}

static void LCD_Sample(void)
{
    Motor_Mode_e Mode = Motor_Mode_Get();
    float Ref;
    float Fbk;

    if (Mode != Display_Mode)
    {
        Display_Mode = Mode;
        History_Head = 0U;
        History_Count = 0U;
        Scale_Min = -1.0f;
        Scale_Max = 1.0f;
    }

    switch (Mode)
    {
        case TORQUE:
            Ref = Motor_Plot_Iq_Ref;
            Fbk = Motor_Run.Iq;
            break;

        case SPEED:
        case SENSORLESS_SPEED:
        case SENSORLESS_SPEED_KNOWN_ANGLE:
        case OPEN_LOOP:
            Ref = Motor_Plot_Wm_Ref;
            Fbk = Motor_Plot_Wm;
            if (((Mode == SPEED) || (Mode == OPEN_LOOP)) &&
                !LCD_Feedback_Valid())
            {
                Fbk = __builtin_nanf("");
            }
            break;

        case POSITION:
        {
            float End = (float)Motor_Cmd.Position_Target.Turn +
                        Motor_Cmd.Position_Target.Theta / TWO_PI_F;
            bool Running = (Motor_State_Get() == RUN);

            Ref = Motor_Plot_Position_Ref;
            Fbk = ((Encoder.Ready != 0U) && (Encoder.Fault == 0U)) ?
                  Motor_Plot_Position : __builtin_nanf("");
            if ((History_Count == 0U) || (End != Pos_End) || (Running && !Pos_Run))
            {
                Pos_Start = __builtin_isfinite(Fbk) ? Fbk : Ref;
                Pos_End = End;
            }
            Pos_Run = Running;
            break;
        }

        default:
            Ref = __builtin_nanf("");
            Fbk = __builtin_nanf("");
            break;
    }

    History_Ref[History_Head] = Ref;
    History_Fbk[History_Head] = Fbk;
    History_Head = (uint16_t)((History_Head + 1U) & (LCD_HISTORY_LEN - 1U));

    if (History_Count < LCD_HISTORY_LEN)
    {
        History_Count++;
    }
}

static void LCD_Scale_Run(void)
{
    float Start = 0.0f;
    float End;
    float Min;
    float Max;
    float Span;
    float Margin;

    switch (Display_Mode)
    {
        case TORQUE:
        {
            float Kt = 1.5f * (float)Motor_Para.Pp * Motor_Para.Flux;
            End = (__builtin_isfinite(Kt) && (Kt > 0.0f)) ?
                  Motor_User_To_Internal(Motor_Cmd.Te_Target) / Kt : 0.0f;
            break;
        }
        case SPEED:
        case SENSORLESS_SPEED:
        case SENSORLESS_SPEED_KNOWN_ANGLE:
        case OPEN_LOOP:
            End = Motor_Cmd.Wm_Target;
            break;
        case POSITION:
            Start = Pos_Start;
            End = Pos_End;
            break;
        default:
            return;
    }

    if (!__builtin_isfinite(Start) || !__builtin_isfinite(End))
    {
        return;
    }

    Min = (Start < End) ? Start : End;
    Max = (Start > End) ? Start : End;
    Span = Max - Min;
    /* A zero command/distance still needs a nonzero axis: A, turns, rad/s. */
    if (Span == 0.0f)
    {
        Span = (Display_Mode == TORQUE) ? 0.5f : ((Display_Mode == POSITION) ? 0.1f : 2.0f);
        Min -= Span * 0.5f;
        Max += Span * 0.5f;
    }

    /* Command span / 60%: each margin is 20% of the complete axis. */
    Margin = Span / 3.0f;
    Min -= Margin;
    Max += Margin;
    Scale_Min = Min;
    Scale_Max = Max;

    /* Only visible feedback can enlarge the axis; old references cannot. */
    for (uint16_t n = 0U; (n < History_Count) && (n < LCD_WINDOW_COUNT); n++)
    {
        uint16_t Index = (uint16_t)((History_Head - n - 1U) & (LCD_HISTORY_LEN - 1U));
        float Value = History_Fbk[Index];

        if (__builtin_isfinite(Value))
        {
            if (Value < Scale_Min)
            {
                Scale_Min = Value;
            }
            if (Value > Scale_Max)
            {
                Scale_Max = Value;
            }
        }
    }
    if (Scale_Min < Min)
    {
        Scale_Min -= Margin;
    }
    if (Scale_Max > Max)
    {
        Scale_Max += Margin;
    }
}

static bool LCD_History_Get(uint16_t Age, float *Ref, float *Fbk)
{
    uint16_t Index;

    if (Age >= History_Count)
    {
        return false;
    }

    Index = (uint16_t)((History_Head - Age - 1U) & (LCD_HISTORY_LEN - 1U));
    *Ref = History_Ref[Index];
    *Fbk = History_Fbk[Index];
    return true;
}

static uint16_t LCD_Graph_Y(float Value)
{
    float Ratio;

    Ratio = (Value - Scale_Min) / (Scale_Max - Scale_Min);
    if (Ratio < 0.0f)
    {
        Ratio = 0.0f;
    }
    if (Ratio > 1.0f)
    {
        Ratio = 1.0f;
    }

    return (uint16_t)(LCD_GRAPH_Y + LCD_GRAPH_H - 1U -
                      (uint16_t)(Ratio * (LCD_GRAPH_H - 1U) + 0.5f));
}

static void LCD_Line(uint16_t X0, uint16_t Y0,
                     uint16_t X1, uint16_t Y1, uint8_t Signal)
{
    int32_t X = X0;
    int32_t Y = Y0;
    int32_t Dx = (int32_t)X1 - X0;
    int32_t Dy = ((int32_t)Y1 > (int32_t)Y0) ?
                 (int32_t)Y0 - Y1 : (int32_t)Y1 - Y0;
    int32_t Sy = (Y0 < Y1) ? 1 : -1;
    int32_t Err = Dx + Dy;

    while (1)
    {
        LCD_Pixel((uint16_t)X, (uint16_t)Y, Signal);
        if ((X == X1) && (Y == Y1))
        {
            break;
        }

        int32_t E2 = 2 * Err;
        if (E2 >= Dy)
        {
            Err += Dy;
            X++;
        }
        if (E2 <= Dx)
        {
            Err += Dx;
            Y += Sy;
        }
    }
}

static void LCD_Render(void)
{
    const char *Title = LCD_Mode_Name(Display_Mode);
    Motor_State_e State = Motor_State_Get();
    float Ref = __builtin_nanf("");
    float Fbk = __builtin_nanf("");
    float Prev_Ref = 0.0f;
    float Prev_Fbk = 0.0f;
    bool Ref_Valid_Pre = false;
    bool Fbk_Valid_Pre = false;
    char Ref_Text[16];
    char Fbk_Text[16];

    memset(Frame_New, 0, sizeof(Frame_New));

    LCD_Scale_Run();

    if (History_Count != 0U)
    {
        (void)LCD_History_Get(0U, &Ref, &Fbk);
    }

    uint8_t Decimals = (Display_Mode == POSITION) ? 3U : ((Display_Mode == TORQUE) ? 2U : 1U);
    LCD_Number(Ref, Decimals, Ref_Text);
    LCD_Number(Fbk, Decimals, Fbk_Text);

    LCD_Text(6U, 4U, Title, 1U, LCD_REF);
    LCD_Text(120U, 4U, LCD_Mode_Unit(Display_Mode), 1U, LCD_REF);
    LCD_Text(192U, 4U, LCD_State_Name(State), 1U, LCD_FBK);
    LCD_Text(6U, 18U, "REF", 1U, LCD_REF);
    LCD_Text(30U, 18U, Ref_Text, 1U, LCD_REF);
    LCD_Text(130U, 18U, "FBK", 1U, LCD_FBK);
    LCD_Text(154U, 18U, Fbk_Text, 1U, LCD_FBK);

    for (uint16_t x = 0U; x < ST7789_WIDTH; x++)
    {
        float Cur_Ref;
        float Cur_Fbk;
        uint16_t Age = (uint16_t)(((uint32_t)(ST7789_WIDTH - 1U - x) *
                                   (LCD_WINDOW_COUNT - 1U)) /
                                  (ST7789_WIDTH - 1U));

        if (!LCD_History_Get(Age, &Cur_Ref, &Cur_Fbk))
        {
            Ref_Valid_Pre = false;
            Fbk_Valid_Pre = false;
            continue;
        }

        if (__builtin_isfinite(Cur_Ref))
        {
            if (Ref_Valid_Pre)
            {
                LCD_Line(x - 1U, LCD_Graph_Y(Prev_Ref),
                         x, LCD_Graph_Y(Cur_Ref), LCD_REF);
            }
            else
            {
                LCD_Pixel(x, LCD_Graph_Y(Cur_Ref), LCD_REF);
            }
        }

        if (__builtin_isfinite(Cur_Fbk))
        {
            if (Fbk_Valid_Pre)
            {
                LCD_Line(x - 1U, LCD_Graph_Y(Prev_Fbk),
                         x, LCD_Graph_Y(Cur_Fbk), LCD_FBK);
            }
            else
            {
                LCD_Pixel(x, LCD_Graph_Y(Cur_Fbk), LCD_FBK);
            }
        }

        Ref_Valid_Pre = __builtin_isfinite(Cur_Ref);
        Fbk_Valid_Pre = __builtin_isfinite(Cur_Fbk);
        Prev_Ref = Cur_Ref;
        Prev_Fbk = Cur_Fbk;
    }
}

static uint8_t LCD_Pixel_Get(uint16_t X, uint16_t Y)
{
    uint32_t Index = (uint32_t)Y * LCD_ROW_BYTES * 4U + X;
    return (Frame_New[Index >> 2U] >> ((Index & 3U) * 2U)) & 3U;
}

static uint16_t LCD_Color(uint16_t X, uint16_t Y)
{
    switch (LCD_Pixel_Get(X, Y))
    {
        case LCD_REF: return 0x17D4U; /* cyan */
        case LCD_FBK: return 0xFD20U; /* amber */
        case 3U:      return 0xFFFFU;
        default:
            if ((Y >= LCD_GRAPH_Y) && (Y < LCD_GRAPH_Y + LCD_GRAPH_H))
            {
                return ((X % 40U == 0U) ||
                        ((Y - LCD_GRAPH_Y) % 26U == 0U)) ? 0x2945U : 0x0841U;
            }
            return 0x10A2U;
    }
}

static void LCD_Dirty_Calc(void)
{
    for (uint16_t ty = 0U; ty < LCD_TILE_ROWS; ty++)
    {
        uint16_t Mask = 0U;

        for (uint16_t tx = 0U; tx < LCD_TILE_COLS; tx++)
        {
            bool Changed = !Frame_Valid || (LCD_Test == 2U);

            if (!Changed)
            {
                for (uint16_t row = 0U; (row < LCD_TILE_H) &&
                     (ty * LCD_TILE_H + row < ST7789_HEIGHT); row++)
                {
                    uint32_t Offset = (uint32_t)(ty * LCD_TILE_H + row) *
                                      LCD_ROW_BYTES + tx * LCD_TILE_W / 4U;
                    if (memcmp(&Frame_Old[Offset], &Frame_New[Offset], LCD_TILE_W / 4U) != 0)
                    {
                        Changed = true;
                        break;
                    }
                }
            }

            if (Changed)
            {
                Mask |= (uint16_t)(1U << tx);
            }
        }

        Dirty[ty] = Mask;
    }
    Flush_Row = 0U;
    Frame_Bytes = 0U;
    Flush_Active = true;
}

static void LCD_Flush_Pump(void)
{
    if (!Flush_Active || !ST7789_Ready())
    {
        return;
    }

    for (uint16_t ty = Flush_Row; ty < LCD_TILE_ROWS; ty++)
    {
        uint16_t Mask = Dirty[ty];
        uint16_t tx = 0U;

        while (tx < LCD_TILE_COLS)
        {
            uint16_t First;
            uint16_t Width;
            uint16_t Height = ST7789_HEIGHT - ty * LCD_TILE_H;
            uint16_t Pixels = 0U;
            uint16_t Sent_Mask = 0U;

            if ((Mask & (1U << tx)) == 0U)
            {
                tx++;
                continue;
            }

            First = tx;
            while ((tx < LCD_TILE_COLS) && ((Mask & (1U << tx)) != 0U))
            {
                Sent_Mask |= (uint16_t)(1U << tx);
                tx++;
            }

            Width = (uint16_t)((tx - First) * LCD_TILE_W);
            if (Height > LCD_TILE_H)
            {
                Height = LCD_TILE_H;
            }
            for (uint16_t y = (uint16_t)(ty * LCD_TILE_H);
                 y < (uint16_t)(ty * LCD_TILE_H + Height); y++)
            {
                for (uint16_t x = (uint16_t)(First * LCD_TILE_W);
                     x < (uint16_t)(First * LCD_TILE_W + Width); x++)
                {
                    uint16_t Color = LCD_Color(x, y);
                    Tx_Buffer[Pixels++] = (uint8_t)(Color >> 8U);
                    Tx_Buffer[Pixels++] = (uint8_t)Color;
                }
            }

            if (!ST7789_Write((uint16_t)(First * LCD_TILE_W),
                              (uint16_t)(ty * LCD_TILE_H),
                              Width, Height, Tx_Buffer))
            {
                return;
            }

            Dirty[ty] &= (uint16_t)~Sent_Mask;
            Frame_Bytes += Pixels;
            Flush_Row = (uint8_t)ty;
            return;
        }
    }

    /* ST7789_Ready above also confirms completion of the final rectangle. */
    if (Frame_Bytes != 0U)
    {
        LCD_Time.Byte_Cnt += Frame_Bytes;
        LCD_Time.Flush_Ticks = (uint32_t)tx_time_get() - Frame_Tick;
        if (LCD_Time.Flush_Ticks > LCD_Time.Flush_Max)
        {
            LCD_Time.Flush_Max = LCD_Time.Flush_Ticks;
        }
        LCD_Time.Frame_Cnt++;
    }
    memcpy(Frame_Old, Frame_New, sizeof(Frame_Old));
    Frame_Valid = true;
    Flush_Active = false;
}

bool LCD_Wave_Init(uint32_t Tick)
{
    LCD_Active = ST7789_Init();
    Sample_Tick = Tick - LCD_SAMPLE_TICKS;
    Frame_Tick = Tick - LCD_FRAME_TICKS;
    return LCD_Active;
}

void LCD_Wave_Run(uint32_t Tick)
{
    if (!LCD_Active)
    {
        return;
    }

    if ((uint32_t)(Tick - Sample_Tick) >= LCD_SAMPLE_TICKS)
    {
        Sample_Tick = Tick;
        LCD_Sample();
    }

    if (Flush_Active)
    {
        LCD_Flush_Pump();
        if (Flush_Active)
        {
            return;
        }
    }

    if (LCD_Test == 1U)
    {
        return;
    }

    if ((uint32_t)(Tick - Frame_Tick) >= LCD_FRAME_TICKS)
    {
        uint32_t T0 = DWT->CYCCNT;

        Frame_Tick = Tick;
        if (LCD_Test == 3U)
        {
            memset(Frame_New, 0, sizeof(Frame_New));
            LCD_Line(0U, 0U, ST7789_WIDTH - 1U, 0U, LCD_REF);
            LCD_Line(0U, ST7789_HEIGHT - 1U, ST7789_WIDTH - 1U, ST7789_HEIGHT - 1U, LCD_FBK);
            LCD_Line(0U, 0U, 0U, ST7789_HEIGHT - 1U, LCD_REF);
            LCD_Line(ST7789_WIDTH - 1U, 0U, ST7789_WIDTH - 1U, ST7789_HEIGHT - 1U, LCD_FBK);
            LCD_Text(4U, 4U, "TL", 1U, LCD_REF);
            LCD_Text(ST7789_WIDTH - 16U, 4U, "TR", 1U, LCD_FBK);
            LCD_Text(4U, ST7789_HEIGHT - 11U, "BL", 1U, LCD_REF);
            LCD_Text(ST7789_WIDTH - 16U, ST7789_HEIGHT - 11U, "BR", 1U, LCD_FBK);
            LCD_Text(76U, 60U, "240X135", 2U, LCD_REF);
        }
        else
        {
            LCD_Render();
        }
        LCD_Time.Render_Cyc = DWT->CYCCNT - T0;
        if (LCD_Time.Render_Cyc > LCD_Time.Render_Max)
        {
            LCD_Time.Render_Max = LCD_Time.Render_Cyc;
        }
        LCD_Dirty_Calc();
        LCD_Flush_Pump();
    }
}
