#include "AxDr_Plot.h"


static AxDr_Plot_Group_T Plot_Group[2] = {0};


static uint8_t Plot_Var_Valid(uint8_t Group, uint16_t Var_ID)
{
    switch (Var_ID)
    {
        case 0x0001: /* Ia */
        case 0x0002: /* Ib */
        case 0x0003: /* Ic */
        case 0x0010: /* Id */
        case 0x0011: /* Iq */
        case 0x0012: /* Ud */
        case 0x0013: /* Uq */
        case 0x0014: /* Theta_e */
        case 0x0101: /* Theta_m */
        case 0x0102: /* Wm */
            return 1U;

        case 0x0004: /* Vbus */
            return (Group == AXDR_PLOT_NORMAL) ? 1U : 0U;

        default:
            return 0U;
    }
}


AxDr_Status_e AxDr_Plot_Config(uint8_t Group,
                               uint8_t Config_ID,
                               const uint16_t *Var,
                               uint8_t Count)
{
    uint8_t Max_Ch;
    AxDr_Plot_Group_T *Plot;

    if (Group > AXDR_PLOT_NORMAL)
    {
        return AXDR_ERR_CONFIG;
    }

    Plot = &Plot_Group[Group];

    if (Plot->Run != 0U)
    {
        return AXDR_ERR_STATE;
    }

    Max_Ch = (Group == AXDR_PLOT_FAST)
           ? AXDR_FAST_MAX_CH
           : AXDR_NORMAL_MAX_CH;

    if ((Count == 0U) || (Count > Max_Ch))
    {
        return AXDR_ERR_CONFIG;
    }

    for (uint8_t n = 0U; n < Count; n++)
    {
        if (Plot_Var_Valid(Group, Var[n]) == 0U)
        {
            return AXDR_ERR_VAR_ID;
        }
    }

    Plot->Config_ID = Config_ID;
    Plot->Count = Count;

    for (uint8_t n = 0U; n < Count; n++)
    {
        Plot->Var[n] = Var[n];
    }

    Plot->Valid = 1U;

    return AXDR_OK;
}


AxDr_Status_e AxDr_Plot_Start(uint8_t Group_Mask)
{
    if ((Group_Mask == 0U) ||
        ((Group_Mask & ~(AXDR_PLOT_FAST_MASK | AXDR_PLOT_NORMAL_MASK)) != 0U))
    {
        return AXDR_ERR_CONFIG;
    }

    if (((Group_Mask & AXDR_PLOT_FAST_MASK) != 0U) &&
        (Plot_Group[AXDR_PLOT_FAST].Valid == 0U))
    {
        return AXDR_ERR_CONFIG;
    }

    if (((Group_Mask & AXDR_PLOT_NORMAL_MASK) != 0U) &&
        (Plot_Group[AXDR_PLOT_NORMAL].Valid == 0U))
    {
        return AXDR_ERR_CONFIG;
    }

    if ((Group_Mask & AXDR_PLOT_FAST_MASK) != 0U)
    {
        Plot_Group[AXDR_PLOT_FAST].Run = 1U;
    }

    if ((Group_Mask & AXDR_PLOT_NORMAL_MASK) != 0U)
    {
        Plot_Group[AXDR_PLOT_NORMAL].Run = 1U;
    }

    return AXDR_OK;
}


AxDr_Status_e AxDr_Plot_Stop(uint8_t Group_Mask)
{
    if ((Group_Mask == 0U) ||
        ((Group_Mask & ~(AXDR_PLOT_FAST_MASK | AXDR_PLOT_NORMAL_MASK)) != 0U))
    {
        return AXDR_ERR_CONFIG;
    }

    if ((Group_Mask & AXDR_PLOT_FAST_MASK) != 0U)
    {
        Plot_Group[AXDR_PLOT_FAST].Run = 0U;
    }

    if ((Group_Mask & AXDR_PLOT_NORMAL_MASK) != 0U)
    {
        Plot_Group[AXDR_PLOT_NORMAL].Run = 0U;
    }

    return AXDR_OK;
}


const AxDr_Plot_Group_T *AxDr_Plot_Group_Get(uint8_t Group)
{
    if (Group > AXDR_PLOT_NORMAL)
    {
        return 0;
    }

    return &Plot_Group[Group];
}
