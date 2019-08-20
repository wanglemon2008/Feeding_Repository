#include <string.h>
#include "FreeRTOS.h"

#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "event_groups.h"
#include "trace.h"

#include "typedef.h"
#include "Eeprom.h"
#include "Control.h"
#include "HalApi.h"
#include "IndoorCtrl.h" 
#include "ExvDrive.h"
//#include "Monitor.h"
#include "flash.h"
#include "OutdoorCtrl.h"

#define LOCAL_TRACE      1
#define PRODUCTION_TYPE	 0

#define INCTRL_EVENT_CTRL_LOOP		0x00000001
#define INCTRL_EVENT_USER_DATA		0x00000002
#define INCTRL_EVENT_ODC1_DATA		0x00000100
#define INCTRL_EVENT_ODC2_DATA		0x00000200
#define INCTRL_EVENT_ODC3_DATA		0x00000400
#define INCTRL_EVENT_ODC4_DATA		0x00000800

#define INCTRL_EVENT_ODC_DATAS		( INCTRL_EVENT_ODC1_DATA | INCTRL_EVENT_ODC2_DATA | \
									  INCTRL_EVENT_ODC3_DATA | INCTRL_EVENT_ODC4_DATA )

#define INCTRL_EVENT_ALL			( INCTRL_EVENT_CTRL_LOOP | INCTRL_EVENT_USER_DATA | INCTRL_EVENT_ODC_DATAS )

#define INCTRL_TIMER_MS			( 250 )   		/* can be 1000ms,500ms,250ms,200ms, minimum is 200ms. */
#define INCTRL_TIMER_TICK		( pdMS_TO_TICKS( INCTRL_TIMER_MS ) )	/* ticks. */
#define INCTRL_TIMER_HZ			( 1000 / (INCTRL_TIMER_MS) )   			/* timer HZ. */

#define VAVLE_ON_AHEAD_MS		(60)

#define MAX_COMP_CNT			(32)
#define MAX_OUT_CTRL_CNT		(MAX_COMP_CNT / 2)  	/* 2 comps per outdoor controller. */

#define HMI_PUBLIC
/*========================== data type definitions. ==============================*/
enum 
{
	FAN_VALVE1_PWM = HAL_AO_PIN0,
	FAN_VALVE2_PWM,
	HUMIDIFIER_PWM,
	HEATER_PWM,
	FAN_PWM,
};

typedef void (*INCTRL_STATE_FUNC_PTR)(void);

/* used to save compressor run time (wear level) to EEPROM. Dbmapi.h */
typedef struct
{
	u32		VerNum;			/* 0 ~ 0x0FFFFFFF. */
	u32 	TotalWorkTime1; 
	u32 	TotalWorkTime2;	
	u32 	TotalWorkTime3; 
	u32 	TotalWorkTime4;	
	u32 	TotalWorkTime5; 
	u32 	TotalWorkTime6;	
	u32 	TotalWorkTime7; 
	u32 	TotalWorkTime8;	
	u32		CheckSum;		/* checksum of all other members. */
} NvAllCompDataT;

/* used to save System preheat time (wear level) to EEPROM. Dbmapi.h */
typedef struct
{
	u32		VerNo;			/* 0 ~ 0x0FFFFFFF. */ /* software version 0~0x0000000F*/ 
	u32 	SysPreHeatTime; 
	u32		CheckSum;		/* sum of all other members. */
} EepromSysDataT;

static EepromSysDataT		NvSysData;

typedef struct
{
	/* following items are received from outdoor controller. */
	u32			TotalWorkTime;		/* unit in minute. */
	bool		bCompExist;			/* 1: comp-0 existed, 0: not existed. */ 
    CompWorkModeT	CurCompMode;
	u16			CompError;
	u16			CompOnTime;			/* in SECOND unit. */
	s16			OutCoilTemp;   
    s16         EvaTemp;    
    s16         OutdoorTemp;
    s16         UnitOutTemp;
    u8          PreFrzHeaterOn;
	u16			CompCurrent;
	u16			ODFanCurrent;

	/* all other items are from indoor controller. */
	s16			InnerCoilTemp;

	/* inner coil temp controls. */
	u16			AvoidColdWindOnTime;	/* inner coil temp < 25C for 5s. */
	u16 		AvoidColdWindOffTime; 	/* inner coil temp >= 25C for 5s. */
	u16			NormalWindOnTime;		/* inner coil temp >= 35C for 5s. */

	/* out coil temp controls. */
	u16			DFLTDuration;		/* defrost low temp duration. <=-8C. */
	u16			DFMTDuration;		/* defrost mid temp duration. <=-4C. */
	u16			DFHTDuration;		/* defrost high temp duration. <=-2C. */
	u16			DFIntervalCnt;		/* defrost interval count. */
	u16			ResetCntrOnTime;	/* reset interval counter duration. >+3C for 150s. */
    
    /* unit out water tempreture */
    
    u16         UnitOut10sCnt;

	/* used for compressor mode commands.(OutCtrlCmdT) */
	CompWorkModeT	ReqCompMode;

	/* used for comp defrosting process. */
	bool		bReqDefrost;
	u16			DFWaitTime;
	u16			DFOnTime;
	u16			DFOffTime;
    
} CompStatusT;

typedef struct
{
	bool		bReqDefrost;
	u16			DFWaitTime;
	u16			DFOnTime;
	u16			DFOffTime;
} CompDefrostT;

typedef struct
{
	/* OUTPUT PIN state times */

	u16			IDFanOnTime;
	u16			IDFanOffTime;
    
    /* System Preheat times */
    u32			PreHeatTime;
    
    /* System power on times */
    u32         PowerOffTime;
   
	u8			PumpOn;   
	u8			FaultOn;
    u8          RunningOn;
    u8          AssHeaterOn;
    
	/* INPUT PIN state times */

	/* INPUT PIN state */
	u8			bRemoteOn   :    1;
	u8			Reserved    :    7;

	SysWorkModeT	CurWorkMode;		/* current work mode. */
	u16				ModeTime;			/* times in CurWorkMode. */
	bool			bReqPending;		/* new work mode request is pending. */
	SysWorkModeT	ReqWorkMode;		/* request work mode. */
	s16				TargetTemp;			/* from user SettingTemp, */
	bool            bSysInit;           /* 1: sys initial going, 0: sys initial over*/  
    u8              CompOnNum;
    u8              CompSum;
    
	s16 		OutdoorTemp;
    s16         DefrostEvaDif;
	u16			ODTLow18Duration;		/* outdoor temp <=18C duration. */

    u16         DefrostInterval;
    
    u16         PreFrzInterval;
    bool        EnterCompPreFrz;
    bool        EnterHeaterPreFrz;
    
	s16			RoomTemp;
    s16         BackWaterTemp;
    s16         OutWaterTemp;
    s16         PreFrzTemp;
    
	u16 		ODTSensorFaultOnTime;
	u16 		ODTSensorFaultOffTime;

	u16 		CtrlError;
	u8			OutError[MAX_OUT_CTRL_CNT];
	bool		bCompsFault;			/* comp with fault. */

} InCtrlStatusT;   /* describes a device. */

/*========================== common IO PIN definitions. ==============================*/

typedef struct 
{
	IoStateT	DiPin1;
	IoStateT	DiPin2;
	IoStateT	DiPin3;
	IoStateT	DiPin4;
	IoStateT	DiPin5;
	IoStateT	DiPin6;
	IoStateT	DiPin7;
	IoStateT	DiPin8;
} DiPinListT;

typedef struct 
{
	IoStateT	DoPin1;
	IoStateT	DoPin2;
	IoStateT	DoPin3;
	IoStateT	DoPin4;
	IoStateT	DoPin5;
	IoStateT	DoPin6;
	IoStateT	DoPin7;
	IoStateT	DoPin8;
	IoStateT	DoPin9;
	IoStateT	DoPin10;
	IoStateT	DoPin11;
	IoStateT	DoPin12;
	IoStateT	DoPin13;
	IoStateT	DoPin14;
    
    /* added according to the project */
    IoStateT	DoPin15;
    IoStateT	DoPin16;

} DoPinListT;

typedef struct 
{
	u16			Adc1;
	u16			Adc2;
	u16			Adc3;
	u16			Adc4;
	u16			Adc5;
	u16			Adc6;
	u16			Adc7;
	u16			Adc8;
	u16			Adc9;
} AiDataListT;

typedef struct 
{
	u16			Pwm1;   /* Used to control the output data. Duty */
	u16			Pwm2;
	u16			Pwm3;
	u16			Pwm4;
	u16			Pwm5;

	IoStateT	Pwm1Pin;	/* Used to record the ON/OFF state. PWM > 0: means ON, PWM == 0: means OFF. */
	IoStateT	Pwm2Pin;
	IoStateT	Pwm3Pin;
	IoStateT	Pwm4Pin;
	IoStateT	Pwm5Pin;

} AoDataListT;


/*========================== heat pump project IO PIN allocations. ==============================*/

#if 0
typedef struct 
{
	IoStateT	BackWaterFault;
	IoStateT	OutFault;
	IoStateT	PumpFault;
             
} IdcInPinListT;

typedef struct
{
	IoStateT	Fault;
	IoStateT	Running;
	IoStateT	AssHeater;
	IoStateT	PumpOn;  
    
    IoStateT    Heater1;
    IoStateT    Heater2;
    IoStateT    Heater3;

} IdcOutPinListT;
#endif

typedef struct
{
	u16			CurCtrlTemp1Adc;
	u16			CurCtrlTemp2Adc;
	u16			CurCtrlTemp3Adc;
       
} IdcInDataListT;


/*========================== variables definition. ==============================*/

static EventGroupHandle_t   InCtrlEventHandle;
static StaticEventGroup_t   InCtrlEventCb;

static TimerHandle_t        InCtrlTimerHandle;
static StaticTimer_t        InCtrlTimerCb;

static SemaphoreHandle_t    ParamSemHandle;
static StaticSemaphore_t    ParamSemCb;

static HmiUserParamT		UserParam;
static HmiDevParamPart1T	DevPart1Setting;
static HmiDevParamPart2T	DevPart2Setting;
static HmiDevParamPart3T	DevPart3Setting;

static InCtrlStatusT		InCtrl;

static CompStatusT			CompStatus[MAX_COMP_CNT];

static IdcInDataListT		IdcInDataList;
//static IdcInPinListT		IdcInPinList;
//static IdcOutPinListT		IdcOutPinList;

static OutCtrlRspT			OutCtrlRsp[MAX_OUT_CTRL_CNT];
//static u16 				OutCtrlCommCnt[MAX_OUT_CTRL_CNT];

static void 		InCtrlStopMode(void);
static void 		InCtrlManMode(void);
static void 		InCtrlAutoMode(void);

static const INCTRL_STATE_FUNC_PTR  StateFunc[] = {
											InCtrlStopMode,     /* it is for SYS_WM_STOP. */
											InCtrlManMode,     /* it is for SYS_WM_MAN. */
											InCtrlAutoMode,     /* it is for SYS_WM_AUTO. */
										};

static bool		InCtrlInitOk = FALSE;

static u8		UserParamDirtyFlag = 0;
static u8		SysParamDirtyFlag = 0;

/*========================== function definitions. ==============================*/

/*   ==================          I/O functions.              ================    */

#include "Ctrl.c"

static void ReadAcInput(void)
{
	u8	NewState;

#if 0
	/* read input PIN state to PIN buffer. */
	NewState = HalReadGpi(CTRL_WATER_PIN);
	SetIoState(&IdcInPinList.PumpFault, NewState);

	NewState = HalReadGpi(CTRL_PUMP_OVERLOAD_PIN);
	SetIoState(&IdcInPinList.BackWaterFault, NewState);
 
    NewState = HalReadGpi(CTRL_REMOTE_PIN);
	SetIoState(&IdcInPinList.OutFault, NewState);    
    
	/* read input data to Data buffer. */
    IdcInDataList.SysEnterTempAdc       = HalReadAdc(SYS_ENTER_WATER_TEMP);

	/* ====converts the PIN state to state variable.===== */

	/* ====convert the read data and store to variables.==== */   
    InCtrl.BackWaterTemp = NtcAdc2Temp(IdcInDataList.SysEnterTempAdc);
#endif

}

static void WriteAcOutput(void)
{
#if 0  
	/* store output-PIN-state to PIN buffer. */    
	SetIoState(&IdcOutPinList.Fault, (u8)(InCtrl.FaultOn == DEV_ON) );
	SetIoState(&IdcOutPinList.Running, (u8)(InCtrl.RunningOn == DEV_ON) );
	SetIoState(&IdcOutPinList.AssHeater, (u8)(InCtrl.AssHeaterOn == DEV_ON) );
	SetIoState(&IdcOutPinList.PumpOn, (u8)(InCtrl.PumpOn == DEV_ON) );
    
	/* Flush the PIN buffer to output PINs. */
    HalWriteGpo(CTRL_ERROR_PIN, IdcOutPinList.Fault.PinState);
	HalWriteGpo(CTRL_RUNING_PIN, IdcOutPinList.Running.PinState);        
	HalWriteGpo(CTRL_ASS_HEATER_PIN, IdcOutPinList.AssHeater.PinState);        
	HalWriteGpo(CTRL_PUMP_PIN, IdcOutPinList.PumpOn.PinState);
#endif      
}

#if 0
/*   ==========================      Pump function.          ===============================     */

static void CheckPreFrzInterval(void)
{
#if 0
    if(InCtrl.CtrlError & CTRL_ERR_OUT_SENSOR) /* outdoor temp error */
      InCtrl.PreFrzInterval = SysPreFrzParam.PreFrzInterval2;
    
    if(InCtrl.OutdoorTemp >= 0)
    {
        InCtrl.PreFrzInterval = SysPreFrzParam.PreFrzInterval1;
    }
    else
    {
        InCtrl.PreFrzInterval = SysPreFrzParam.PreFrzInterval2;
    } 
#endif
}

static s16  MinUnitOutTemp(void)
{
    u8		i;
    s16     MinValue = 0;
	
	for(i = 0; i < MAX_COMP_CNT; i++)
	{
		if( CompStatus[i].bCompExist && (CompStatus[i].CompError == 0) )
		{
			if(MinValue > CompStatus[i].UnitOutTemp)
            {
                MinValue = CompStatus[i].UnitOutTemp;
            }
				
		}
	}

	return MinValue;
}

static void CheckPreFrzTemp(void)
{
    if(InCtrl.BackWaterTemp <= InCtrl.OutWaterTemp)
    {
        InCtrl.PreFrzTemp = InCtrl.BackWaterTemp;
    }
    else
    {
        InCtrl.PreFrzTemp = InCtrl.OutWaterTemp;
    }
    
    if((InCtrl.CtrlError & INCTRL_ERR_SYS_OUT_SENSOR)&&(InCtrl.CtrlError & INCTRL_ERR_SYS_BACK_SENSOR))
    {
        InCtrl.PreFrzTemp = MinUnitOutTemp();
    }
}

/*   ==========================   COMPRESSOR functions.   =========================      */

static bool IsPreHeatComp(void)
{
    if(SysParam.PreHeatTime<=0)
    {
        return TRUE;
    }
    else if(InCtrl.OutdoorTemp<150)
    {
        /* power shut down during the preheat process */
      
        if(InCtrl.PowerOffTime>30*60*INCTRL_TIMER_HZ)
        {
            return TRUE;
        }
        else
        {
            if(InCtrl.PreHeatTime <480*60*INCTRL_TIMER_HZ)
            {
                return FALSE;
            }
        }
        
        if(InCtrl.PreHeatTime > 480*60*INCTRL_TIMER_HZ)
        {
            return TRUE;
        }
        else 
        {
            return FALSE;
        }
    }   
    
    return FALSE;
}

static void CheckCompTemp(CompStatusT * pComp, SysWorkModeT WorkMode)
{
	if( !pComp->bCompExist )
	{
		return;
	}

	/* inner coil temp handle for cool mode. */
	if( (WorkMode == SYS_WM_COOL) && (pComp->InnerCoilTemp != HAL_TEMP_ERR) )
	{
	}

	/* inner coil temp handle for heat mode. */
	if( (WorkMode == SYS_WM_HEAT) && (pComp->InnerCoilTemp != HAL_TEMP_ERR) )
	{
		if( pComp->InnerCoilTemp < 250 )	/* < 25C */
		{
			pComp->AvoidColdWindOffTime = 0;
			pComp->NormalWindOnTime    = 0;

			if(pComp->AvoidColdWindOnTime < 0xFF00)
				pComp->AvoidColdWindOnTime++;
		}
		else if( pComp->InnerCoilTemp < 350 )	/* 25C ~ 35C */
		{
			pComp->AvoidColdWindOnTime = 0;
			pComp->NormalWindOnTime   = 0;

			if(pComp->AvoidColdWindOffTime < 0xFF00)
				pComp->AvoidColdWindOffTime++;
		}
		else /* >= 35C */
		{
			pComp->AvoidColdWindOnTime  = 0;
			pComp->AvoidColdWindOffTime = 0;

			if(pComp->NormalWindOnTime < 0xFF00)
				pComp->NormalWindOnTime++;
		}
	}

	/* out coil temp handle for heat mode. */
	if( (WorkMode == SYS_WM_HEAT) && (pComp->OutCoilTemp != HAL_TEMP_ERR) )
	{
		/* defrost interval counter. */
		if( (pComp->OutCoilTemp <= 30) && (pComp->CompOnTime >= 120) )		/* <= 3C for 120s. */
		{
			if(pComp->DFIntervalCnt < 0xFF00)
				pComp->DFIntervalCnt++;
		}

		/* check to clear defrost interval counter. */
		if(pComp->OutCoilTemp > 30)		/* > 3C */
		{
			if(pComp->ResetCntrOnTime < 0xFF00)
				pComp->ResetCntrOnTime++;

			if(pComp->ResetCntrOnTime >= (150 * INCTRL_TIMER_HZ)) /* 150s of time. */
			{
				pComp->DFIntervalCnt = 0;
			}
		}
		else
		{
			pComp->ResetCntrOnTime = 0;
		}

		if(pComp->OutCoilTemp <= -80) 	/* <= -8C */
		{
			if(pComp->DFLTDuration < 0xFF00)
				pComp->DFLTDuration++;
			
			if(pComp->DFMTDuration < 0xFF00)
				pComp->DFMTDuration++;
			
			if(pComp->DFHTDuration < 0xFF00)
				pComp->DFHTDuration++;
		}
		else if(pComp->OutCoilTemp <= -40) 	/* <= -4C, & > -8C */
		{
			pComp->DFLTDuration = 0;

			if(pComp->DFMTDuration < 0xFF00)
				pComp->DFMTDuration++;
			
			if(pComp->DFHTDuration < 0xFF00)
				pComp->DFHTDuration++;
		}
		else if(pComp->OutCoilTemp <= -20) 	/* <= -2C, & > -4C */
		{
			pComp->DFLTDuration = 0;
			pComp->DFMTDuration = 0;

			if(pComp->DFHTDuration < 0xFF00)
				pComp->DFHTDuration++;
		}
		else /* > -2C */
		{
			pComp->DFLTDuration = 0;
			pComp->DFMTDuration = 0;
			pComp->DFHTDuration = 0;
		}
	}

}

static bool IsCompDefrostRequired(CompStatusT * pComp)
{
    /* will add the two comp into defrost status at the same time ??? */
  
  
  
  
	if( !pComp->bCompExist )
	{
		return FALSE;
	}

	if(pComp->CompError & (~(COMP_ERR_LP)))
	{
		return FALSE;
	}

	if(pComp->CurCompMode == COMP_WM_STOP)
	{
		pComp->DFLTDuration = 0;
		pComp->DFMTDuration = 0;
		pComp->DFHTDuration = 0;
		pComp->DFIntervalCnt = 0;
		pComp->ResetCntrOnTime = 0;

		return FALSE;
	}

	/* out coil temp <= 3C, and compressor run for 2 hours. */
	if(pComp->DFIntervalCnt >= (7200 * INCTRL_TIMER_HZ))  /* Td >= 2 hours. */
	{
		if(pComp->DFHTDuration >= (180 * INCTRL_TIMER_HZ)) /* -2C for 3 minutes. */
		{
			return TRUE;
		}
	}
	else if(pComp->DFIntervalCnt >= (3600 * INCTRL_TIMER_HZ))  /* 1 hour <= Td < 2 hours. */
	{
		if(pComp->DFMTDuration >= (180 * INCTRL_TIMER_HZ)) /* -4C for 3 minutes. */
		{
			return TRUE;
		}
	}
	else if(pComp->DFIntervalCnt >= (1800 * INCTRL_TIMER_HZ))  /* 30 minutes <= Td < 1 hour. */
	{
		if(pComp->DFLTDuration >= (180 * INCTRL_TIMER_HZ)) /* -8C for 3 minutes. */
		{
			return TRUE;
		}
	}

	/* out coil temp <= -8C. defrost interval >= 20 minutes. Low Pression occured. */
	if( (pComp->OutCoilTemp <= -80) && 
		(pComp->DFIntervalCnt >= (1200 * INCTRL_TIMER_HZ)) && 
		(pComp->CompError & COMP_ERR_LP) )
	{
		return TRUE;
	}

	return FALSE;

}

/* Exit deforst status */
static bool IsCompDefrostCompleted(CompStatusT * pComp, u16 DFRunTime)
{
    /* will add the two comp exit defrost status at the same time ??? */
	if( !pComp->bCompExist )
	{
		return TRUE;
	}

	if(pComp->EvaTemp >= SysDefrostParam.ExitDefrostEvaTemp)  
	{
		return TRUE;
	}

	if(DFRunTime > (SysDefrostParam.DefrostTime * INCTRL_TIMER_HZ))
	{
		return TRUE;
	}

	if(pComp->CompError & COMP_ERR_HP)  /* High Pression occured. */
	{
		return TRUE;
	}

    if(pComp->UnitOutTemp <= SysDefrostParam.ExitDefrostExitWaterTemp)
    {
        if(pComp->UnitOut10sCnt > 10 * INCTRL_TIMER_HZ)
        {
            return TRUE;
        }
        
        pComp->UnitOut10sCnt++;
        
    }
    else
    {     
        pComp->UnitOut10sCnt = 0;
        return FALSE;
    }
    
	return FALSE;
}
#endif
/*   ==========================   INDOOR CONTROLLER functions.   =========================      */

static void CheckIndoorInput(void)
{
#if 0
	if(IdcInPinList.PumpFault.OffTime >= INCTRL_TIMER_HZ)
	{
		InCtrl.CtrlError |= INCTRL_ERR_PUMP;
	}

	if(IdcInPinList.PumpFault.OnTime >= INCTRL_TIMER_HZ)
	{
		InCtrl.CtrlError &= ~(INCTRL_ERR_PUMP);
	}

	if(IdcInPinList.BackWaterFault.OffTime >= INCTRL_TIMER_HZ)
	{
		InCtrl.CtrlError |= INCTRL_ERR_SYS_BACK_SENSOR;
	}

	if(IdcInPinList.BackWaterFault.OnTime >= INCTRL_TIMER_HZ)
	{
		InCtrl.CtrlError &= ~(INCTRL_ERR_SYS_BACK_SENSOR);
	}
    
    if(IdcInPinList.OutFault.OffTime >= INCTRL_TIMER_HZ)
	{
		InCtrl.CtrlError |= INCTRL_ERR_SYS_OUT_SENSOR;
	}

	if(IdcInPinList.OutFault.OnTime >= INCTRL_TIMER_HZ)
	{
		InCtrl.CtrlError &= ~(INCTRL_ERR_SYS_OUT_SENSOR);
	}
    
	/* indoor temp sensor detection. */
	if(InCtrl.OutdoorTemp == HAL_TEMP_ERR)
	{
		InCtrl.ODTSensorFaultOffTime = 0;

		if(InCtrl.ODTSensorFaultOnTime < 0xFF00)
			InCtrl.ODTSensorFaultOnTime++;

		if(InCtrl.ODTSensorFaultOnTime >= INCTRL_TIMER_HZ) /* 1s of fault time. */
			InCtrl.CtrlError |= CTRL_ERR_OUT_SENSOR;
	}
	else
	{
		InCtrl.ODTSensorFaultOnTime = 0;

		if(InCtrl.ODTSensorFaultOffTime < 0xFF00)
			InCtrl.ODTSensorFaultOffTime++;

		if(InCtrl.ODTSensorFaultOffTime >= INCTRL_TIMER_HZ) /* 1s of fault time. */
			InCtrl.CtrlError &= ~(CTRL_ERR_OUT_SENSOR);
	}

	if(InCtrl.OutdoorTemp <= 180)  /* outdoor temp <= 18C. */
	{
		/*	if(low temp forbid cool mode is enabled) */
		if((InCtrl.ODTLow18Duration < 0xFF00) && (SysCfg & SYS_CFG_COOL_FORBID))
			InCtrl.ODTLow18Duration++;
	}
	else if(InCtrl.OutdoorTemp >= 185) /* outdoor temp >= 18.5C, cool mode can be used. */
	{
		InCtrl.ODTLow18Duration = 0;
	}
#endif

}

/*   ==========================   common functions.   =========================      */
#if 0
static bool IsHeaterOnRequired(void)
{
	u8		i;

	if(InCtrl.IDFanOnTime == 0)    /* Indoor fan closed. */
		return FALSE;

	if(InCtrl.RoomTemp > 225)      /*room temp > 22.5C. */
		return FALSE;

	if((InCtrl.RoomTemp + 35) > UserParam.SettingTemp) /* room temp not less 3.5C than target temp. */
		return FALSE;

	for(i = 0; i < MAX_COMP_CNT; i++)
	{
		if( CompStatus[i].bCompExist && (CompStatus[i].CompError == 0) )
		{
			if(CompStatus[i].CompOnTime < 300)	/* < 5 minutes. */
				return FALSE;
			
			if(CompStatus[i].InnerCoilTemp > 500)  /* inner coil temp > 50C. */
				return FALSE;
		}
	}

	return TRUE;
}

static bool IsHeaterOffRequired(void)
{
	u8		i;

	if(InCtrl.RoomTemp >= (UserParam.SettingTemp - SysParam.TempErrorNum))
		return TRUE;
	
	for(i = 0; i < MAX_COMP_CNT; i++)
	{
		if( CompStatus[i].bCompExist && (CompStatus[i].CompError == 0) )
		{
			if(CompStatus[i].InnerCoilTemp >= 550) /* inner coil temp should be >= 55C. */
				return TRUE;
		}
	}

	return FALSE;
}

static u8 HeaterSelectOneOn(void)
{
	/* if heater fault, do nothing. */
	if(InCtrl.CtrlError & INCTRL_ERR_HEATER)
		return 0xFF;

#if 0

	if(IdcOutPinList.Heater1.OffTime >= (60 * INCTRL_TIMER_HZ))
	{
		InCtrl.Heater1On = DEV_ON;
		return;
	}
	
	if(IdcOutPinList.Heater2.OffTime >= (60 * INCTRL_TIMER_HZ))
	{
		InCtrl.Heater2On = DEV_ON;
		return;
	}
	
	if(IdcOutPinList.Heater3.OffTime >= (60 * INCTRL_TIMER_HZ))
	{
		InCtrl.Heater3On = DEV_ON;
		return;
	}

#else

	u8		HeaterId = 0xFF;
	u16		MaxOffTime = 0;  /* (60 * INCTRL_TIMER_HZ); */

	/* select the heater with more off time. */

	if(IdcOutPinList.Heater1.OffTime > MaxOffTime)
	{
		MaxOffTime = IdcOutPinList.Heater1.OffTime;
		HeaterId = 1;
	}

	if(IdcOutPinList.Heater2.OffTime > MaxOffTime)
	{
		MaxOffTime = IdcOutPinList.Heater2.OffTime;
		HeaterId = 2;
	}

	if(IdcOutPinList.Heater3.OffTime > MaxOffTime)
	{
		MaxOffTime = IdcOutPinList.Heater3.OffTime;
		HeaterId = 3;
	}

	if(MaxOffTime <= (60 * INCTRL_TIMER_HZ))	/* can't open the same heater in 1 minute. */
	{
		return 0xFF;
	}

	if(HeaterId == 1)
	{
		InCtrl.Heater1On = DEV_ON;
	}

	if(HeaterId == 2)
	{
		InCtrl.Heater2On = DEV_ON;
	}

	if(HeaterId == 3)
	{
		InCtrl.Heater3On = DEV_ON;
	}

	return HeaterId;

#endif

}

static u8 HeaterSelectOneOff(void)
{

#if 0

	if(IdcOutPinList.Heater1.OnTime > 0)
	{
		InCtrl.Heater1On = 0;
		return;
	}
	
	if(IdcOutPinList.Heater2.OnTime > 0)
	{
		InCtrl.Heater2On = 0;
		return;
	}
	
	if(IdcOutPinList.Heater3.OnTime > 0)
	{
		InCtrl.Heater3On = 0;
		return;
	}

#endif

	u8		HeaterId = 0xFF;
	u16		MaxOnTime = 0;

	/* select the heater with more on time. */

	if(IdcOutPinList.Heater1.OnTime > MaxOnTime)
	{
		MaxOnTime = IdcOutPinList.Heater1.OnTime;
		HeaterId = 1;
	}

	if(IdcOutPinList.Heater2.OnTime > MaxOnTime)
	{
		MaxOnTime = IdcOutPinList.Heater2.OnTime;
		HeaterId = 2;
	}

	if(IdcOutPinList.Heater3.OnTime > MaxOnTime)
	{
		MaxOnTime = IdcOutPinList.Heater3.OnTime;
		HeaterId = 3;
	}

	if(HeaterId == 1)
	{
		InCtrl.Heater1On = DEV_OFF;
	}

	if(HeaterId == 2)
	{
		InCtrl.Heater2On = DEV_OFF;
	}

	if(HeaterId == 3)
	{
		InCtrl.Heater3On = DEV_OFF;
	}

	return HeaterId;

}

/*
* 5) achieved the temperature control goal, power off one compressor.
*/
static u8 CompSelectOneOff(void)
{
	/* select one compressor to power off. */
	/* wear level control. select the comp with more worktime. */

	u8		i;
	u8		MaxIdx = 0xFF;
	u32		MaxWorkTime = 0x0;

	for(i = 0; i < MAX_COMP_CNT; i++)
	{
		if(CompStatus[i].bCompExist)
		{
			if((CompStatus[i].ReqCompMode != COMP_WM_STOP) && (CompStatus[i].ReqCompMode != COMP_WM_EMERGENCY_STOP))
			{
				/* stop the comp with error firstly. */
				if(CompStatus[i].CompError != 0)
				{
					MaxIdx = i;
					break;
				}

				if(CompStatus[i].TotalWorkTime >= MaxWorkTime)
				{
					MaxWorkTime = CompStatus[i].TotalWorkTime;
					MaxIdx = i;
				}
			}

		}
	}

	if(MaxIdx != 0xFF)
	{
		CompStatus[MaxIdx].ReqCompMode = COMP_WM_STOP;

		CompStatus[MaxIdx].bReqDefrost = FALSE;
	}

	return MaxIdx;
}

/*
* 7) select one comp to power on.
*/
static u8 CompSelectOneOn(CompWorkModeT CompMode)
{
	/* select one compressor to power on. */
	/* wear level control. select the comp with less worktime. */

	u8		i;
	u8		MinIdx = 0xFF;
	u32		MinWorkTime = 0xFFFFFFFF;

	for(i = 0; i < MAX_COMP_CNT; i++)
	{
		if(CompStatus[i].bCompExist && (CompStatus[i].CompError == 0))
		{
			/*if(CompStatus[i].CurCompMode == COMP_WM_STOP) */
			if(CompStatus[i].CompOnTime == 0)
			{
				if(CompStatus[i].TotalWorkTime < MinWorkTime)
				{
					MinWorkTime = CompStatus[i].TotalWorkTime;
					MinIdx = i;
				}
			}
			else if(CompStatus[i].CompOnTime < 30) /* the comp powered on < 30s. */
			{
				MinIdx = 0xFF; /* don't poweron any comp untill 30s later. */
				break;
			}
		}
	}

	if(MinIdx != 0xFF)
	{
		CompStatus[MinIdx].ReqCompMode = CompMode;
	}

	return MinIdx;
}


static bool IsHeaterAllOff(void)
{
	if(IdcOutPinList.Heater1.OnTime > 0)
	{
		return FALSE;
	}
	
	if(IdcOutPinList.Heater2.OnTime > 0)
	{
		return FALSE;
	}
	
	if(IdcOutPinList.Heater3.OnTime > 0)
	{
		return FALSE;
	}

	return TRUE;
}


static bool IsCompAllOff(void)
{
	u8		i;

	for(i = 0; i < MAX_COMP_CNT; i++)
	{
		if(CompStatus[i].bCompExist && (CompStatus[i].CurCompMode != COMP_WM_STOP))
			return FALSE;
	}

	return TRUE;	/* all are in STOP mode. */
}


static bool HeaterAllOff(u16 TimesInSecond)
{
	static u16 	OffInterval = 0;

	OffInterval++;

	/* stop one heater per TimesInSecond. */
	if(OffInterval >= (TimesInSecond * INCTRL_TIMER_HZ))
	{
		if(HeaterSelectOneOff() != 0xFF)
			OffInterval = 0;
	}

	return IsHeaterAllOff();
}


static bool CompAllOff(CompWorkModeT CompMode, u16 TimesInSecond)
{
	u8			i;
	static u16 	OffInterval = 0;

	if(CompMode == COMP_WM_EMERGENCY_STOP)
	{
		for(i = 0; i < MAX_COMP_CNT; i++)
		{
			if(CompStatus[i].bCompExist)
				CompStatus[i].ReqCompMode = COMP_WM_EMERGENCY_STOP;
		}

		return TRUE;
	}
	else
	{
		OffInterval++;

		/* stop one compressor per 3s or 30s. */
		if(OffInterval >= (TimesInSecond * INCTRL_TIMER_HZ))
		{
			if(CompSelectOneOff() != 0xFF)
				OffInterval = 0;    /* power off succeeded, clear the count. */
		}

		return IsCompAllOff();
	}
}

static bool SysPowerOff(void)
{
//	bool	bHeatersOff;
	bool	bCompsOff;

//	bHeatersOff = HeaterAllOff(5);	/* power off one heater every 5s. */

	bCompsOff = CompAllOff(COMP_WM_STOP, 5);	/* power off one comp every 5s. */

//	return (bool)(bHeatersOff && bCompsOff);
    
    return (bool)(bCompsOff);
}

static void CheckAllCompTemp(SysWorkModeT WorkMode)
{
	u8		i;

	if(WorkMode == SYS_WM_DEHUMIDIFY)
	{
		WorkMode = SYS_WM_COOL;
	}

	for(i = 0; i < MAX_COMP_CNT; i++)
	{
		CheckCompTemp(&CompStatus[i], WorkMode);
	}
}

static void CheckAllCompFault(void)
{
	u8		i;

#if 0
	u16		CompErrMask;

	/* Inner coil temperature errors. */
	CompErrMask = COMP_ERR_IC_SENSOR /*| COMP_ERR_LEAK*/ | COMP_ERR_IC_OVERCOLD | COMP_ERR_IC_OVERHEAT;

	CompErrMask |= (COMP_ERR_ODF_OVERCURNT | COMP_ERR_ODF_FAULT);  /* outdoor fans. */
	CompErrMask |= (COMP_ERR_OC_SENSOR);  /* out coil. */
	CompErrMask |= (COMP_ERR_OVERLOAD | COMP_ERR_OVERCURNT | COMP_ERR_PWR_PHASE | COMP_ERR_LP_LOCK);  /* comp errors. */

#endif

	InCtrl.bCompsFault = FALSE;

	for(i = 0; i < MAX_COMP_CNT; i++)
	{
		if(!CompStatus[i].bCompExist)
			continue;

		if(CompStatus[i].CompError != 0 /* & CompErrMask */)
		{
			/* this is a patch for HP fault, 
			   HP fault maybe used for defrosting in Indoor Ctrl,
			   so Outdoor Ctrl doesn't handle this emergency fault itself. */
			if(CompStatus[i].CompError & COMP_ERR_HP)
				CompStatus[i].ReqCompMode = COMP_WM_EMERGENCY_STOP;
			else
				CompStatus[i].ReqCompMode = COMP_WM_STOP;
			
			CompStatus[i].bReqDefrost = FALSE;

			InCtrl.bCompsFault = TRUE;
		}
	}

}
#endif

static bool HandleSysFault(void)
{
#if 0
	CheckAllCompFault();

	if(InCtrl.bRemoteOn == 0)
	{
		InCtrl.bReqPending = TRUE;
		InCtrl.ReqWorkMode = SYS_WM_STOP;
	}

	if( (InCtrl.CtrlError == 0) && (!InCtrl.bCompsFault) )
	{
		InCtrl.FaultOn = 0;

		return FALSE;
	}

	if(InCtrl.bCompsFault)
	{
		InCtrl.FaultOn = DEV_ON;
	}

	if(InCtrl.CtrlError & CTRL_ERR_OUT_SENSOR)
	{
		InCtrl.FaultOn = DEV_ON;
	}

	if(InCtrl.CtrlError & CTRL_ERR_OUT_POWER)
	{
		/*SysPowerOff();*/  /* just powerr off the commps with this error! */

		InCtrl.FaultOn = DEV_ON;

		/*return TRUE;*/ /* can't return here! */
	}

	/* begin to handle CTRL (system) fault ---  FATAL errors. */
	if(InCtrl.CtrlError & INCTRL_ERR_FIRE_ALARM)
	{
		CompAllOff(COMP_WM_EMERGENCY_STOP, 5);

		InCtrl.IDFanSpeed = FAN_SPEED_STOP;

		return TRUE;
	}

	if(InCtrl.CtrlError & (INCTRL_ERR_INDOOR_FAN | INCTRL_ERR_FAN_PRESS_DIF | INCTRL_ERR_FILTER))
	{
		/*CompAllOff(COMP_WM_STOP, 3);*/
		/*CompAllOff(COMP_WM_EMERGENCY_STOP, 3);*/

		if( SysPowerOff() )
		{
			InCtrl.IDFanSpeed = FAN_SPEED_STOP;

			if(InCtrl.IDFanOffTime >= (15 * INCTRL_TIMER_HZ))	/* Indoor fan should be closed for 15s. */
			{

			}
		}

		InCtrl.FaultOn = DEV_ON;

		return TRUE;
	}
#endif
	return FALSE;

}

#if 0
static u8 GetInitCompNum(SysWorkModeT WorkMode)
{
    if(WorkMode == SYS_WM_COOL)
    {
        return((InCtrl.BackWaterTemp - UserParam.CoolSettingTemp)*InCtrl.CompSum/SysCompParam.FirstPowerOnTempDif);
    }  
    else if(WorkMode == SYS_WM_HEAT)
    {
        return((UserParam.CoolSettingTemp - InCtrl.BackWaterTemp)*InCtrl.CompSum/SysCompParam.FirstPowerOnTempDif);
    }
    
    return 0;
}

static void CheckAllCompDefrost(void)
{
	u8		i;

	for(i = 0; i < MAX_COMP_CNT; i++)
	{
		if(!CompStatus[i].bCompExist)
			continue;

		if(CompStatus[i].CurCompMode == COMP_WM_DEFROST)
		{
			if(CompStatus[i].DFOnTime < 0xFF00)
				CompStatus[i].DFOnTime++;
		
			CompStatus[i].DFWaitTime = 0;
			CompStatus[i].DFOffTime = 0;
		}
		else
		{
			CompStatus[i].DFOnTime	 = 0;
			
			if(CompStatus[i].DFOffTime < 0xFF00)
				CompStatus[i].DFOffTime++;
			
			if(CompStatus[i].bReqDefrost)
			{
				if(CompStatus[i].DFWaitTime < 0xFF00) /* Defrost waiting is used in Auto mode */
					CompStatus[i].DFWaitTime++;
			}
			else
			{
				CompStatus[i].DFWaitTime = 0;

				if( IsCompDefrostRequired(&CompStatus[i]) &&             /* Defrost required. */
					(CompStatus[i].DFOffTime > (SysParam.DefrostInterval * INCTRL_TIMER_HZ)) )   /* Defrost interval > 30 minutes. */
				{
					CompStatus[i].bReqDefrost = TRUE;
				}
			}
		}
	}

	return;
}

static u8	ManualDefrostCompIdx = 0xFF;

static bool SelectCompDefrostManual(u8 CompIdx)
{
	u8		i;
	u8		DFCnt = 0;
//	u8		CompCnt = 0;

	if( CompIdx >= MAX_COMP_CNT )	/* 0 ~ (MAX_COMP_CNT - 1): total MAX_COMP_CNT compressors. */
		return FALSE;

    
	if( !CompStatus[CompIdx].bCompExist )
	{
		return FALSE;
	}

	if( (ManualDefrostCompIdx < MAX_COMP_CNT) &&
		(CompStatus[ManualDefrostCompIdx].ReqCompMode == COMP_WM_DEFROST) )
	{
		return FALSE;  /* has one manually defrosting request. */
	}
    
	if(CompStatus[CompIdx].DFOffTime <= ( InCtrl.DefrostInterval* INCTRL_TIMER_HZ))
	{
		return FALSE;    
	}
    
    if(CompStatus[CompIdx].UnitOutTemp <= SysDefrostParam.ExitDefrostExitWaterTemp)
    {
        return FALSE;
    }

	for(i = 0; i < MAX_COMP_CNT; i++)
	{
		if(CompStatus[i].bCompExist)
		{
//			CompCnt++;

//			if(CompStatus[i].ReqCompMode == COMP_WM_DEFROST)
            if(CompStatus[i].CurCompMode == COMP_WM_DEFROST)
            DFCnt++;
            
		}
	}

#if 0
	if((DFCnt >= 2) || ((DFCnt > 0) && (CompCnt <= 3)))
	{
		return FALSE;
	}
#endif
    
    if(DFCnt >= SysDefrostParam.MaxDefrostCompNum)
	{
		return FALSE;
	}
    
	if( CompStatus[CompIdx].CurCompMode != COMP_WM_HEAT ) /* MUST be in HEAT mode. */
	{
		return FALSE;
	}

	/* LP is acceptable for defrost, other faults are not allowed. */
	if(CompStatus[CompIdx].CompError & (~(COMP_ERR_LP)))
	{
		return FALSE;
	}


    if(CompStatus[CompIdx].EvaTemp >= SysDefrostParam.ExitDefrostEvaTemp)
    {
        return FALSE;
    }
    
//	if(CompStatus[CompIdx].OutCoilTemp <= 0)
    if(CompStatus[CompIdx].EvaTemp < SysDefrostParam.ExitDefrostEvaTemp)
	{
		CompStatus[CompIdx].bReqDefrost = TRUE;
		CompStatus[CompIdx].DFOffTime = 0xFF00;
		CompStatus[CompIdx].DFOnTime = 0;
		CompStatus[CompIdx].DFWaitTime = 0xFF00;

		CompStatus[CompIdx].ReqCompMode = COMP_WM_DEFROST;

		ManualDefrostCompIdx = CompIdx;

		DebugPrintf("\r\nAC: Manually select [Comp-%d] to defrost. \r\n", CompIdx);

		return TRUE;
	}

	return FALSE;

}

static u8 SelectCompDefrostAuto(void)
{
	u8		i;
	u8		DFCnt = 0;

    /* will add the two comp into defrost status at the same time ??? */
  
  
  
  
    
	/* let comp (with LP fault or Eva tempreture to defrost) run DEFROST. */
	for(i = 0; i < MAX_COMP_CNT; i++)
	{
		if(!CompStatus[i].bCompExist)
			continue;

		if(CompStatus[i].bReqDefrost && (CompStatus[i].CurCompMode != COMP_WM_DEFROST))
		{
			if((CompStatus[i].CompError & COMP_ERR_LP)&&(SysDefrostParam.LowPressDefrostUsedSet == 1))
			{
                if((CompStatus[i].DFOffTime >= SysDefrostParam.LowPressDefrostIntervalTime) 
                        &&(CompStatus[i].CompOnTime > SysDefrostParam.LowPressDefrostDetectDelayTime) 
                          &&(CompStatus[i].EvaTemp < SysDefrostParam.LowPressDefrostEvaTemp) 
                            &&((InCtrl.OutdoorTemp - CompStatus[i].OutCoilTemp) >= SysDefrostParam.LowPressDefrostEvaTempDif) 
                              &&(CompStatus[i].UnitOutTemp > SysDefrostParam.ExitDefrostExitWaterTemp))
                {
                    CompStatus[i].CompError &= ~(COMP_ERR_LP);
                    CompStatus[i].ReqCompMode = COMP_WM_DEFROST;
                    DFCnt++;
                }
			}
            
            if((CompStatus[i].DFOffTime >= InCtrl.DefrostInterval)
               &&(CompStatus[i].EvaTemp <= SysDefrostParam.DefrostPermissionEvaTemp)
                 &&(InCtrl.OutdoorTemp <= SysDefrostParam.DefrostPermissionTemp)
                   &&((InCtrl.OutdoorTemp - CompStatus[i].EvaTemp) >= InCtrl.DefrostEvaDif)
                     &&(CompStatus[i].UnitOutTemp > SysDefrostParam.ExitDefrostExitWaterTemp)
                       &&(CompStatus[i].CompOnTime > SysDefrostParam.DefrostCompOnTime))
            {
                    CompStatus[i].ReqCompMode = COMP_WM_DEFROST;
                    DFCnt++;
            
            }
#if 0
			if(CompStatus[i].DFWaitTime >= (600 * INCTRL_TIMER_HZ)) /* wait for >=10 minutes.*/
			{
				CompStatus[i].ReqCompMode = COMP_WM_DEFROST;
				DFCnt++;
			}
#endif
            
		}
	}

	return DFCnt;
}

static u8 SelectOneCompDefrost(void)
{
	u8		i;
	u8		CompIdx = 0xFF;
	u16 	MaxTime = 0;

	/* select the comp with longer time of running. */
	for(i = 0; i < MAX_COMP_CNT; i++)
	{
		if(!CompStatus[i].bCompExist)
			continue;

		if(CompStatus[i].bReqDefrost && (CompStatus[i].CurCompMode != COMP_WM_DEFROST))
		{
			if(CompStatus[i].CompOnTime > MaxTime)
			{
				MaxTime = CompStatus[i].CompOnTime;
				CompIdx = i;
			}
		}
	}

	if(CompIdx != 0xFF)
	{
		CompStatus[CompIdx].ReqCompMode = COMP_WM_DEFROST;

		DebugPrintf("\r\nAC: Select [Comp-%d] to defrost. \r\n", CompIdx);

		return 1;
	}

	return 0;
}

static u8 RunCompDefrost(void)
{
	u8		i;
	u8		DFCnt = 0;
	u8		CompCnt = 0;

	CheckAllCompDefrost(); /* check the comp deforst require according the defrost internal */

	/* check defrost complete. */
	for(i = 0; i < MAX_COMP_CNT; i++)
	{
		if(!CompStatus[i].bCompExist)
			continue;

		if(CompStatus[i].bReqDefrost && (CompStatus[i].DFOnTime > 0))
		{
			if(IsCompDefrostCompleted(&CompStatus[i], CompStatus[i].DFOnTime))
			{
				CompStatus[i].ReqCompMode = COMP_WM_HEAT; /* quit the DEFROST mode. */

				CompStatus[i].bReqDefrost = FALSE;
				/* deleted the following codes, because it will be set in CheckAllCompDefrost. */
				/*
				CompStatus[i].DFOffTime = 0;
				CompStatus[i].DFOnTime  = 0;
				CompStatus[i].DFWaitTime = 0;
				*/

				CompStatus[i].CompError &= ~(COMP_ERR_HP);

				CompStatus[i].DFIntervalCnt = 0;         /* DFIntervalCnt re-Count. */ /* move this to CheckAllCompDefrost()? */
			}
		}
	}

	SelectCompDefrostAuto(); /* select the Comp into defrost status */

	for(i = 0; i < MAX_COMP_CNT; i++)
	{
		if(CompStatus[i].bCompExist)
		{
			CompCnt++;

			if(CompStatus[i].ReqCompMode == COMP_WM_DEFROST)
				DFCnt++;
		}
	}

	if((DFCnt == 0) || ((DFCnt < 2) && (CompCnt > 3)))
	{
		DFCnt += SelectOneCompDefrost();
	}

	if((DFCnt > 0) && ((InCtrl.CtrlError & INCTRL_ERR_HEATER) == 0))
	{
		/*
		InCtrl.Heater1On = DEV_ON;
		InCtrl.Heater2On = DEV_ON;
		InCtrl.Heater3On = DEV_ON; 
		*/
//		HeaterSelectOneOn();
	}

	for(i = 0; i < MAX_COMP_CNT; i++)
	{
		if(!CompStatus[i].bCompExist)
			continue;

		/* mask LP flag for 6 minutes after defrost stopped. */
		/* or maks the LP flag when running defrost. */
		if(CompStatus[i].DFOffTime <= (360 * INCTRL_TIMER_HZ))
		{
			CompStatus[i].CompError &= ~(COMP_ERR_LP);
		}

		if(CompStatus[i].DFOffTime <= (3 * INCTRL_TIMER_HZ)) /* mask HP flag for 3s after defrost stopped. */
		{
			CompStatus[i].CompError &= ~(COMP_ERR_HP);
		}
	}

	DebugPrintf("\r\nAC:  Total %d comps in defrost mode. \r\n", DFCnt);

	return DFCnt;
}

static void ExitPreFrz(void)
{
    if((InCtrl.PumpOn == DEV_ON)&&(!InCtrl.EnterCompPreFrz)&&(!InCtrl.EnterHeaterPreFrz))
    {
        if(InCtrl.PreFrzTemp > SysPreFrzParam.PumpPreFrzTemp)
        {
            InCtrl.PumpOn = 0;
        }
    }
    
    if(InCtrl.PreFrzTemp > SysPreFrzParam.ExitCompPreFrzTemp)
    {
        if(HeaterAllOff(5))
        {
            InCtrl.EnterHeaterPreFrz = FALSE;
        }   
    }
    
    if(InCtrl.PreFrzTemp > SysPreFrzParam.ExitCompPreFrzTemp)
    {        
        if(HeaterAllOff(5) && CompAllOff(COMP_WM_STOP,5))
        {
            InCtrl.PumpOn = 0;
            InCtrl.EnterCompPreFrz = FALSE;
        }   
    }
}

static void PreFrzPump(void)
{
    static u16 CompUpLoad8min = 0;
    static u8 HeaterUpLoad60s = 0;
    static s16 LastPreFrzTemp = 0;
    
    if(InCtrl.CurWorkMode == SYS_WM_STOP)
    {
        CompUpLoad8min = 0;
        HeaterUpLoad60s = 0;
        LastPreFrzTemp = InCtrl.PreFrzTemp;
    }
    
         /* out prevent frozen */
    if(InCtrl.OutdoorTemp > (SysPreFrzParam.EnterPreFrzTemp + SysPreFrzParam.ExitPreFrzTempDif))
    {
        if(InCtrl.OutError[1] & CTRL_ERR_OUT_SENSOR)
        {
            ExitPreFrz();
        }
        
        CompAllOff(COMP_WM_STOP,5);
        HeaterAllOff(5);
        
        return;
    }

    CheckPreFrzInterval();
    
    /* exit prevent frozen */
    ExitPreFrz();
       
    /* in prevent frozen */
    if(InCtrl.OutdoorTemp <= SysPreFrzParam.EnterPreFrzTemp)
    {
        if(IdcOutPinList.PumpOn.OffTime >= InCtrl.PreFrzInterval)
        {
            InCtrl.PumpOn = DEV_ON;
        }     
    }
    
    if(IdcOutPinList.PumpOn.OnTime >= 60*INCTRL_TIMER_HZ)
    {
        CheckPreFrzTemp();
        
        if(InCtrl.PreFrzTemp < SysPreFrzParam.PumpPreFrzTemp)
        {
            InCtrl.PumpOn = 0;
        }
        else if(InCtrl.PreFrzTemp < SysPreFrzParam.HeatPreFrzTemp)
        {
            InCtrl.PumpOn = DEV_ON;
        }
        else if(InCtrl.PreFrzTemp < SysPreFrzParam.CompPreFrzTemp)
        {
            if(HeaterUpLoad60s == 0)
            {
                HeaterSelectOneOn();
                HeaterUpLoad60s = 60;
                InCtrl.EnterHeaterPreFrz = TRUE;
            }
            
            HeaterUpLoad60s--;
        }
        else
        {           
            if(CompUpLoad8min == 0)
            {
                if(InCtrl.PreFrzTemp < (LastPreFrzTemp + 10))
                {
                    LastPreFrzTemp = InCtrl.PreFrzTemp;
                    CompSelectOneOn(COMP_WM_HEAT);
                    CompUpLoad8min = 8*60;
                    InCtrl.EnterCompPreFrz = TRUE;
                }
            }
            
            CompUpLoad8min--;                                  
        }       
    }   
}

static void PreFrzHeater(void)
{
    u8  i;
    
    for(i=0;i< MAX_COMP_CNT;i++)
    {
        if(CompStatus[i * 2 + 1].bCompExist == FALSE)
          return;
        
        if(CompStatus[i].UnitOutTemp < 50)
        {
            CompStatus[i].PreFrzHeaterOn = DEV_ON;
        }
        
        if(CompStatus[i].UnitOutTemp >80)
        {
            CompStatus[i].PreFrzHeaterOn = 0;
        }
    }
}

#if 0
static u8 HeatModeFanSpeed(void)
{
	u8		i;

	if((SysCfg & SYS_CFG_AVOID_COLD_WIND) == 0)
		return 2;  /* use the user setting fan speed. */

	for(i = 0; i < MAX_COMP_CNT; i++)
	{
		if( CompStatus[i].bCompExist && (CompStatus[i].CompError == 0) )
		{
			if(CompStatus[i].NormalWindOnTime >= (5 * INCTRL_TIMER_HZ))	/* >= 5s. */
				return 2;  /* use the user setting fan speed. */
		}
	}

	for(i = 0; i < MAX_COMP_CNT; i++)
	{
		if( CompStatus[i].bCompExist && (CompStatus[i].CompError == 0) )
		{
			if(CompStatus[i].AvoidColdWindOffTime >= (5 * INCTRL_TIMER_HZ)) /* >= 5s. */
				return 1; /* FAN_SPEED_LOW. */
		}
	}

	for(i = 0; i < MAX_COMP_CNT; i++)
	{
		if( CompStatus[i].bCompExist && (CompStatus[i].CompError == 0) )
		{
			if(CompStatus[i].AvoidColdWindOnTime < (5 * INCTRL_TIMER_HZ)) /* < 5s. */
				return 3; /* FAN_SPEED unchanged. */
		}
	}

	return 0;   /* FAN_SPEED_STOP. */
}
#endif

static FanSpeedT GetFanSpeed(SysWorkModeT WorkMode)
{
	FanSpeedT		fanSpeed = FAN_SPEED_STOP;
	
	switch(WorkMode)
	{
		case SYS_WM_STOP:
			fanSpeed = FAN_SPEED_STOP;
			break;
			
		case SYS_WM_DEHUMIDIFY:
			fanSpeed = FAN_SPEED_LOW;
			break;
			
		case SYS_WM_FAN:
			if( (UserParam.FanSpeed == FAN_SPEED_STOP) ||	(UserParam.FanSpeed == FAN_SPEED_AUTO) )
			{
				fanSpeed = FAN_SPEED_MID;  /* can't return FAN_STOP in fan mode. */
			}
			else
			{
				fanSpeed = UserParam.FanSpeed;
			}
			break;
			
		case SYS_WM_COOL:
			if(UserParam.FanSpeed == FAN_SPEED_STOP)
			{
				fanSpeed = FAN_SPEED_LOW;
			}
			else if(UserParam.FanSpeed == FAN_SPEED_AUTO)
			{
				if(InCtrl.RoomTemp >= (InCtrl.TargetTemp + 15))
				{
					fanSpeed = FAN_SPEED_HIGH;
				}
				else if(InCtrl.RoomTemp >= (InCtrl.TargetTemp + 5))
				{
					if(InCtrl.IDFanSpeed == FAN_SPEED_HIGH)
						fanSpeed = FAN_SPEED_HIGH;
					else
						fanSpeed = FAN_SPEED_MID;
				}
				else if(InCtrl.RoomTemp >= (InCtrl.TargetTemp - 5))
				{
					if(InCtrl.IDFanSpeed == FAN_SPEED_HIGH)
						fanSpeed = FAN_SPEED_MID;
					else
						fanSpeed = InCtrl.IDFanSpeed;
				}
				else
				{
					fanSpeed = FAN_SPEED_LOW;
				}
				
			}
			else
			{
				fanSpeed = UserParam.FanSpeed;
			}

			if((fanSpeed == FAN_SPEED_STOP) || (fanSpeed >= FAN_SPEED_MAX))
				fanSpeed = FAN_SPEED_LOW;

			break;
			
		case SYS_WM_HEAT:
			/* set indoor fan speed. */
			if(UserParam.FanSpeed == FAN_SPEED_STOP)
			{
				fanSpeed = FAN_SPEED_LOW;
			}
			else if(UserParam.FanSpeed == FAN_SPEED_AUTO)
			{
				if(InCtrl.RoomTemp >= (InCtrl.TargetTemp + 10))
				{
					fanSpeed = FAN_SPEED_LOW;
				}
				else if(InCtrl.RoomTemp >= InCtrl.TargetTemp)
				{
					if(InCtrl.IDFanSpeed == FAN_SPEED_LOW)
						fanSpeed = FAN_SPEED_LOW;
					else
						fanSpeed = FAN_SPEED_MID;
				}
				else if(InCtrl.RoomTemp >= (InCtrl.TargetTemp - 10))
				{
					fanSpeed = FAN_SPEED_MID;
				}
				else if(InCtrl.RoomTemp >= (InCtrl.TargetTemp - 20))
				{
					if( (InCtrl.IDFanSpeed == FAN_SPEED_MID) || 
						(InCtrl.IDFanSpeed == FAN_SPEED_HIGH) )
					{
						fanSpeed = InCtrl.IDFanSpeed;
					}
					else
						fanSpeed = FAN_SPEED_MID;
				}
				else
				{
					fanSpeed = FAN_SPEED_HIGH;
				}
				
			}
			else
			{
				fanSpeed = UserParam.FanSpeed;
			}

			if((fanSpeed == FAN_SPEED_STOP) || (fanSpeed >= FAN_SPEED_MAX))
				fanSpeed = FAN_SPEED_LOW;

			break;
	}

	return fanSpeed;

}
#endif

/*   ==========================   AC-Work-Mode functions.   =========================      */
#if 0
static u16    HeaterLoadDelay   = 0xFF00;
static u16    HeaterUnloadDelay = 0xFF00;
#endif

static u16    CompLoadDelay = 0xFF00;
static u16    CompUnloadDelay = 0xFF00;

static void InCtrlManMode(void)
{
	static SysWorkModeT  CurMode = SYS_WM_STOP;
	static u16           CompsOffTime = 0xFF00;

		/* check mode change request. */
	if( (InCtrl.bReqPending) && (InCtrl.ReqWorkMode != InCtrl.CurWorkMode) )
	{
		/* all comps off and quit the defrost mode. */
		/* set to new mode. */
		if( SysPowerOff() )
		{
			InCtrl.bReqPending = FALSE;
			InCtrl.CurWorkMode = InCtrl.ReqWorkMode;

//			CompLoadDelay = 0xFF00;		/* re-init the vars. */
//			CompUnloadDelay = 0xFF00;

//			HeaterLoadDelay   = 0xFF00;
//			HeaterUnloadDelay = 0xFF00;

			InCtrl.ModeTime = 0;
		}

		return;      /* Work Mode will be changed. */
	}

	/* Logic */

}

static void InCtrlAutoMode(void)
{
	u8		i;

	static SysWorkModeT  CurMode = SYS_WM_STOP;
	static u16           CompsOffTime = 0xFF00;

//	s16		HeatTempMin = UserParam.SettingTemp - 35;
//	s16		CoolTempMax = UserParam.SettingTemp + 30;

	/* in AUTO mode, can't call CheckCompTemp(), because of the time calculation in the function. */
	/* call it in the corresponding MODE handler. */
	/* CheckAllCompTemp(InCtrl.CurWorkMode); */

	/* check mode change request. */
	if( (InCtrl.bReqPending) && (InCtrl.ReqWorkMode != InCtrl.CurWorkMode) )
	{
		if(InCtrl.ReqWorkMode != CurMode)
		{
			/* set to new mode if all component are powered off. */
			if( SysPowerOff() )
			{
				InCtrl.bReqPending = FALSE;
				InCtrl.CurWorkMode = InCtrl.ReqWorkMode;

				CurMode = SYS_WM_STOP;	/* reset to the initial mode. */

				InCtrl.ModeTime = 0;
			}
		}

		return;      /* Work Mode may be changed. */

	}

	/* Logic */

}

static void InCtrlStopMode(void)
{

	u8		i;
	u16		clearFlags = 0;

	/* check mode change request. */
	if( (InCtrl.bReqPending) && (InCtrl.ReqWorkMode != InCtrl.CurWorkMode) )
	{
		/* set to new mode. */
		InCtrl.bReqPending = FALSE;
		InCtrl.CurWorkMode = InCtrl.ReqWorkMode;

	//	IDFanDelayTime = 0;		/* clear counter. */

		InCtrl.ModeTime = 0;

		return;
	}

	if(InCtrl.ModeTime < 0xFF00)
		InCtrl.ModeTime++;

#if 0
	if(SysPowerOff())	/* power off all comps & heaters ok? */
	{
		IDFanDelayTime++;
	}
	else
	{
		IDFanDelayTime = 0;

		return;
	}

	if(IDFanDelayTime >= (SysParam.IdfDelayTime * INCTRL_TIMER_HZ))
	{
		InCtrl.IDFanSpeed = FAN_SPEED_STOP;
	}

	if(InCtrl.IDFanOffTime > (15 * INCTRL_TIMER_HZ))	/* Indoor fan should be closed for 15s. */
	{

	}
#endif

	clearFlags = (COMP_ERR_HP | COMP_ERR_ODF_OVERCURNT | COMP_ERR_OVERCURNT | COMP_ERR_PWR_PHASE | COMP_ERR_LP_LOCK /*| COMP_ERR_LEAK*/ );

	/* system power off, clear the fatal error flags. */
	for(i = 0; i < MAX_COMP_CNT; i++)
	{
		CompStatus[i].CompError &= ~(clearFlags);  /* recover from HP... faults by power off. */
	}

	clearFlags = 0;

#if 0
	if(IdcInPinList.HeaterFault.OnTime >= INCTRL_TIMER_HZ)
	{
		clearFlags |= ( INCTRL_ERR_HEATER );
	}

	if(IdcInPinList.FireAlarm.OnTime >= INCTRL_TIMER_HZ)
	{
		clearFlags |= ( INCTRL_ERR_FIRE_ALARM );
	}

	if(IdcInPinList.IDFanFault.OnTime >= INCTRL_TIMER_HZ)
	{
		clearFlags |= ( INCTRL_ERR_INDOOR_FAN );
	}

	if(IdcInPinList.PressDifFault.OnTime >= (20 * INCTRL_TIMER_HZ))
	{
		clearFlags |= ( INCTRL_ERR_FAN_PRESS_DIF );
	}

	if(IdcInPinList.FilterFault.OnTime >= INCTRL_TIMER_HZ)
	{
		clearFlags |= ( INCTRL_ERR_FILTER );
	}

	InCtrl.CtrlError &= ~( clearFlags );

    InCtrl.bSysInit = TRUE;     /* Initial System Power on Process */
#endif
    
	return;

}

/*   ==========================   TASK functions.   =========================      */
static bool ReadNvSysData(EepromSysDataT * pParam);
static bool WriteNvSysData(EepromSysDataT * pParam);
static bool SaveNvSysData(void);

static void InitCompData(void)
{
	u8	i;

	for(i = 0; i < MAX_COMP_CNT; i++)
	{
		memset(&CompStatus[i], 0, sizeof(CompStatusT));

		CompStatus[i].CurCompMode = COMP_WM_STOP;
		CompStatus[i].ReqCompMode = COMP_WM_STOP;
	}
}

static void InitOutCtrl(void)
{
#if 0
	u8	i;

	for(i = 0; i < MAX_OUT_CTRL_CNT; i++)
	{
		memset(&OutCtrlRsp[i], 0, sizeof(OutCtrlRspT));
		OutCtrlCommCnt[i] = 0;
	}
#endif
}

static void InitParam(void)
{
#if 0
	memset(&IdcInPinList, 0, sizeof(IdcInPinList));
	memset(&IdcInDataList, 0, sizeof(IdcInDataList));

	memset(&IdcOutPinList, 0, sizeof(IdcOutPinList));
#endif
    
	InitCompData();

	InitOutCtrl();

	/* set to default setting. */
	memset(&UserParam, 0, sizeof(HmiUserParamT));
    
#if 0
	UserParam.bPowerOn = FALSE;
	UserParam.WorkMode = SYS_WM_COOL;     /* NvParam.WorkMode; */
	/*UserParam.bIDFanOn = DEV_ON;     */ /* NvParam.FanSpeed; */
	UserParam.FanSpeed = FAN_SPEED_HIGH;  /* NvParam.FanSpeed; */
	UserParam.SettingTemp= 240;           /* NvParam.SettingTemp; */ /* 24C, in 0.1C unit. */
	UserParam.bManualDefrost = FALSE;

	memset(&SysParam, 0, sizeof(HmiSysParamT));
	SysParam.TempErrorNum = 10;			/* 1.0C */
	SysParam.TempDiffForLoad = 15;		/* 1.5C */
	SysParam.TempDiffForUnload = 15;	/* 1.5C */
	SysParam.CompStartInterval = 300;	/* 300s = 5 minutes */
	SysParam.CompMinStopTime = 180;		/* 180s = 3 minutes */
	SysParam.LoadDelay = 300;			/* 300s = 5 minutes */
	SysParam.UnloadDelay = 300;			/* 300s = 5 minutes */
	SysParam.QuitDefrostTemp = 100;		/* 10C */
	SysParam.IdfLeadTime = 15;			/* 15s */
	SysParam.IdfDelayTime = 60;			/* 60s */
	SysParam.DefrostInterval = 1800; 	/* 1800s = 30 minutes */
	SysParam.MaxCompCurrent  = 640;		/* 64A */
	SysParam.MaxFanCurrent   = 50;		/* 5A  */
	SysParam.MaxDehumidComps = 2;		/* 2 comps for dehumidify. */
#endif
    
	memset(&InCtrl, 0, sizeof(InCtrl));
    
#if 0
	InCtrl.CurWorkMode = SYS_WM_STOP;
	InCtrl.TargetTemp = UserParam.SettingTemp;
    InCtrl.bSysInit = TRUE;
#endif
    
	/* get hardware state and init the hardware state. */
	ReadAcInput();
	WriteAcOutput();
	
}

static void InCtrlTimerCallback( TimerHandle_t xTimer )
{
    xEventGroupSetBits( InCtrlEventHandle, INCTRL_EVENT_CTRL_LOOP );
}

#if 0
static void SetCompStatusData(u8 idx)
	u8		compIdx = idx * 2;
	u16		CompErrMask;

	CompErrMask = (COMP_ERR_HP | COMP_ERR_ODF_OVERCURNT | COMP_ERR_OVERCURNT | COMP_ERR_PWR_PHASE | COMP_ERR_LP_LOCK /*| COMP_ERR_LEAK*/ );

	static u8    loopCnt = 0;
	static bool  bOdSensorErr = TRUE;
	static s16   OutMinTemp = HAL_TEMP_ERR;

	CompStatus[compIdx].TotalWorkTime = OutCtrlRsp[idx].Comp1.TotalWorkTime;
	CompStatus[compIdx].bCompExist    = OutCtrlRsp[idx].Comp1.bCompExist;
	CompStatus[compIdx].CurCompMode   = OutCtrlRsp[idx].Comp1.CompMode;
	CompStatus[compIdx].CompError    &= CompErrMask;
	CompStatus[compIdx].CompError    |= OutCtrlRsp[idx].Comp1.CompError;
	CompStatus[compIdx].CompOnTime    = OutCtrlRsp[idx].Comp1.CompOnTime;
	CompStatus[compIdx].OutCoilTemp   = OutCtrlRsp[idx].Comp1.OutCoilTemp;
	CompStatus[compIdx].CompCurrent   = OutCtrlRsp[idx].Comp1.CompCurrent;
	CompStatus[compIdx].ODFanCurrent  = OutCtrlRsp[idx].Comp1.ODFanCurrent;

	CompStatus[compIdx + 1].TotalWorkTime = OutCtrlRsp[idx].Comp2.TotalWorkTime;
	CompStatus[compIdx + 1].bCompExist    = OutCtrlRsp[idx].Comp2.bCompExist;
	CompStatus[compIdx + 1].CurCompMode   = OutCtrlRsp[idx].Comp2.CompMode;
	CompStatus[compIdx + 1].CompError    &= CompErrMask;
	CompStatus[compIdx + 1].CompError    |= OutCtrlRsp[idx].Comp2.CompError;
	CompStatus[compIdx + 1].CompOnTime    = OutCtrlRsp[idx].Comp2.CompOnTime;
	CompStatus[compIdx + 1].OutCoilTemp   = OutCtrlRsp[idx].Comp2.OutCoilTemp;
	CompStatus[compIdx + 1].CompCurrent   = OutCtrlRsp[idx].Comp2.CompCurrent;
	CompStatus[compIdx + 1].ODFanCurrent  = OutCtrlRsp[idx].Comp2.ODFanCurrent;

	InCtrl.OutError[idx] = (u8)OutCtrlRsp[idx].CtrlError;

	/* OUTCTRL_ERR_OD_SENSOR is set when temp=HAL_TEMP_ERR for 1s. 
	   There is 1 second of timeslot that temp==HAL_TEMP_ERR but OUTCTRL_ERR_OD_SENSOR is not set.
	*/
	if( ((OutCtrlRsp[idx].CtrlError & OUTCTRL_ERR_OD_SENSOR) == 0) && 
		(OutCtrlRsp[idx].OutdoorTemp != HAL_TEMP_ERR) )
	{
		if( OutCtrlRsp[idx].OutdoorTemp < OutMinTemp )
			OutMinTemp = OutCtrlRsp[idx].OutdoorTemp;

		bOdSensorErr = FALSE;
	}

	if(OutCtrlRsp[idx].CtrlError & OUTCTRL_ERR_POWER)
	{
		InCtrl.CtrlError |= CTRL_ERR_OUT_POWER;
	}

	loopCnt++;

	if(loopCnt >= MAX_OUT_CTRL_CNT) /* max number of outdoor controller is MAX_COMP_CNT. */
	{
		if(bOdSensorErr)
		{
			InCtrl.CtrlError |= CTRL_ERR_OUT_SENSOR;
		}
		else
		{
			InCtrl.CtrlError &= ~CTRL_ERR_OUT_SENSOR;
		}

		InCtrl.OutdoorTemp = OutMinTemp;

		OutMinTemp = HAL_TEMP_ERR;
		bOdSensorErr = TRUE;
		loopCnt = 0;
	}

}

static void UpdateOutdoorData(EventBits_t Event)
{
	if ( Event & INCTRL_EVENT_ODC1_DATA )
	{
		SetCompStatusData(0);

		OutCtrlCommCnt[0] = 0;
		InCtrl.OutError[0] &= ~(OUTCTRL_ERR_COMM);
	}

	if ( Event & INCTRL_EVENT_ODC2_DATA )
	{
		SetCompStatusData(1);

		OutCtrlCommCnt[1] = 0;
		InCtrl.OutError[1] &= ~(OUTCTRL_ERR_COMM);
	}

	if ( Event & INCTRL_EVENT_ODC3_DATA )
	{
		SetCompStatusData(2);

		OutCtrlCommCnt[2] = 0;
		InCtrl.OutError[2] &= ~(OUTCTRL_ERR_COMM);
	}

	if ( Event & INCTRL_EVENT_ODC4_DATA )
	{
		SetCompStatusData(3);

		OutCtrlCommCnt[3] = 0;
		InCtrl.OutError[3] &= ~(OUTCTRL_ERR_COMM);
	}
}
#endif

static void InCtrlTaskInit( void )
{
	InitParam();
	
    InCtrlEventHandle = xEventGroupCreateStatic( &InCtrlEventCb );
	
    InCtrlTimerHandle = xTimerCreateStatic( "InCtrlTimer", INCTRL_TIMER_TICK, pdTRUE, NULL,
                                            InCtrlTimerCallback, &InCtrlTimerCb );

	ParamSemHandle = xSemaphoreCreateMutexStatic(&ParamSemCb);

	UserParamDirtyFlag = ~(MOD_ID_HMI);
	SysParamDirtyFlag  = ~(MOD_ID_HMI);

	InCtrlInitOk = TRUE;
}

void InCtrlTask( void *pvParameters )
{
//	u8				i;
    int cnt = 0;
    EventBits_t		Event;

	InCtrlTaskInit();

    xTimerStart( InCtrlTimerHandle, 0 );
	
    xEventGroupSetBits( InCtrlEventHandle, INCTRL_EVENT_USER_DATA );
	
    while(TRUE)
    {
        Event = xEventGroupWaitBits( InCtrlEventHandle, INCTRL_EVENT_ALL, pdTRUE, pdFALSE, portMAX_DELAY );

        if ( Event & INCTRL_EVENT_USER_DATA )
        {
#if 0
        	/* process User request. */
			InCtrl.TargetTemp = UserParam.SettingTemp;

			if( UserParam.bPowerOn && (InCtrl.bRemoteOn != 0) )
			{
				if(InCtrl.CurWorkMode != UserParam.WorkMode)
				{
					InCtrl.bReqPending = TRUE;
					InCtrl.ReqWorkMode = UserParam.WorkMode;
				}
				else
				{
					InCtrl.bReqPending = FALSE;
					InCtrl.ReqWorkMode = UserParam.WorkMode;
				}
			}
			else /* system is powered off. */
			{
				if(InCtrl.CurWorkMode != SYS_WM_STOP)
				{
					InCtrl.bReqPending = TRUE;
					InCtrl.ReqWorkMode = SYS_WM_STOP;
				}
			}

			/* request to defrost the specified comp. */
			if( UserParam.bManualDefrost )
			{
				SelectCompDefrostManual(UserParam.CompIdx);
				UserParam.bManualDefrost = FALSE;
			}

			if( UserParam.bPowerOn && (InCtrl.bRemoteOn == 0) ) /* remote ctrl is OFF. */
			{
				HmiUserParamT		temp;

				/* Let LCD display the POWER off state. */
				memcpy(&temp, &UserParam, sizeof(HmiUserParamT));
				temp.bPowerOn = FALSE;
				InCtrlSetUserParam(&temp, 0);  /* notify all module that the param is changed. */
			}
#endif

        }
		
        if ( Event & INCTRL_EVENT_CTRL_LOOP )
        {        

            if(++cnt>=4)
            {
              LTRACEF("Timer test-------> ");
              cnt = 0;
            }

          
#if 0
            /* read and save Sys preheat time */
            ReadNvSysData(&NvSysData);
            SaveNvSysData(); /* Save the preheat times every 20min*/ 
#endif
            
            if(UserParam.SysSetting.bPowerOn == TRUE)
            {
                InCtrl.AssHeaterOn = DEV_ON;
            }
            else
            {
                InCtrl.AssHeaterOn = 0;
            }
            
            WriteAcOutput();
       
          /* get device status. INPUT samples. */
            ReadAcInput();
        
            CheckIndoorInput();
            HandleSysFault();
            
#if 0
            /* PreHeat process */
            if(!IsPreHeatComp())
            {
                InCtrl.CurWorkMode = SYS_WM_STOP;
            }
            
        	/* get device status. INPUT samples. */
        	ReadAcInput();

			CheckIndoorInput();

			/* call state handler. */
			if(SysCfg & SYS_CFG_COOL_ONLY)
			{
				StateFunc2[InCtrl.CurWorkMode]();
			}
			else
			{
				StateFunc[InCtrl.CurWorkMode]();
			}

			HandleSysFault();

			/* flush control signal to device. OUTPUT FLUSH. */
			WriteAcOutput();

			/* detect communication. */
			for(i = 0; i < MAX_OUT_CTRL_CNT; i++)
			{
				OutCtrlCommCnt[i]++;

				if(OutCtrlCommCnt[i] >= (10 * INCTRL_TIMER_HZ))	/* 10s for communication fault. */
				{
					OutCtrlCommCnt[i] = 0;

					if(OutCtrlRsp[i].Comp1.bCompExist || OutCtrlRsp[i].Comp2.bCompExist)
					{
						InCtrl.OutError[i] |= OUTCTRL_ERR_COMM;
						OutCtrlRsp[i].Comp1.bCompExist = FALSE;   /* no corresponding comps.*/
						OutCtrlRsp[i].Comp2.bCompExist = FALSE;

						/* reset comp data. */
						memset(&CompStatus[i * 2], 0, sizeof(CompStatusT));
						CompStatus[i * 2].bCompExist = FALSE;
						CompStatus[i * 2].CurCompMode = COMP_WM_STOP;
						CompStatus[i * 2].ReqCompMode = COMP_WM_STOP;

						/* reset comp data. */
						memset(&CompStatus[i * 2 + 1], 0, sizeof(CompStatusT));
						CompStatus[i * 2 + 1].bCompExist = FALSE;
						CompStatus[i * 2 + 1].CurCompMode = COMP_WM_STOP;
						CompStatus[i * 2 + 1].ReqCompMode = COMP_WM_STOP;
					}
				}
			}
#endif
        }

        if ( Event & INCTRL_EVENT_ODC_DATAS )
        {
#if 0
        	UpdateOutdoorData(Event);
#endif
    	}

    }
}

extern const u8		SwVersion;

#ifdef  HMI_PRIVATE

/* get the Indoor controller informations. */ 
void InCtrlGetStatus(InCtrlRspT *pStatus)
{
	if(SysCfg & SYS_CFG_CTCH)
	{
            CtchGetStatus(pStatus);
            return;
	}

#if 0
	pStatus->DevInfo.SwVerNum = SwVersion;
	pStatus->DevInfo.SysUsed  = 0;

	pStatus->CtrlError = InCtrl.CtrlError;
	pStatus->RoomTemp = 200;
	pStatus->OutdoorTemp = InCtrl.OutdoorTemp;
	pStatus->RoomHumid = 0;
	pStatus->WorkMode = InCtrl.CurWorkMode;

	pStatus->bRemoteOn = InCtrl.bRemoteOn;

	pStatus->bIDFanOn = (bool)(InCtrl.IDFanSpeed != FAN_SPEED_STOP);
	pStatus->bHeater1On = (bool)(InCtrl.Heater1On == DEV_ON);

	pStatus->bFaultOn = (bool)(InCtrl.FaultOn == DEV_ON);
	pStatus->bHumidOn = 0;
#endif

}

#else

/* get the display informations. */
void InCtrlGetHmiStatus(HmiStatusT *pStatus)
{
//	pStatus->CtrlError      =   InCtrl.CtrlError;
}

#endif


typedef struct 
{
    u32 Flag;
    u32 FileLen;
    u8  Md5Code[16];
}SoftInfoT;

typedef __packed struct
{
    u16      StructVer;
    u16      StructLen;
    u32      FileLen;
    u32      FileVer;
    u8       Md5[16]; 
}UpdateHeaderT;

static UpdateHeaderT UpdateInfo;
static u32 UpdateOffset = 0;

static bool InCtrlFirmwareInfoProc(u8 * pCode , u8 Len)
{
    SoftInfoT SoftInfo;
    UpdateHeaderT* pUpdateHeader;
    
    pUpdateHeader = (UpdateHeaderT*)pCode;
    memcpy((u8*)&UpdateInfo, pCode , sizeof(UpdateHeaderT));
    //erase flash
    FlashEraseSoftInfo();
    FlashEraseSoftData();
    UpdateOffset = 0;
    //write soft info
    SoftInfo.FileLen = pUpdateHeader->FileLen;
    memcpy(SoftInfo.Md5Code , pUpdateHeader->Md5 , 16);
    return FlashWriteSoftInfo( 4 , (u8*)&SoftInfo.FileLen , 20 );       
}


static bool InCtrlFirmwareWriteProc( u32 Offset , u8 * pCode , u8 Len ) 
{
    if( UpdateOffset != Offset )
    {
        LTRACEF("Addr err, UpdateOffset = %d, Offset = %d, FileLen = %d", UpdateOffset , Offset , UpdateInfo.FileLen);
        return FALSE;
    }
    if( UpdateOffset + Len > UpdateInfo.FileLen )
    {
         LTRACEF("Len err, UpdateOffset = %d, Len = %d , FileLen =%d", 
                              UpdateOffset , Offset , UpdateInfo.FileLen );
         return FALSE;
    }
    if( UpdateOffset + Len == UpdateInfo.FileLen )
    {
        FlashWriteSoftData( Offset , pCode , Len );
        UpdateOffset += Len;
        if(IsSoftCheckOk(UpdateInfo.FileLen))
        {
            LTRACEF("Md5 Ok and Reset!!!");
            vTaskDelay( pdMS_TO_TICKS( 1 ) );

            ResetByWatchDog();
            return TRUE;
        }
        else
        {
            LTRACEF("Update Md5 error !!!");
            return FALSE;
        }
    }
    else
    {
        FlashWriteSoftData( Offset , pCode , Len );
        UpdateOffset += Len;
        LTRACEF("UpdateOffset = %d", UpdateOffset);
        return TRUE;
    }
}

bool InCtrlWriteFirmware(u32 Offset, u8 Len, u8 * pCode)
{
    
    if( Offset == 0xFFFFFFFA )
    {
        InCtrlFirmwareInfoProc(pCode , Len);
    }
    else
    {
        InCtrlFirmwareWriteProc(Offset, pCode , Len);
    } 
	return TRUE;
}

void UpdateToHmi(void)
{
}

/*=======================update the Hmi modbus ================*/

/* Now the Sw is used on our devices, the controllers communicate with Ui by HMI modbus protocol
    and we will not develop the Ui by ourself now. So we will mount the UI on the modbus bus, and mount all the 
    controllers on another bus.*/
/* transfer user setting to control task. */
bool InCtrlHmiUserSetting(HmiUserParamT * pUserSetting)
{
	if( !InCtrlInitOk )
		return FALSE;
  
      	/* Sys setting no changes, then return. */
	if( memcmp(&UserParam, pUserSetting, sizeof(HmiUserParamT)) == 0 )
        return TRUE;

	/* validate the params in structure pointed by pUserSetting. */
	if( (pUserSetting->SysSetting.WorkMode >= SYS_WM_MAX) || (pUserSetting->SysSetting.WorkMode == SYS_WM_STOP) )
	{
		UpdateToHmi();
		pUserSetting->SysSetting.WorkMode = SYS_WM_AUTO;
		/* return FALSE; */
	}

	if( (pUserSetting->SysSetting.SettingTemp > 350) || (pUserSetting->SysSetting.SettingTemp < 0) )
		return FALSE;
        
	/* copy data to UserParam */
	memcpy(&UserParam, pUserSetting, sizeof(HmiUserParamT));
    
	/* notify the control task. */
	xEventGroupSetBits( InCtrlEventHandle, INCTRL_EVENT_USER_DATA );

	return TRUE;	
}

/* transfer user setting to control task. */
bool InCtrlHmiDevParamPart1Setting(HmiDevParamPart1T * pDevPart1Setting)
{
	/* Sys setting no changes, then return. */
	if( memcmp(&DevPart1Setting, pDevPart1Setting, sizeof(HmiDevParamPart1T)) == 0 )
		return TRUE;

	/* validate the params in structure pointed by pSysSetting. */

	/* copy data to DevParam */
	memcpy(&DevPart1Setting, pDevPart1Setting, sizeof(HmiDevParamPart1T));

	return TRUE;
}

bool InCtrlHmiDevParamPart2Setting(HmiDevParamPart2T * pDevPart2Setting)
{
	/* Sys setting no changes, then return. */
	if( memcmp(&DevPart2Setting, pDevPart2Setting, sizeof(HmiDevParamPart2T)) == 0 )
		return TRUE;

	/* validate the params in structure pointed by pExvSetting. */

	/* copy data to SysParam */
	memcpy(&DevPart2Setting, pDevPart2Setting, sizeof(HmiDevParamPart2T));

	return TRUE;
}

bool InCtrlHmiDevParamPart3Setting(HmiDevParamPart3T * pDevPart3Setting)
{
	/* Sys setting no changes, then return. */
	if( memcmp(&DevPart3Setting, pDevPart3Setting, sizeof(HmiDevParamPart3T)) == 0 )
		return TRUE;

	/* validate the params in structure pointed by pFanSetting. */

	/* copy data to SysParam */
	memcpy(&DevPart3Setting, pDevPart3Setting, sizeof(HmiDevParamPart3T));

	return TRUE;
}

/* ========================used to update the corresponding model ===========*/

/* transfer user setting to control task. */
bool InCtrlSetUserParam(HmiUserParamT * pUserSetting, u8 ModId)
{
#if 0
	if(SysCfg & SYS_CFG_CTCH)
	{
		return CtchSetUserParam(pUserSetting, ModId);
	}
#endif

	if( !InCtrlInitOk )
		return FALSE;

	xSemaphoreTake(ParamSemHandle, portMAX_DELAY);

	if((UserParamDirtyFlag & ModId) == 0)
	{
		UserParamDirtyFlag = ~(ModId);  /* Set all dirty flags except of the caller. */

		/* validate the params in structure pointed by pUserSetting. */
		/* copy data to UserParam for valid members. */
        	/* Sys setting no changes, then return. */
        if( memcmp(&UserParam, pUserSetting, sizeof(HmiUserParamT)) == 0 )
		return TRUE;

        /* validate the params in structure pointed by pFanSetting. */

        /* copy data to SysParam */
        memcpy(&UserParam, pUserSetting, sizeof(HmiUserParamT));
	}

	xSemaphoreGive(ParamSemHandle);

	/* notify the control task. */
	xEventGroupSetBits( InCtrlEventHandle, INCTRL_EVENT_USER_DATA );

	return TRUE;
	
}

bool InCtrlSetOdcStatus(u8 idx, OutCtrlRspT * pOutRsp)
{
	if(SysCfg & SYS_CFG_CTCH)
	{
		return CtchSetOdcStatus(idx, pOutRsp);
	}

    EventBits_t		Event;

	if( !InCtrlInitOk )
		return FALSE;

	if( idx >= MAX_OUT_CTRL_CNT)
	{
		return FALSE;
	}

	memcpy(&OutCtrlRsp[idx], pOutRsp, sizeof(OutCtrlRspT));

	Event = (INCTRL_EVENT_ODC1_DATA << idx);

	/* notify the control task. */
	xEventGroupSetBits( InCtrlEventHandle, Event );

	return TRUE;
}

/* get the Outdoor controller informations. */  
bool InCtrlGetOdcCmd(u8 idx, OutCtrlCmdT *pCmd)
{
#if 0
	if(SysCfg & SYS_CFG_CTCH)
	{
		return CtchGetOdcCmd(idx, pCmd);
	}

	if(SysParam.CompStartInterval >= (SysParam.CompMinStopTime + 90))
		pCmd->CompMinRunTime  = SysParam.CompStartInterval - SysParam.CompMinStopTime;
	else
		pCmd->CompMinRunTime  = 90;  /* at least 90s. */
		
	pCmd->CompMinStopTime = SysParam.CompMinStopTime;

	if(pCmd->CompMinStopTime < 150)
		pCmd->CompMinStopTime = 150;	/* at least 150s. */

	pCmd->MaxCompCurrent = SysParam.MaxCompCurrent;
	pCmd->MaxFanCurrent  = SysParam.MaxFanCurrent;

	if( idx >= MAX_OUT_CTRL_CNT)
	{
		pCmd->ReqWorkMode1 = COMP_WM_UNCHANGED;
		pCmd->ReqWorkMode2 = COMP_WM_UNCHANGED;
		
		return FALSE;
	}

	pCmd->ReqWorkMode1   = CompStatus[idx * 2].ReqCompMode;
	pCmd->InnerCoilTemp1 = CompStatus[idx * 2].InnerCoilTemp;

	pCmd->ReqWorkMode2   = CompStatus[idx * 2 + 1].ReqCompMode;
	pCmd->InnerCoilTemp2 = CompStatus[idx * 2 + 1].InnerCoilTemp;
#endif
	return TRUE;
}

/* read/write EEPROM. */

#define DBM_SYS_DATA_BASE			0x0020

static bool WriteNvSysData(EepromSysDataT * pParam)
{
	u16				offset;

    pParam->VerNo = SwVersion;
    
    /* Zone the map according to the version number */
	offset = ((u16)(pParam->VerNo & 0x0F)) * sizeof(EepromSysDataT);
    pParam->SysPreHeatTime = InCtrl.PreHeatTime;
	pParam->CheckSum = pParam->VerNo + pParam->SysPreHeatTime;

	EepromWrite(DBM_SYS_DATA_BASE + offset, (u8 *)pParam, sizeof(EepromSysDataT));

	return TRUE;
}

static bool ReadNvSysData(EepromSysDataT * pParam)
{
	EepromSysDataT   temp;
	u16			     offset = 0;

	pParam->VerNo = 0;
	pParam->SysPreHeatTime = 0;

    EepromRead(DBM_SYS_DATA_BASE + offset, (u8 *)&temp, sizeof(EepromSysDataT));

    if(temp.CheckSum == (temp.VerNo + temp.SysPreHeatTime))
    {
      
        pParam->VerNo = temp.VerNo;
        pParam->SysPreHeatTime = temp.SysPreHeatTime;         
        return TRUE;
    }

    return FALSE;	
}

static bool SaveNvSysData(void)
{
	/*if(InCtrl.CurWorkMode == COMP_WM_STOP)*/
	{
        if(InCtrl.PreHeatTime>20*60*INCTRL_TIMER_HZ)
        {
            WriteNvSysData(&NvSysData);
            InCtrl.PreHeatTime = 0;
            return TRUE;
        }
	}
	return FALSE;
}

/*****
* History
* Initial version.
*
*/
