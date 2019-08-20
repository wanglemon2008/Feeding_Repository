#ifndef __INDOOR_CTRL_H__
#define __INDOOR_CTRL_H__

#include "OutdoorCtrl.h"

#if 0
/* system(Indoor Controller) fault. */
#define INCTRL_ERR_PUMP     		0x0001
#define INCTRL_ERR_SYS_OUT_SENSOR   0x0002
#define INCTRL_ERR_SYS_BACK_SENSOR	0x0004


#define INCTRL_ERR_INDOOR_FAN		0x0008
#define INCTRL_ERR_HEATER			0x0010
#define INCTRL_ERR_FAN_PRESS_DIF	0x0020
#define INCTRL_ERR_FILTER			0x0040
#define INCTRL_ERR_FIRE_ALARM		0x0080

#define INCTRL_ERR_HUMIDIFIER		0x0100
#define INCTRL_ERR_WATER_LEAK		0x0200

/* Sensor fault. */
#define INCTRL_ERR2_TEMP1_SENSOR    0x00000001
#define INCTRL_ERR2_TEMP1_SENSOR			0x00000001
#define INCTRL_ERR2_TEMP1_SENSOR			0x00000001
#define INCTRL_ERR2_TEMP1_SENSOR			0x00000001

typedef enum
{
	SYS_WM_STOP = 0,
	SYS_WM_COOL,
	SYS_WM_HEAT,
	SYS_WM_FAN,
	SYS_WM_DEHUMIDIFY,
	SYS_WM_AUTO,
	SYS_WM_MAX,
} SysWorkModeT;

typedef enum
{
	FAN_SPEED_STOP = 0,
	FAN_SPEED_LOW,
	FAN_SPEED_MID,
	FAN_SPEED_HIGH,
	FAN_SPEED_AUTO,
	FAN_SPEED_MAX,
} FanSpeedT;

typedef enum
{
	CTCH_WM_STOP = 0,
	CTCH_WM_STERILIZE,
	CTCH_WM_DETOX,
	CTCH_WM_FAN,
	CTCH_WM_CTCH,
	CTCH_WM_AUTO,
	CTCH_WM_MAX,
} CtchWorkModeT;

typedef __packed struct
{
	u8				bCompExist;		/* 1: comp-1 existed, 0: not existed. */
	CompWorkModeT	WorkMode;
	u16				CompError;

	u32				TotalWorkTime;     /* unit in minute. */

	s16				OutCoilTemp;
	s16				InnerCoilTemp;
	u16				CompCurrent;
	u16				ODFanCurrent;
} HmiCompStatusT;

#if 0
/* used to display system status in LCD. */
typedef __packed struct
{
	HmiCompStatusT	CompStat[8];
	u16 			CtrlError;
	u8				OutError[4];
	s16				RoomTemp;
	s16				OutdoorTemp;
	u8				RoomHumid;

	SysWorkModeT	WorkMode;		/* current work mode. */

	u8			bIDFanOn   :  1;	
	u8			bHeater1On :  1;
	u8			bHeater2On :  1;	
	u8			bHeater3On :  1;

	u8			bOzoneOn     :  1;
	u8			bFanValve1On :  1;
	u8			bFanValve2On :  1;
	u8			bFaultOn     :  1;

	u8			bRemoteOn : 1;
	u8			bHumidOn  : 1;

} HmiStatusT;

#endif

typedef __packed struct
{  

	s16			RoomTemp;			/* from extend board. */
	u16			RoomHumidity;		/* from extend board. */
	u16			WindPress;			/* from extend board. */
	u16			WindSpeed;			/* from extend board. */

     
	u32 		CtrlError2;
	u16 		CtrlError;
    
    u8          WorkMode;
	u8			LowFilterDifOnT   :  1;
    u8          ExhFanOnT         :  1;
	u8			MidFilterDifOnT   :  1;
	u8			HighFilterDifOnT  :  1;	
	u8			AirPressDifOnT      :  1;	    
	u8			HumidiOnT         :  1;
	u8			RemoteOnT         :  1;
    
    u16			PreFanPwmData;
  
    u16			A;
    
    u16			HumidifierPwmData;    

    u16			EEV1TargetStep;
    u16			NowEEV1StepOut;
    u16			EEV2TargetStep;
    u16			NowEEV2StepOut;
    u16			EEV3TargetStep;
    u16			NowEEV3StepOut;
    

	u8			Heater11n :  1;	
	u8			Heater12n :  1; 
	u8			Heater13n :  1;	
	u8			Heater21n :  1;   
    u8			Heater22n :  1;	
	u8			Heater23n :  1;  
   
    s16         bacup3;//reserved
    u16         PowerRequest;
    u16         OutWorkMode;
    s16         PreHeaterTemp;    
       
    u16         MachineType;
    
    /*calculation value*/
    
    s16			FroCoilEntrTemp;
    s16			FroCoilTemp;
	s16			FroCoilExitTemp;
    s16			PreCoolAvgOverHeat;
    s16			PreCoolAi;
    s16         backup1;
    s16         backup2;
    s16         backup3;
    
	s16			Heat1EntrTemp;
	s16			Heat1ExitTemp;
    s16			EvaCoil1AvgOverHeat;
    s16			EvaCoil1Ai;
    s16         backup4;
    s16         backup5;
    
    s16			Heat2EntrTemp;
	s16			Heat2ExitTemp;
	s16			EvaCoil2AvgOverHeat;
    s16			EvaCoil2Ai;
    s16         backup6;
    s16         backup7;
    
    s16         Ap;
    s16         Ad;
    
} HmiStatusT;

#define SYSCFG_HEAT_PUMP     0
#define SYSCFG_CTCH          1

/* system setting for control program. */
typedef __packed struct 
{
    u8	       Type;
    u8	       Model;
    u8         CompNo;
    u8         CtrlMemb;
    u8         FanType;
    u8         FanModel;
    u8         PreHeatTime;
    u8         RemoteOnOffType;
//    u8         RemoteOnOff;		//新增
    u8         UpperLimitHeatTemp;
    u8         LowerLimitHeatTemp;
    u8         UpperLimitCoolTemp;
    u8         LowerLimitCoolTemp;
    u8         MemoryOnOff;
    u8         AutoPowerOn;
    u8         WaterSwitchOnOff;    
    u8         PowerOn;
    u8         WelcomeOnOff;
//    u8         FlagMannualCtrl;//新增
            
    /* old prarmeter modified by chi */
    u16			SysConfig;			/* 0: heat pump, 1: CTCH. */
	u16			TempErrorNum;
	u16			TempDiffForLoad;
	u16			TempDiffForUnload;
	u16			CompStartInterval;
	u16			CompMinStopTime;
	u16			LoadDelay;
	u16			UnloadDelay;
	u16			QuitDefrostTemp;	/* temperature for quit defrost. 100~150(10C~15C), default is 100(10C). */
	u16			IdfLeadTime;
	u16			IdfDelayTime;
	u16			DefrostInterval;
	u16			MaxCompCurrent;
	u16			MaxFanCurrent;
	u16			MaxDehumidComps;
    
} HmiSysParamT;


typedef __packed struct 
{
	u16			HumidErrorNum;
	u16			DiffForHumidify;
	u16			DiffForDehumidify;
	u16			HumidFirst;
	s16			LowestTemp;
	s16			StopTempDiff;
	s16			RollbackTempDiff;
	u16			SterilizeTime;
	u16			DetoxTime;
	u16			LoadDelay;		/* humidifier load delay. */
	u16			UnloadDelay;	/* humidifier unload delay. */
} HmiHumidParamT;

/* user setting for control program. */  /* PL01 PL02 */
typedef __packed struct
{
    bool	         bPowerOn;
    SysWorkModeT	WorkMode;
    u8	            bPid;     //test
    s8			   CoolSettingTemp;
    s8			   HeatSettingTemp;
    s8             RecySettingTemp;
    
    u8             bModel9:1;
    u8             bModel10:1;
    u8             bModel11:1;
    u8             bModel12:1;
    u8             bModel13:1;
    u8             bModel14:1;
    u8             bModel15:1;  
    u8             bModel16:1;  
    u8             bModel1:1;
    u8             bModel2:1;
    u8             bModel3:1;
    u8             bModel4:1;
    u8             bModel5:1;
    u8             bModel6:1;
    u8             bModel7:1;
    u8             bModel8:1;  
    u8             ModelNo;
    
    s16            SettingTemp;
    u16            SettingHumidity;
    bool            bManualDefrost;
    u16             FanSpeed;
    u8              CompIdx;
    
} HmiUserParamT;


/* indoor controller cmd. */
typedef __packed struct
{
	SlaveDataSegT	DataSeg;
} InCtrlCmdT;

/* indoor controller response msg. */
typedef __packed struct
{
	SlaveDataSegT	DataSeg;

	SlaveDevInfoT   DevInfo;

	u16 			CtrlError;
	s16				RoomTemp;
	s16				OutdoorTemp;
	u8				RoomHumid;

	SysWorkModeT	WorkMode;		/* current work mode. */

	u8			bIDFanOn   :  1;	
	u8			bHeater1On :  1;
	u8			bHeater2On :  1;	
	u8			bHeater3On :  1;

	u8			bOzoneOn     :  1;
	u8			bFanValve1On :  1;
	u8			bFanValve2On :  1;
	u8			bFaultOn     :  1;

	u8			bRemoteOn  : 1;
	u8			bHumidOn   : 1;

} InCtrlRspT;

/* ModleMachine parameter setting */ /* PL04 */
typedef __packed struct 
{
    u8         TempCtrlCycleTime;
    u8         UpLoadTempOffset;
    u8         UnLoadTempOffset;
    u8         TempAutoHeatTemp;
    u8         TempAutoCoolTemp;
    
} HmiEnergyCtrlParamT;

/* PL05 */
typedef __packed struct  
{        
    u8         Valve4Type;
    u8         Valve4DelayOnTime;
    u8         Valve4DelayOffTime;//32
    u8         WaterPumpDelayOnTime;//33
    u8         WaterPumpDelayOffTime;
    u8         PreFanDelayPauseTime;    
}HmiSysEvaValveParamT;

/* PL06 */
typedef __packed struct 
{        
    s8         CoolFanDelayOnTime;
    s8         CoolFanDelayOffTime;
    s8         HeatFanDelayOnTime;
    s8         HeatFanDelayOffTime;
    u8         CoolMulFanTemp;   
    u8         HeatMulFanTemp;
//    u8         PreFanDelayTime;//删除
}HmiSysFanParamT;

/* PL07 */
typedef __packed struct 
{        
    u8         DeadTime;//压缩机防频繁启动
    u8         MinRunTime;
    u8         PowerOnInternalTime;//修改
    u8         FirstPowerOnTempDif;
    u8         RunPowerOffTime;  
}HmiSysCompParamT;

/* PL08 */
typedef __packed struct 
{        
    u8         FaultDelayTime;
    u8         WaterLessDelayTime;
    u8         FaultRstTime;
    u8         AutoRstTime;
    u8         LowPressFaultDelayTime;
    u8         ExitDefrostRecoveryLowPressTime;  
    u8         CoolLowPressDeadTime;
    u8         HeatLowPressDeadTime;
    u8         ExitTempProtectTempDif;
    u8         CoolOutOverLowTemp;
    u8         HeatOutOverHighTemp;
    u8         EvaOverHeatTemp;
    u8         ExitEvaOverHeatProtectTempDif;
    u8         AmpeRated;
    u8         AmpeLow;
    u8         AmpeDetectDelayTime;
    u8         AmpeUsedSet;
    u8         AmpeDisMul;//电流显示倍数   //64
    u8         ExhaustOverHeatTemp;//65
    u8         ExitExhaustOverHeatTempDif;    
}HmiSysProtectParamT;

/* PL09 */
typedef __packed struct 
{        
    u8         MaxDefrostCompNum;
    u8         FistDefrostMinRunTime;
    u8         DefrostTemp1;
    s8         DefrostTemp2;
    u8         DefrostIntervalTime1;
    u8         DefrostIntervalTime2;  
    u8         DefrostIntervalTime3;
    u8         DefrostTime;
    u8         DefrostPermissionTemp;
    s8         DefrostPermissionEvaTemp;
    u8         DefrostPermissionTempDif1;
    u8         DefrostPermissionTempDif2;
    u8         ExitDefrostEvaTemp;
    u8         ExitDefrostExitWaterTemp;
    u8         LowPressDefrostDetectDelayTime;
    u8         LowPressDefrostIntervalTime;
    s8         LowPressDefrostEvaTemp;
    u8         ExitDefrostFanDelayTime;
    u8         LowPressDefrostUsedSet;
    u8         CompFrostTimeSet;    
    u8         DefrostCompOnTime;
    u8         LowPressDefrostEvaTempDif;
    u8         FirstDefrostTemp;
    s8         FirstDefrostEvaTemp;
    u8         FirstDefrostEvaTempDif;
}HmiSysDefrostParamT;

/* PL10 */
typedef __packed struct 
{        
    u8         PreFrzInterval1;
    u8         PreFrzInterval2;
    s8         PumpPreFrzTemp;
    s8         HeatPreFrzTemp;
    s8         CompPreFrzTemp;//96
    s8         ExitHeatPreFrzTemp;  //97
    s8         ExitCompPreFrzTemp;
    s8         PreFrzIntervalTemp;
    s8         EnterPreFrzTemp;
    s8         ExitPreFrzTempDif;
    u8         PreFrzUsedSet;

}HmiSysPreFrzParamT;

/* PL11 */
typedef __packed struct 
{        
    s8         HeaterOnTemp;
//    s8         DeltaUpLoadTemp;
//    s8         HeaterEnvTemp;
}HmiSysHeaterParamT;

/* PL12 */
typedef __packed struct 
{
    /*Below Exv Data*/ 
    u8          ExvUsedSet;
    u8          ExcitationMode;
    u8          ExcitationFreq;
    u8          MotorSum;
    u8          InitPlus;
    u8          AwaitPlus;
    u8          InitPlusKeepTime;
    u8          CoolInitOpeningAmpK;    
    u8          CoolInitConvEvaTemp;   
    u8          InTargetOverHeat;    
    u8          CoolMinOpening;
    u8          MinCoolExhAirOverHeat;
    u8          HeatInitOpeningAmpK;
    u8          HeatInitConvEvaTempDif;
    s8          ExhOverHeatTargetModify;   
    u8          MinExhOverCool; 
    u8          MinHeatOpening;
    u8          MaxValveBackTemp;
    u8          DefrostFixPLus;  
    u8          ExhProtectTemp;    
    s8          HeatTurnOnModifyK; 
    u8          HeatTurnOnModifyTime; 
    u8          ExhOverCoolModifyDead; 
    u8          MinCoolInitOpening;
    u8          MaxCoolInitOpening; //128
    u8          MinHeatInitOpening;//129
    u8          MaxHeatInitOpening;  
    u8          MaxCoolRuningOpening;
    
    
} HmiSysExvParamT;

/* PL13 */
typedef __packed struct 
{
    /*Below Exv Data*/ 
    u8          P;
    u8          I;
    u8          D;
    u8          ExvRuingCycle;
    u8          ExvCalCycle;//*10  暂时程序中用的最小为1s
    u8          ExvFixedStep;//开关阀步数
    u8          ExvCoolMannualOpening1;
    u8          ExvCoolMannualOpening2;    
    u8          ExvHeatMannualOpening1;   
    u8          ExvHeatMannualOpening2;   
    u8          ExvHeatInitOpeningKeepTime;  
    
    u8          CoolLowPressOpening;
    u8          CoolLowPressIntervalTime;
    u8          HeatLowPressOpening;
    u8          HeatLowPressIntervalTime;

} HmiSysPIDParamT;

/* PL14 */
typedef __packed struct 
{
    /*Below Exv Data*/ 
    u8          bFan2OverLoadSwitch :    1;   
    u8          bComp2OverLoadSwitch :   1;   
    u8          bComp2HPressSwitch   :   1;    
    u8          bComp2LPressSwitch   :   1;
    u8          bFan1OverLoadSwitch :    1;   
    u8          bComp1OverLoadSwitch :   1;   
    u8          bComp1HPressSwitch   :   1;    
    u8          bComp1LPressSwitch   :   1;
    u8          bWaterLessSwitch    :   1;
    u8          bPumpOverLoadSwitch :   1;
    u8          bRemoteOnSwitch     :   1;
    u8          bClainSwitch        :   1;
    u8          bFrzSwitch          :   1;
    u8          bPowerFaultSwitch   :   1;
    u8          bModeSwitch         :   1;
    u8          bbackup             :   1;     
} HmiSysSwitchConstParamT;

/* PL15 */
typedef __packed struct 
{
    /*Below Exv Data*/ 
    s8          OutWaterTemp;
    s8          RstWaterTemp;
    s8          OutdoorTemp;
    s8          HeatRecyTemp;

} HmiSysTempModifyParamT;

/* PL16 */
typedef __packed struct 
{
	/*Below Exv Data*/ 
	u8	Eva1OutProbeSet;
	u8	Eva1ProbeSet;
	u8	Eva2ProbeSet;
	u8	SysCareTimeH;
	u8	SysCareTimeL;
	u8	SensorTypeUsed;
	u8	RefigerantChosen;
	u8	PressSensorType;//160

} HmiSysTempProbeSetParamT;

#if 0
typedef __packed struct 
{
	HmiUserParamT		UserParam;
    HmiSysParamT 	SysParam;
    HmiEnergyCtrlParamT 	EnergyCtrlParam;
    HmiSysEvaValveParamT	SysEvaValveParam;
	HmiSysFanParamT		SysFanParam;
	HmiSysCompParamT	SysCompParam;
	HmiSysProtectParamT		SysProtectParam;
	HmiSysDefrostParamT		SysDefrostParam;
	HmiSysPreFrzParamT		SysPreFrzParam;
	HmiSysHeaterParamT		SysHeaterParam;
	HmiSysExvParamT		SysExvParam;
	HmiSysPIDParamT		SysPIDParam;
	HmiSysSwitchConstParamT	SysSwitchConstParam;
	HmiSysTempModifyParamT		SysTempModifyParam;
	HmiSysTempProbeSetParamT	SysTempProbeSetParam;

}AllParamSettingT;

#endif

/*========================backup========================*/
/*factory setting for control program*/
//typedef __packed struct 
//{
//    u16     HavePreCool;
//    u16     PreCoolMethod;
//    u16     HavePreHeat;
//    u16     PreHeatMethod;
//    s16     PreCoolTargetTemp;
//    s16     PreHeatTargetTemp;
//    u16     HeaterType;//电加热、热水、蒸汽
//    u16     HaveSecEva;//二级氟盘管
//    u16     HumiType;
//    u16     HaveHeat;
//    u16     HeaterNum;//加热段分段
//    u16     PreFanType;
//    u16     PreFanCtrl;//送风机控制方式
//    u16     TempCtrlType;
//    u16     NTCmodify;
//    u16     InWindType;
//    u16     HumiDead;
//    u16     FanValveDelayTime;
////    SwitchInitStatus1T     SwitchInitStatus1;//所有开关的常开常闭状态
//    u16     backup;
//    u16     PreCoolDead;//预处理段死区值
//    u16     DeltaPreCool;//预处理段精度值
////    SwitchInitStatus2T     SwitchInitStatus2;//所有开关的常开常闭状态     
//    u16     backup1;
//}HmiFactoryParamT;
//
///* used to display OutRoom status in LCD. */
//typedef __packed struct
//{
//	s16			OutRoomTemp1;			/* from extend board. */
//	s16			OutRoomTemp2;		/* from extend board. */
//	s16			OutRoomTemp3;			/* from extend board. */
//	s16			OutRoomTemp4;			/* from extend board. */
//    
//    u8          bOutDefrost1n : 1;
//    u8          bOutDefrost2n : 1;
//    u8          bOutDefrost3n : 1;
//    u8          bOutDefrost4n : 1;
//    
//	u16 		Out1CtrlError71;
//	u16 		Out1CtrlError72;
//	u16 		Out1CtrlError73;
//	u16 		Out1CtrlError74;
//	u16 		Out1CtrlError75;
//    
//	u16 		Out2CtrlError71;
//	u16 		Out2CtrlError72;
//	u16 		Out2CtrlError73;
//	u16 		Out2CtrlError74;
//	u16 		Out2CtrlError75;
//
//	u16 		Out3CtrlError71;
//	u16 		Out3CtrlError72;
//	u16 		Out3CtrlError73;
//	u16 		Out3CtrlError74;
//	u16 		Out3CtrlError75;
//    
//	u16 		Out4CtrlError71;
//	u16 		Out4CtrlError72;
//	u16 		Out4CtrlError73;
//	u16 		Out4CtrlError74;
//	u16 		Out4CtrlError75;
//    
//    s16			TargetTempMonitor;			/* from user SettingTemp. */
//	u16			TargetHumidityMonitor;    
//	u16			PowerOnMonitor;		/* outdoor system power on. */   
//    u16         WorkMode;
//
//} HmiOutStatusT;


#define MOD_ID_HMI		0x0001
#define MOD_ID_MODBUS	0x0002
#define MOD_ID_GATEWAY	0x0004

extern void     InCtrlGetHmiStatus(HmiStatusT *pStatus);
extern void     InCtrlGetStatus(InCtrlRspT *pStatus);

extern bool     InCtrlHmiUserSetting(HmiUserParamT * pUserSetting);
extern bool     InCtrlHmiSysSetting(HmiSysParamT * pSysSetting);
extern bool     InCtrlHmiSysExvParam(HmiSysExvParamT * pExvSetting);
extern bool     InCtrlHmiSysFanParam(HmiSysFanParamT * pFanSetting);
extern void     InCtrlGetStatus(InCtrlRspT *pStatus);
extern void     InCtrlGetStatus(InCtrlRspT *pStatus);
extern void     InCtrlGetStatus(InCtrlRspT *pStatus);
extern void     InCtrlGetStatus(InCtrlRspT *pStatus);

/*==============================used for wds display=========================*/
extern bool		InCtrlSetUserParam(HmiUserParamT * pUserSetting, u8 ModId);/* ModId :1 used for dirty,0 for not dirty */
extern bool		InCtrlSetSysParam(HmiSysParamT * pSysSetting, u8 ModId);
extern bool 	    InCtrlSetHumidParam(HmiHumidParamT * pHumidSetting, u8 ModId);
extern bool		InCtrlSyncUserParam(HmiUserParamT * pUserSetting, u8 ModId);
extern bool		InCtrlSyncSysParam(HmiSysParamT * pSysSetting, u8 ModId);
extern bool 	    InCtrlSyncHumidParam(HmiHumidParamT * pHumidSetting, u8 ModId);


extern bool 	    InCtrlGetOdcCmd(u8 idx, OutCtrlCmdT *pCmd);
extern bool 	    InCtrlSetOdcStatus(u8 idx, OutCtrlRspT * pOutRsp);
extern void 	    InCtrlTask( void *pvParameters );
extern bool 	    InCtrlWriteFirmware(u32 Offset, u8 Len, u8 * pCode);


/*====================== CONSTANT temperature & CONSTANT humidity ========================*/

extern void		CtchGetHmiStatus(HmiStatusT *pStatus);
extern void 	    CtchGetStatus(InCtrlRspT *pStatus);
//extern bool		CtchSetUserParam(HmiUserParamT * pUserSetting, u8 ModId);
extern bool		CtchSetSysParam(HmiSysParamT * pSysSetting, u8 ModId);
extern bool 	    CtchSetHumidParam(HmiHumidParamT * pHumidSetting, u8 ModId);
extern bool 	    CtchGetOdcCmd(u8 idx, OutCtrlCmdT *pCmd);
extern bool 	    CtchSetOdcStatus(u8 idx, OutCtrlRspT * pOutRsp);

extern bool		CtchSyncUserParam(HmiUserParamT * pUserSetting, u8 ModId);
extern bool		CtchSyncSysParam(HmiSysParamT * pSysSetting, u8 ModId);
extern bool 	    CtchSyncHumidParam(HmiHumidParamT * pHumidSetting, u8 ModId);

extern void 	    CtchTask( void *pvParameters );
#endif

/*=====================Feeding parameter framework=========================================*/

/* system(Indoor Controller) fault. */
#define INCTRL_ERR_FEED1		0x0001
#define INCTRL_ERR_FEED2		0x0002
#define INCTRL_ERR_TEMP_L		0x0004
#define INCTRL_ERR_TEMP_H		0x0008
#define INCTRL_ERR_FAN1		    0x0010
#define INCTRL_ERR_FAN2		    0x0020
#define INCTRL_ERR_FAN3		    0x0040
#define INCTRL_ERR_FAN4		    0x0080
#define INCTRL_ERR_L2		    0x0100
#define INCTRL_ERR_L3		    0x0200
#define INCTRL_ERR_COMM		    0x0400




/* Sensor fault. */
#define INCTRL_ERR2_TEMP1_SENSOR        0x00000001
#define INCTRL_ERR2_TEMP2_SENSOR	    0x00000002
#define INCTRL_ERR2_DIF_SENSOR		    0x00000004





typedef enum
{
	SYS_WM_STOP = 0,
	SYS_WM_MAN,
	SYS_WM_AUTO,
	SYS_WM_MAX,
} SysWorkModeT;

typedef __packed struct
{
	u16 		InCtrlError1;
	u16 		InCtrlError2;
	u16 		InCtrlError3;
} CtrlErrorT;

typedef __packed struct
{  
#if 0
	s16			RoomTemp;			/* from extend board. */
	u16			RoomHumidity;		/* from extend board. */
	u16			WindPress;			/* from extend board. */
	u16			WindSpeed;			/* from extend board. */

	u32 		CtrlError2;
	u16 		CtrlError;
#endif
    
    /* 200 bytes  */
    u16     Relay1RuningTime;
    u16     Relay1Times;
    u16     Relay2RuningTime;
    u16     Relay2Times;
    u16     Relay3RuningTime;
    u16     Relay3Times;
    u16     Relay4RuningTime;
    u16     Relay4Times;
    u16     Relay5RuningTime;
    u16     Relay5Times;
    u16     Relay6RuningTime;
    u16     Relay6Times;
    u16     Relay7RuningTime;
    u16     Relay7Times;
    u16     Relay8RuningTime;
    u16     Relay8Times;
    u16     Relay9RuningTime;
    u16     Relay9Times;
    u16     Relay10RuningTime;
    u16     Relay10Times;
    u16     Relay11RuningTime;
    u16     Relay11Times;
    u16     Relay12RuningTime;
    u16     Relay12Times;
    u16     Relay13RuningTime;
    u16     Relay13Times;
    u16     Relay14RuningTime;
    u16     Relay14Times;
    u16     Relay15RuningTime;
    u16     Relay15Times;
    u16     Relay16RuningTime;
    u16     Relay16Times;
    u16     ChangeVol1RuningTime;
    u16     ChangeVol1Times;
    u16     ChangeVol2RuningTime;
    u16     ChangeVol2Times;
    u16     ChangeVol3RuningTime;
    u16     ChangeVol3Times;
    u16     ChangeVol4RuningTime;
    u16     ChangeVol4Times;
//#if 0
    u16     backup1;
    u16     backup2;
    u16     backup3;
    u16     backup4;
    u16     backup5;
    u16     backup6;
    u16     backup7;
    u16     backup8;
    
    u16     FC1RuningTime;
    u16     FC1Times;
    u16     FC2RuningTime;
    u16     FC2Times;
    u16     FC3RuningTime;
    u16     FC3Times;
    
    u16     backup2_1;
    u16     backup2_2;
    u16     backup2_3;
    u16     backup2_4;
    u16     backup2_5;
    u16     backup2_6;
    u16     backup2_7;
    u16     backup2_8;  
    u16     backup2_9;
    u16     backup2_10;     
    
    u16     RunCtrlInfo;
    
    /* Device running information */
    u16   Time;
    u16   Date;
    u16   CurCtrlTemp1;
    u16   CurCtrlTemp2;
    u16   CurCtrlTemp3;
    u16   YdayAvgTemp;
    u16   BfYDayAvgTemp;
    u16   Bf3DayAvgTemp;
    u16   Bf4DayAvgTemp;
    u16   Bf5DayAvgTemp;
    u16   AmpSensor1;
    u16   AmpSensor2;
    u16   ResSensor1;
    u16   ResSensor2;
    
    u16     backup3_1; 
    u16     backup3_2; 
    u16     backup3_3; 
    u16     backup3_4; 
    u16     backup3_5; 
    u16     backup3_6; 
    u16     backup3_7; 
    u16     backup3_8; 
    u16     backup3_9; 
    u16     backup3_10;    
    u16     backup3_11; 
    u16     backup3_12;
    u16     backup3_13; 
    u16     backup3_14; 
    
    CtrlErrorT  CtrlError;
    
    /* Sensor data */
    u16   IndoorUnitFrontTemp;
    u16   IndoorUnitFrontHumid;
    u16   IndoorUnitBackTemp;
    u16   IndoorUnitBackHumid;
    u16   CO2;
    u16   PM25;
    u16   PM100;
//#endif
} HmiStatusT;

typedef __packed struct 
{
    SysWorkModeT	WorkMode;
    u16     bPowerOn;
    u16     MachineType;
    u16     RoomType;
    u16     RS485Baud1;
    u16     RS485Baud2;
    u16     CANBaud;
    u16     ModifiedTime;
    /*u16     CurCtrlTemp;*/
    u16     SettingTemp;
    u16     ModifiedDate;
    u16     CurCtrlCoreVol;
    u16     Temp1Adj;
    u16     Temp2Adj;
    u16     Temp3Adj;
    u16     TempAutoAdj;
    u16     Vol12OnOff;
    u16     FarmName1;
    u16     FarmName2;
    u16     FarmName3;
    u16     FarmName4;
    u16     FarmName5;
    u16     FarmName6;
    u16     FarmName7;
    u16     FarmName8;
    u16     HWVersion;
    u16     DSHWVersion;
    u16     RecovSWOnOff;
    u16     UpdateSWOnOff;
    u16     TransSWOnOff;
    u16     AllTransSWOnOff;
    u16     ParaTransOnOff;
    u16     AllParaTransOnOff;
    u16     OldTempLocation;
    u16     OldTempDisOnOff;
    u16     SysErrLocation;
    u16     VarSpeedDuty1;
    u16     VarSpeedDuty2;
    u16     VarSpeedDuty3;
    u16     VarSpeedDuty4;
    u16     backup1;
    u16     backup2;
    u16     backup3;
    u16     backup4;
    u16     OnOffTempSettingInter;
    u16     FeedCurDate;
    u16     FeedMode;
    u16     Sim_Uid;
    u16     Sim_Id;
    u16     CalCrc;   
    
    u16     backup5;
    u16     backup6;
    u16     backup7;
    
}HmiSysParamT;

typedef __packed struct 
{
  u8     bConstSpeed1OnOff:1;
  u8     bConstSpeed2OnOff:1;
  u8     bConstSpeed3OnOff:1;
  u8     bConstSpeed4OnOff:1;
  u8     bConstSpeed5OnOff:1;
  u8     bConstSpeed6OnOff:1;
  u8     bConstSpeed7OnOff:1;
  u8     backup1:1;
  
  u8     bBreakerOnOff:1;
  u8     bInllumOnOff:1;
  u8     bWarmLightOnOff:1;
  u8     bSprayMore:1;
  u8     bSprayLess:1;
  u8     bDrinkOnOff:1;
  u8     bFeedOnOff:1;
  u8     bL2PowerOnOff:1;
  
  u8     bL3PowerOnOff:1;
  u8     bAdjVolFan1:1;
  u8     bAdjVolFan2:1;
  u8     bAdjVolFan3:1;
  u8     bAdjVolFan4:1;
  u8     backup2:1;
  u8     backup3:1;
  u8     backup4:1;
  
  u8     backup5:1;
  u8     bTempSensor1OnOff:1;
  u8     bTempSensor2OnOff:1;
  u8     bTempSensor3OnOff:1;
  u8     bFC1OnOff:1;
  u8     bFC2OnOff:1;
  u8     bFC3OnOff:1;
  u8     backup6:1;
  
  u8    backup7:1;
  u8    backup8:1;
  u8    backup9:1;
  u8    backup10:1;
  u8    bIndoorUnitFront:1;
  u8    bIndoorUnitBack:1;
  u8    bTempNoPlusDet:1;
  u8    bTempChangeDet:1;
  
  u8    bCO2TempHumid:1;
  u8    bAirSensor:1;
  u8    bHumidCtrl:1;
  u8    bAirCtrl:1;
  u8    bPoleMoto:1;
  u8    bPoleOutAirMoto:1;
  u8    bRoomInAirMoto:1;
  u8    bHeatChangeMoto:1;

}ForbidFunParamT;

typedef __packed struct 
{
  
  HmiSysParamT          SysSetting;
  ForbidFunParamT       ForbidFunSetting;
}HmiUserParamT;

typedef __packed struct 
{
  u16   Fan1OnTemp;
  u16   Fan1OffInter;
  u16   Fan2OnTemp;
  u16   Fan2OffInter;
  u16   Fan3OnTemp;
  u16   Fan3OffInter;
  u16   Fan4OnTemp;
  u16   Fan4OffInter;
  u16   Fan5OnTemp;
  u16   Fan5OffInter;
  u16   Fan6OnTemp;
  u16   Fan6OffInter;
  u16   Fan7OnTemp;
  u16   Fan7OffInter;
  
  u16   backup1;
  u16   backup2;
  
  u16   FanOnOffInter;
  u16   FanOnOffDelay;
}ConstSpeedFanParamT;

typedef __packed struct 
{
  u16   CurIP;
  u16   CurMask;
  u16   CurGateway;
  u16   CurPort;
  u16   DstIP;
  u16   DstPort;
  u16   TCPStatus;
}NetParamT;

typedef __packed struct 
{
  u16   IllumOnTime;
  u16   IllumOffTime;
  u16   backup1;
  u16   backup2;
  u16   backup3;
  u16   backup4;
  u16   backup5;
  
} IllumParamT;

typedef __packed struct 
{
  u16   SprayMoreOnTemp;
  u16   SprayMoreOffTemp;
  u16   SprayMoreOnInter;
  u16   SprayMoreOffInter;
  u16   SprayLessOnTemp;
  u16   SprayLessOffTemp;
  u16   SprayLessOnInter;
  u16   SprayLessOffInter;
  u16   Drink1Time;
  u16   Drink2Time;
  u16   Drink3Time;
  u16   Drink4Time;
  u16   Drink5Time;
  u16   DrinkDur;
  
  u16   backup1;
  u16   backup2;
  u16   backup3;
  u16   backup4;
  u16   backup5;
  u16   backup6;
    
}WaterCoolParamT;

typedef __packed struct 
{
  u16   AlarmUpTemp;
  u16   AlarmLowTemp;
  u16  MaxSensorTempDif;
  u16   TempNoPlus; //温度无波动检测 
  u16   TempMutateTempRange;
  u16   TempMutateTimeRange;
  
  u16   backup1;
  u16   backup2;
  
}AlarmParamT;
  
typedef __packed struct 
{
  u16   MasterMode;
  u16   CurRst;
  u16   CurNo;
  u16   SWVersion;
  u16   RstTimes;
  u16   FarmName;
  u16   ParamFacRst;
  u16   LCDComp;
  u16   TempManuAdj1;
  u16   TempManuAdj2;
  u16   TempManuAdj3;
  u16   SWRst;
  u16   ArrayIDList;
  u16   ArrayCtrlNum;
  u16   AutoSerial;
  u16   VarSpeedOnOffInter;
  u16   UnitFrontTempHumidID;
  u16   UnitBackTempHumid;
  u16   CO2TempHumidID;
  u16   AirSensorID;
  
  u16   backup1;
  u16   backup2;
  u16   backup3;
  u16   backup4;
  u16   backup5;
  u16   backup6;
  
}ServeParamT;


typedef __packed struct 
{
  u16   CtrlMode;
  u16   TargetTemp;
  u16   RoomPigNum;
  u16   PigDay;
  u16   RoomType;
  u16   VarVolFan1Brand;
  u16   VarVolFan2Brand;
  u16   VarVolFan3Brand;
  u16   VarVolFan4Brand;
  
  u16   backup1;
  u16   backup2;
  u16   backup3;
  u16   backup4;
  
  u16   FC1Type;
  u16   FC2Type;
  u16   FC3Type;
  
  u16   backup5;
  u16   backup6;
  u16   backup7;
  u16   backup8;
  u16   backup9;
  
  u16   MinHeatExgVent;//热交换最小通风量
  u16   MinTrenchVent;//地沟最小通风量
  u16   MinBGVent;//板地最小通风量
  
  u16   backup10;
  
}CtrlModeParamT;

typedef __packed struct 
{
  u16   SeasonPoleOpen;
  u16   SusCellPoleOpen;
  u16   HolePoleOpen;
  u16   HeatExgPoleOpen;
  u16   PoleAdj;
  
}PoleCtrlParamT;

typedef __packed struct 
{
  u16   MinTrenchTemp;
  u16   TrenchOffTemp;
  u16   TrenchOnTemp;
  u16   MaxTrenchVentTemp;
  u16   MaxTrenchVent;
  u16   MinHeatExgVentTemp;
  u16   HeatExgOffTemp;
  u16   HeatExgOnTemp;
  u16   MaxHeatExgVentTemp;
  u16   MaxHeatExgVent;
  u16   ModVolFan1Type;
  u16   ModVolFan2Type;
  u16   ModVolFan3Type;
  u16   ModVolFan4Type;
  
  u16   backup1;
  u16   backup2;
  u16   backup3;
  u16   backup4;
  
}ModVoltFanParamT;

typedef __packed struct 
{
  u16   Feed1Time1;
  u16   Feed1Time2;
  u16   Feed1Time3;
  u16   Feed1Time4;
  u16   Feed1Time5;
  
  u16   Feed2Time1;
  u16   Feed2Time2;
  u16   Feed2Time3;
  u16   Feed2Time4;
  u16   Feed2Time5;
  
  u16   FeedDur;
  u16   Feed1TdyDur;
  u16   Feed2TdyDur;
  u16   Feed1YdyDur;
  u16   Feed2YdyDur;

}FeedParamT;

typedef __packed struct 
{
  u16   FC1Adr;
  u16   FC2Adr;
  u16   FC3Adr;
  
  u16   backup1;
  u16   backup2;
  u16   backup3;
  u16   backup4;
  u16   backup5;
  
  u16   FC1Type;
  u16   FC2Type;
  u16   FC3Type;
  u16   FC4Type;
  u16   FC5Type;
  u16   FC6Type;
  u16   FC7Type;
  
  u16   backup6;
  
  u16   FCOnTemp;
  u16   FCOffTemp;
  u16   MaxFCVentTemp;
  u16   MaxFCVent;
  u16   MinFCVent;
  u16   TrenchOnTemp;
  u16   TrenchOffTemp;
  u16   MaxBGVentTemp;
  u16   MinBGVentTemp;
  u16   MaxBGVent;
  u16   FC1Status;
  u16   FC2Status;
  u16   FC3Status;
  
  u16   backup7;
  u16   backup8;
  u16   backup9;
  u16   backup10;
  u16   backup11;
  
  u16   FC1Setting;
  u16   FC2Setting;
  u16   FC3Setting;
  
  u16   backup12;
  u16   backup13;
  u16   backup14;
  u16   backup15;
  u16   backup16;
  
}FCParamT;

typedef __packed struct 
{
  u16   WarmLightOnTemp;
  u16   WarmLightOffTemp;
  u16   ProtTemp;
  u16   MinHumid;
  u16   MaxHumid;
  u16   MaxCO2Conc;
  u16   MaxH2SConc;
  u16   MaxNH4Conc;
  u16   HumidTime;  //湿度稳定时间
}WarmLightParamT;

typedef __packed struct 
{
    ConstSpeedFanParamT		ConstSpeedFanSetting;
    NetParamT		        NetSettting;
    IllumParamT		        IllumSetting;
    WaterCoolParamT	        WaterCoolSetting;
    AlarmParamT		        AlarmSetting;
    ServeParamT	            ServeSetting;
}HmiDevParamPart1T;

typedef __packed struct 
{
    CtrlModeParamT	        CtrlModeSetting;
    PoleCtrlParamT		    PoleCtrlSetting;
    ModVoltFanParamT	    ModVoltFanSetting;
    FeedParamT		        FeedSetting;
}HmiDevParamPart2T;

typedef __packed struct 
{
    FCParamT                FCSetting;
    WarmLightParamT	        WarmLightSetting;
}HmiDevParamPart3T;

#if 0
typedef __packed struct 
{
      HmiDevParamPart1T    DevPart1Setting;
      HmiDevParamPart2T    DevPart2Setting;
      HmiDevParamPart3T    DevPart3Setting;

} HmiDevParamPartT;
#endif 

#define MOD_ID_HMI		0x0001
#define MOD_ID_MODBUS	0x0002
#define MOD_ID_GATEWAY	0x0004

/*==============================used for Hmi Setting =========================*/
//extern void     InCtrlGetStatus(InCtrlRspT *pStatus);

extern bool     InCtrlHmiUserSetting(HmiUserParamT * pUserSetting);
extern bool     InCtrlHmiSysSetting(HmiSysParamT * pSysSetting);
//extern bool     InCtrlHmiSysExvParam(HmiSysExvParamT * pExvSetting);
//extern bool     InCtrlHmiSysFanParam(HmiSysFanParamT * pFanSetting);
//extern void     InCtrlGetStatus(InCtrlRspT *pStatus);
//extern void     InCtrlGetStatus(InCtrlRspT *pStatus);
//extern void     InCtrlGetStatus(InCtrlRspT *pStatus);
//extern void     InCtrlGetStatus(InCtrlRspT *pStatus);

/*==============================used for Hmi display=========================*/
extern void        InCtrlGetHmiStatus(HmiStatusT *pStatus);
extern bool		InCtrlSetUserParam(HmiUserParamT * pUserSetting, u8 ModId);/* ModId :1 used for dirty,0 for not dirty */
extern bool		InCtrlSetSysParam(HmiSysParamT * pSysSetting, u8 ModId);
//extern bool 	    InCtrlSetHumidParam(HmiHumidParamT * pHumidSetting, u8 ModId);
extern bool		InCtrlSyncUserParam(HmiUserParamT * pUserSetting, u8 ModId);
extern bool		InCtrlSyncSysParam(HmiSysParamT * pSysSetting, u8 ModId);
//extern bool 	    InCtrlSyncHumidParam(HmiHumidParamT * pHumidSetting, u8 ModId);


extern bool 	    InCtrlGetOdcCmd(u8 idx, OutCtrlCmdT *pCmd);
extern bool 	    InCtrlSetOdcStatus(u8 idx, OutCtrlRspT * pOutRsp);
extern void 	    InCtrlTask( void *pvParameters );
extern bool 	    InCtrlWriteFirmware(u32 Offset, u8 Len, u8 * pCode);

#endif /* __INDOOR_CTRL_H__ */

