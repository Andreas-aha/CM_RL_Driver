/*
******************************************************************************
**  CarMaker - Version 9.0.2
**  Vehicle Dynamics Simulation Toolkit
**
**  Copyright (C)   IPG Automotive GmbH
**                  Bannwaldallee 60             Phone  +49.721.98520.0
**                  76185 Karlsruhe              Fax    +49.721.98520.99
**                  Germany                      WWW    www.ipg-automotive.com
******************************************************************************
*/

#ifndef _USER_H__
#define _USER_H__

#include <Global.h>
#include <Vehicle/MBSUtils.h>

#ifdef __cplusplus
extern "C" {
#endif



extern int UserCalcCalledByAppTestRunCalc;


#define N_USEROUTPUT	10

void RoadBorderDist (tRoadEval*, tRoadEval*, double *, double);
void Calc_sRoad_Distance (void);
void Reset_sRoad_Distance (void);
int CheckRoadBorderDist (void);

tRoadEval *CheckPath_at_s;

tRoadEval *OnRoadSens_RE_00_1;
tRoadEval *OnRoadSens_RE_00_2;

tRoadEval *OnRoadSens_RE_L90_1;
tRoadEval *OnRoadSens_RE_L90_2;

tRoadEval *OnRoadSens_RE_L60_1;
tRoadEval *OnRoadSens_RE_L60_2;

tRoadEval *OnRoadSens_RE_L30_1;
tRoadEval *OnRoadSens_RE_L30_2;


tRoadEval *OnRoadSens_RE_R90_1;
tRoadEval *OnRoadSens_RE_R90_2;

tRoadEval *OnRoadSens_RE_R60_1;
tRoadEval *OnRoadSens_RE_R60_2;

tRoadEval *OnRoadSens_RE_R30_1;
tRoadEval *OnRoadSens_RE_R30_2;

double LSMarkerPos_Add_00;

double LSMarkerPos_Add_L90;
double LSMarkerPos_Add_L60;
double LSMarkerPos_Add_L30;

double LSMarkerPos_Add_R90;
double LSMarkerPos_Add_R60;
double LSMarkerPos_Add_R30;

double sRoad_Distance;
double sRoad_StartPos;
double LapLength;
int LapOrder;
int LapNo;

double reward;
double reward_factor;

double user_steer_acc;

/* -> Repositioning */
typedef struct {
    struct {
	char DVA;
	char now;
	char old;
    } Order;
    struct {
	double Pos[3], Ang[3];
    } Freeze;
    struct {
	double Pos[3], PosOffset[3], Ang[3];
    } New;
    /* Offset, X-Lage of rear wheel carrier,
     * an offset exists due to pitch. */
    double X_whlcrr;	/* diff Refpt UserPgm/CarMaker */
    //double RefPos[3];	/* diff pt ConBdy/Refpt */
} tResetPos;


/* Struct for user variables. */
typedef struct tUser {
    /* For debugging purposes */
    double Out[N_USEROUTPUT];

    /* Ego repositioning by reset */
    tResetPos	ResetPos;
    int		SensorID;

} tUser;

extern tUser User;

void Update_RoadSensorPrevDist (void);

int 	User_Init_First		(void);
int 	User_Init		(void);
void	User_PrintUsage		(const char *Pgm);
char  **User_ScanCmdLine	(int argc, char **argv);
int 	User_Start		(void);
int	User_Register		(void);
void	User_DeclQuants		(void);
int 	User_ShutDown		(int ShutDownForced);
int 	User_End		(void);
void 	User_Cleanup		(void);

int	User_TestRun_Start_atBegin		(void);
int	User_TestRun_Start_atEnd		(void);
int	User_TestRun_Start_StaticCond_Calc	(void);
int	User_TestRun_Start_Finalize		(void);
int	User_TestRun_RampUp			(double dt);
int	User_DrivMan_Calc			(double dt);
int	User_VehicleControl_Calc		(double dt);
int	User_Brake_Calc				(double dt);
int	User_Traffic_Calc			(double dt);
int	User_Calc				(double dt);
int	User_Check_IsIdle			(int IsIdle);
int	User_TestRun_End_First 			(void);
int	User_TestRun_End 			(void);

void 	User_In  (const unsigned CycleNo);
void	User_Out (const unsigned CycleNo);


/* User_<> functions,
** - called from SimCore and in CM_Main.c,
** - already defined in SimCore.h
*/
int 	User_Param_Get		(void);
int 	User_Param_Add		(void);
int 	User_ApoMsg_Eval (int channel, char *msg, int len, int who);
void 	User_ApoMsg_Send (double T, const unsigned CycleNo);


#define User_TestRun_Start   User_TestRun_Start__deprecated_function__Change_to__User_TestRun_Start_XYZ;


#ifdef __cplusplus
}
#endif

#endif	/* #ifndef _USER_H__ */
