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
**
** Functions
** ---------
**
** Initialization
**
**	User_Init_First ()
**	User_PrintUsage ()
**	User_ScanCmdLine ()
**
**	User_AppLogFilter ()
**
**	User_Init ()
**	User_Register ()
**	User_DeclQuants ()
**
**	User_Param_Add ()
**	User_Param_Get ()
**
**
** Main TestRun Start/End:
**
**	User_TestRun_Start_atBegin ()
**	User_TestRun_Start_atEnd ()
**	User_TestRun_Start_StaticCond_Calc ()
**	User_TestRun_Start_Finalize ()
**	User_TestRun_RampUp ()
**
**	User_TestRun_End_First ()
**	User_TestRun_End ()
**
**
** Main Cycle:
**
**	User_In ()
**
**	User_DrivMan_Calc ()
** 	User_Traffic_Calc ()
**	User_VehicleControl_Calc ()
**	User_Brake_Calc ()           in Vhcl_Calc ()
**	User_Calc ()
**	User_Check_IsIdle ()
**
**	User_Out ()
**
**
** APO Communication:
**
**	User_ApoMsg_Eval ()
**	User_ApoMsg_Send ()
**
**	User_ShutDown ()
**	User_End ()
**	User_Cleanup ()
**
**
******************************************************************************
*/

#include <Global.h>

#if defined(WIN32)
# include <windows.h>
#endif

#include <stdlib.h>
#include <string.h>
#include <math.h>

#if defined(XENO)
# include <mio.h>
#endif

#include <CarMaker.h>
#include <Car/Vehicle_Car.h>

#include <ADASRP.h>

#include <rbs.h>

#include "IOVec.h"
#include "User.h"

#include <zmq.h>
#include <czmq.h>
#include <stdio.h>
#include <unistd.h>
#include <assert.h>

#include <cJSON.h>
#include <cJSON2.h>

#include <Vehicle/Sensor_Line.h>
#include <Vehicle/Sensor_Inertial.h>

/* @@PLUGIN-BEGIN-INCLUDE@@ - Automatically generated code - don't edit! */
/* @@PLUGIN-END@@ */

#define DIM(x) (sizeof(x)/sizeof((x)[0]))

const char* Send_State (int action);


int UserCalcCalledByAppTestRunCalc = 0;
unsigned int counter = 1;
int drvr_counter;
int ovrwrt_drvr = true;
struct tRL_Agent {
    int On;
    double gas;
    double steer;
    int Episodes;
    char **uaq_txt;
};
struct tRL_Agent RL_Agent = {
    .On = true
};

char ipc_adress[] = "ipc:///tmp/99999";

double LS_F;

tUser	User;

void
RoadBorderDist (tRoadEval *re1, tRoadEval *re2, double *in, double deg)
{

    int max_loops = 360;

    double MyVhclPos_Cos;
    double MyVhclPos_Sin;
    double Res;
    double alpha;
    tRoadGeoIn gIn;
    tRoadGeoOut gOut;

    tRoadGeoIn gIn2;
    tRoadGeoOut gOut2;

    double LSMarkerPos_Add = 3;

    alpha = deg*deg2rad;

    Res = LSMarkerPos_Add/100;


    M_SINCOS(Vehicle.Yaw + alpha, &MyVhclPos_Sin, &MyVhclPos_Cos);

    gIn.xyz[0]= Vehicle.PoI_Pos[0] + LSMarkerPos_Add * MyVhclPos_Cos;
    gIn.xyz[1]= Vehicle.PoI_Pos[1] + LSMarkerPos_Add * MyVhclPos_Sin;
    gIn.xyz[2]= Vehicle.PoI_Pos[2];

    gIn2.xyz[0]= Vehicle.PoI_Pos[0] + (LSMarkerPos_Add + Res) * MyVhclPos_Cos;
    gIn2.xyz[1]= Vehicle.PoI_Pos[1] + (LSMarkerPos_Add + Res) * MyVhclPos_Sin;
    gIn2.xyz[2]= Vehicle.PoI_Pos[2];

    RoadGeoEval (re1, NULL, &gIn, &gOut);
    RoadGeoEval (re2, NULL, &gIn2, &gOut2);

    int loops = 0;
    while (gOut.onRoad != 1 && loops <= max_loops) {
        loops++;
        Res = LSMarkerPos_Add/100;
        LSMarkerPos_Add -= Res*0.99;
        gIn.xyz[0]= Vehicle.PoI_Pos[0] + LSMarkerPos_Add * MyVhclPos_Cos;
        gIn.xyz[1]= Vehicle.PoI_Pos[1] + LSMarkerPos_Add * MyVhclPos_Sin;
        RoadGeoEval (re1, NULL, &gIn, &gOut);
    }
    while (gOut2.onRoad == 1 && loops <= max_loops) {
        loops++;
        Res = LSMarkerPos_Add/100;
        LSMarkerPos_Add += Res*0.99;
        gIn2.xyz[0]= Vehicle.PoI_Pos[0] + (LSMarkerPos_Add + Res) * MyVhclPos_Cos;
        gIn2.xyz[1]= Vehicle.PoI_Pos[1] + (LSMarkerPos_Add + Res) * MyVhclPos_Sin;
        RoadGeoEval (re2, NULL, &gIn2, &gOut2);
    }
    
    *in = LSMarkerPos_Add;
}


static void
CarMVPreFunc_ResetPos (tCarMVPreIF *IF)
{
    tResetPos *rstps = &User.ResetPos;
    static char FreezeOnce;

    /* low level: nothing to do */
    if (rstps->Order.now == 0 && rstps->Order.old == 0) {
	
	if (FreezeOnce != 0)
	    FreezeOnce = 0;
	return;

    /* rising edge: saving current position */
    } else if (rstps->Order.now == 1 && rstps->Order.old == 0) {
	
	/* In order no repositioning during static equilibrium calculation */
	if (SimCore.State < SCState_StartLastCycle) {
	    FreezeOnce = 1;
	    return;
	}

	//Log ("Step0@CycleNo %d: Saving freeze position\n", SimCore.CycleNo);
	// VEC_Assign(rstps->Freeze.Pos,Car.ConBdy1.t_0);
	VEC_Assign(rstps->Freeze.Pos,Car.Fr1.t_0);
	rstps->Freeze.Ang[0] = Car.Roll;
	rstps->Freeze.Ang[1] = Car.Pitch;
	rstps->Freeze.Ang[2] = Car.Yaw;

    /* high level: holding current position */
    } else if (rstps->Order.now == 1 && rstps->Order.old == 1) {
	
	/* In order no repositioning during static equilibrium calculation */
	if (SimCore.State < SCState_StartLastCycle)
	    return;

	//if (FreezeOnce == 0)
	    //Log ("Setting speeds to zero\n");
	/* -> velocities */
	VEC_Assign(IF->v_0,Null3x1);
	VEC_Assign(IF->rv_zyx,Null3x1);
	
	/*if (FreezeOnce == 0) {
	    //Log ("Step1@CycleNo %d: Freezing at pos: %0.3fm %0.3fm %0.3fm - %0.3fdeg %0.3fdeg %0.3fdeg\n",
		SimCore.CycleNo,
		rstps->Freeze.Pos[0], 
		rstps->Freeze.Pos[1],
		rstps->Freeze.Pos[2],
		rstps->Freeze.Ang[0]*rad2deg,
		rstps->Freeze.Ang[1]*rad2deg,
		rstps->Freeze.Ang[2]*rad2deg);
	    //Log ("Desire to move to pos: %0.3fm %0.3fm %0.3fm - %0.3fdeg %0.3fdeg %0.3fdeg\n",
		rstps->New.Pos[0],
		rstps->New.Pos[1],
		rstps->New.Pos[2],
		rstps->New.Ang[0]*rad2deg,
		rstps->New.Ang[1]*rad2deg,
		rstps->New.Ang[2]*rad2deg);
	}*/
	
	/* -> positions */
	VEC_Assign(IF->t_0,rstps->Freeze.Pos);
	VEC_Assign(IF->r_zyx,rstps->Freeze.Ang);
	if (FreezeOnce == 0)
	    FreezeOnce = 1;

    /* falling edge: moving to indicated position (input) */
    } else if (rstps->Order.now == 0 && rstps->Order.old == 1) {
	
	/* In order no repositioning during static equilibrium calculation */
	if (SimCore.State < SCState_StartLastCycle)
	    return;
	
	Log ("Starting Episode %d at %.2fs.\n", RL_Agent.Episodes+1, SimCore.Time);
    ovrwrt_drvr = false;

	/* -> velocities */
    double speed_0[3];
    //double speed_fac = (double)rand()/(double)(RAND_MAX/30);
	//Log ("Setting speed to %.2f m/s\n", speed_fac);
    VEC_Mul(speed_0, EX3x1, 0);
	VEC_Assign(IF->v_0,speed_0);
	VEC_Assign(IF->rv_zyx,Null3x1);

	/*Log ("Step2@CycleNo %d: Switching to pos: %0.3fm %0.3fm %0.3fm - %0.3fdeg %0.3fdeg %0.3fdeg\n",
	    SimCore.CycleNo,
	    rstps->New.Pos[0],
	    rstps->New.Pos[1],
	    rstps->New.Pos[2],
	    rstps->New.Ang[0]*rad2deg,
	    rstps->New.Ang[1]*rad2deg,
	    rstps->New.Ang[2]*rad2deg);
    */
	/* -> positions */
	VEC_Assign(IF->t_0,rstps->New.Pos);
	VEC_Assign(IF->r_zyx,rstps->New.Ang);
    }
    
}


void
Calc_sRoad_Distance ()
{
    if (counter <= 2) {
        sRoad_StartPos = Vehicle.sRoad;
    }

    if (Vehicle.sRoad < 1 && sRoad_Distance > 0 && !LapOrder) {
        LapNo++;
        LapOrder = 1;
    }

    if (Vehicle.sRoad > 1) {
        LapOrder = 0;
    }
    double sRoad_Distance_old = sRoad_Distance;
    sRoad_Distance = Vehicle.sRoad + LapNo * Env.Route.Length - sRoad_StartPos;

    if (!CheckRoadBorderDist()) {
        reward_factor = 1/(1 + fabs(Car.ConBdy1.SideSlipAngle)*4);
        reward = (sRoad_Distance - sRoad_Distance_old) * 10 * reward_factor;
    }
    else {
        reward_factor = 1;
        reward =  -Vehicle.v * Vehicle.v *0.01;
    }
}

void
Reset_sRoad_Distance () {
    sRoad_Distance = 0;
    sRoad_StartPos = 0;
    LapNo = 0;
}

/*
** User_Init_First ()
**
** First, low level initialization of the User module
**
** Call:
** - one times at start of program
** - no realtime conditions
**
*/

int
User_Init_First (void)
{
    memset (&User, 0, sizeof(User));

    return 0;
}



/*
** User_PrintUsage ()
**
** Print the user/application specific programm arguments
*/

void
User_PrintUsage (const char *Pgm)
{
    /* REMARK: 1 log statement for each usage line, no line breaks */
    LogUsage("\n");
    LogUsage("Usage: %s [options] [testrun]\n", Pgm);
    LogUsage("Options:\n");

#if defined(CM_HIL)
    {
	const tIOConfig *cf;
	const char *defio = IO_GetDefault();
	LogUsage(" -io %-12s Default I/O configuration (%s)\n", "default",
	    (defio!=NULL && strcmp(defio, "none")!=0) ? defio : "minimal I/O");
	for (cf=IO_GetConfigurations(); cf->Name!=NULL; cf++)
	    LogUsage(" -io %-12s %s\n", cf->Name, cf->Description);
    }
#endif
}



/*
** User_ScanCmdLine ()
**
** Scan application specific command line arguments
**
** Return:
** - argv: last unscanned argument
** - NULL: error or unknown argument
*/

char **
User_ScanCmdLine (int argc, char **argv)
{
    const char *Pgm = argv[0];

    /* I/O configuration to be used in case no configuration was
       specified on the command line. */
    IO_SelectDefault("default" /* or "demoapp", "demorbs,demofr" etc. */);

    while (*++argv) {
	if (strcmp(*argv, "-io") == 0 && argv[1] != NULL) {
	    if (IO_Select(*++argv) != 0)
		return NULL;
	} else if (strcmp(*argv, "-h") == 0 || strcmp(*argv, "-help") == 0) {
	    User_PrintUsage(Pgm);
	    SimCore_PrintUsage(Pgm); /* Possible exit(), depending on CM-platform! */
	    return  NULL;
    } else if ( strcmp(*argv, "-servern") == 0 ) {
	    int server_n = atoi(*++argv);
        Log("Server: %d\n", server_n);
        sprintf(ipc_adress, "ipc:///tmp/%d", server_n);
        Log("%s\n", ipc_adress);
	    return  NULL;
	} else if ((*argv)[0] == '-') {
	    LogErrF(EC_General, "Unknown option '%s'", *argv);
	    return NULL;
	} else {
	    break;
	}
    }

    return argv;
}



/*
** User_Init ()
**
** Basic initialization of the module User.o
**
** Call:
** - once at program start
** - no realtime conditions
*/

int
User_Init (void)
{
    #ifdef BEAM_FCT
        Set_UserCarMVPreFunc (&CarMVPreFunc_ResetPos);
    #endif /* BEAM_FCT */


    return 0;
}



int
User_Register (void)
{

    /* @@PLUGIN-BEGIN-REGISTER@@ - Automatically generated code - don't edit! */
    /* @@PLUGIN-END@@ */

    return 0;
}



/*
** User_DeclQuants ()
**
** Add user specific quantities to the dictionary
**
** Call:
** - once at program start
** - no realtime conditions
*/

void
User_DeclQuants (void)
{
    int i;

    #ifdef BEAM_FCT
        tDDictEntry *de;
        tDDefault *df = DDefaultCreate ("User.ResetPos.Order.");
        tResetPos *rstps = &User.ResetPos;

        de=DDefChar(df, "DVA",	"",	&(rstps->Order.DVA), DVA_IO_In); DDefStates (de, 2, 0);
        de=DDefChar(df, "now",	"",	&(rstps->Order.now), DVA_None); DDefStates (de, 2, 0);
        de=DDefChar(df, "old",	"",	&(rstps->Order.old), DVA_None); DDefStates (de, 2, 0);
        
        DDefPrefix(df, "User.ResetPos.New.");
        DDefDouble4(df, "Pos_x",	 "m",	&(rstps->New.Pos[0]), DVA_IO_In);
        DDefDouble4(df, "Pos_y",	 "m",	&(rstps->New.Pos[1]), DVA_IO_In);
        DDefDouble4(df, "Pos_z",	 "m",	&(rstps->New.Pos[2]), DVA_IO_In);
        DDefDouble4(df, "PosO_x",	 "m",	&(rstps->New.PosOffset[0]), DVA_IO_In);
        DDefDouble4(df, "PosO_y",	 "m",	&(rstps->New.PosOffset[1]), DVA_IO_In);
        DDefDouble4(df, "PosO_z",	 "m",	&(rstps->New.PosOffset[2]), DVA_IO_In);
        DDefDouble4(df, "Ang_x",	 "rad",	&(rstps->New.Ang[0]), DVA_IO_In);
        DDefDouble4(df, "Ang_y",	 "rad",	&(rstps->New.Ang[1]), DVA_IO_In);
        DDefDouble4(df, "Ang_z",	 "rad",	&(rstps->New.Ang[2]), DVA_IO_In);
        
        DDefPrefix(df, "User.ResetPos.Freeze.");
        DDefDouble4(df, "Pos_x",	 "m",	&(rstps->Freeze.Pos[0]), DVA_None);
        DDefDouble4(df, "Pos_y",	 "m",	&(rstps->Freeze.Pos[1]), DVA_None);
        DDefDouble4(df, "Pos_z",	 "m",	&(rstps->Freeze.Pos[2]), DVA_None);
        DDefDouble4(df, "Ang_x",	 "rad",	&(rstps->Freeze.Ang[0]), DVA_None);
        DDefDouble4(df, "Ang_y",	 "rad",	&(rstps->Freeze.Ang[1]), DVA_None);
        DDefDouble4(df, "Ang_z",	 "rad",	&(rstps->Freeze.Ang[2]), DVA_None);
        
        DDefaultDelete (df);
    #endif /* BEAM_FCT */

    DDefInt(NULL , "RL_Agent.On" , "-", &(RL_Agent.On) , DVA_IO_In);
    DDefInt(NULL , "RL_Agent.Episodes" , "-", &(RL_Agent.Episodes) , DVA_IO_In);
    DDefDouble4(NULL , "RL_Agent.Reward" , "-", &(reward), DVA_None);
    DDefDouble4(NULL , "RL_Agent.Reward_Factor" , "-", &(reward_factor), DVA_None);
    DDefDouble4(NULL , "RL_Agent.Steer" , "-", &(RL_Agent.steer), DVA_None);
    DDefDouble4(NULL , "RL_Agent.Gas" , "-", &(RL_Agent.gas), DVA_None);

    DDefDouble4(NULL , "UserTest.RoadBorderDist.0" , "m", &(LSMarkerPos_Add_00), DVA_None);
    DDefDouble4(NULL , "UserTest.RoadBorderDist.L90" , "m", &(LSMarkerPos_Add_L90), DVA_None);
    DDefDouble4(NULL , "UserTest.RoadBorderDist.L60" , "m", &(LSMarkerPos_Add_L60), DVA_None);
    DDefDouble4(NULL , "UserTest.RoadBorderDist.L30" , "m", &(LSMarkerPos_Add_L30), DVA_None);
    DDefDouble4(NULL , "UserTest.RoadBorderDist.R90" , "m", &(LSMarkerPos_Add_R90), DVA_None);
    DDefDouble4(NULL , "UserTest.RoadBorderDist.R60" , "m", &(LSMarkerPos_Add_R60), DVA_None);
    DDefDouble4(NULL , "UserTest.RoadBorderDist.R30" , "m", &(LSMarkerPos_Add_R30), DVA_None);

    DDefDouble4(NULL , "UserTest.Road.DrivenDistance" , "m", &(sRoad_Distance), DVA_None);
    DDefDouble4(NULL , "UserTest.Road.sRoad_StartPos" , "m", &(sRoad_StartPos), DVA_None);
    DDefDouble4(NULL , "UserTest.Road.LapLength" , "m", &(Env.Route.Length), DVA_None);
    DDefInt(NULL , "UserTest.Road.LapNo" , "-", &(LapNo), DVA_None);

    for (i=0; i<N_USEROUTPUT; i++) {
	char sbuf[32];
	sprintf (sbuf, "UserOut_%02d", i);
	DDefDouble (NULL, sbuf, "", &User.Out[i], DVA_IO_Out);
    }
#if !defined(LABCAR)
    RBS_DeclQuants();
#endif
}


/*
** User_Param_Add ()
**
** Update all modified application specific parameters in the test stand
** parameter file (ECUParameters).
**
** If the variable SimCore.TestRig.ECUParam.Modified set to 1 somewhere else
** CarMaker calls this function to let the user add or change all necessary
** entries before the file is written.
** So, if writing the ECUParam file is necessary, set ECUParam.Modified to 1.
** The next TestRun start or end, CarMaker calls this function and writes
** the file to the harddisk.
**
** Call:
** - in a separate thread (no realtime contitions)
** - when starting a new test run
*/

int
User_Param_Add (void)
{
#if defined(CM_HIL)
    /* ECU parameters */
    if (SimCore.TestRig.ECUParam.Inf == NULL)
	return -1;
#endif

    return 0;
}



/*
** User_Param_Get ()
**
** Update all modified application specific parameters from the test stand
** parameter file (ECUParameters).
**
** Call:
** - in a separate thread (no realtime conditions)
** - if User_Param_Get() wasn't called
** - when starting a new test run, if
**   - the files SimParameters and/or
**   - ECUParameters
**   are modified since last reading
**
** return values:
**  0	ok
** -1	no testrig parameter file
** -2	testrig parameter error
** -3	i/o configuration specific error
** -4	no simulation parameters
** -5	simulation parameters error
** -6	FailSafeTester parameter/init error
*/

int
User_Param_Get (void)
{
    int rv = 0;

#if defined(CM_HIL)
    /*** testrig / ECU parameters */
    if (SimCore.TestRig.ECUParam.Inf == NULL)
	return -1;

    if (IO_Param_Get(SimCore.TestRig.ECUParam.Inf) != 0)
	rv = -2;
#endif

    /*** simulation parameters */
    if (SimCore.TestRig.SimParam.Inf == NULL)
	return -4;

    return rv;
}



/*
** User_TestRun_Start_atBegin ()
**
** Special things before a new simulation starts like
** - reset user variables to their default values
** - reset counters
** - ...
**
** Call:
** - in separate thread (no realtime conditions)
** - when starting a new test run
** - after (standard) infofiles are read in
** - before reading parameters for Environment, DrivMan, Car, ...
**   the models are NOT in the simulation-can-start-now state
**   (after Start(), before StaticCond())
*/

int
User_TestRun_Start_atBegin (void)
{
    int rv = 0;
    int i;

    for (i=0; i<N_USEROUTPUT; i++)
	User.Out[i] = 0.0;


    if (IO_None)
	return rv;

#if defined(CM_HIL)
    if (FST_New(SimCore.TestRig.ECUParam.Inf) != 0)
	rv = -6;
#endif

    return rv;
}




/*
** User_TestRun_Start_atEnd ()
**
** Special things before a new simulation starts like
** - reset user variables to there default values
** - reset counters
** - ...
**
** Call:
** - in separate thread (no realtime conditions)
** - when starting a new test run
** - at the end, behind reading parameters for Environment, DrivMan,
**   Car, ...
**   the models are NOT in the simulation-can-start-now state
**   (after Start(), before StaticCond())
*/

int
User_TestRun_Start_atEnd (void)
{
    CheckPath_at_s = RoadNewRoadEval (Env.Road, ROAD_BUMP_ALL, ROAD_OT_EXT, NULL);
    RoadEvalSetRouteByObjId (CheckPath_at_s, Env.Route.ObjId, 1);

    OnRoadSens_RE_00_1 = RoadNewRoadEval (Env.Road, ROAD_BUMP_NONE, ROAD_OT_MIN, NULL);
    OnRoadSens_RE_00_2 = RoadNewRoadEval (Env.Road, ROAD_BUMP_NONE, ROAD_OT_MIN, NULL);


    OnRoadSens_RE_L90_1 = RoadNewRoadEval (Env.Road, ROAD_BUMP_NONE, ROAD_OT_MIN, NULL);
    OnRoadSens_RE_L90_2 = RoadNewRoadEval (Env.Road, ROAD_BUMP_NONE, ROAD_OT_MIN, NULL);
    OnRoadSens_RE_L60_1 = RoadNewRoadEval (Env.Road, ROAD_BUMP_NONE, ROAD_OT_MIN, NULL);
    OnRoadSens_RE_L60_2 = RoadNewRoadEval (Env.Road, ROAD_BUMP_NONE, ROAD_OT_MIN, NULL);
    OnRoadSens_RE_L30_1 = RoadNewRoadEval (Env.Road, ROAD_BUMP_NONE, ROAD_OT_MIN, NULL);
    OnRoadSens_RE_L30_2 = RoadNewRoadEval (Env.Road, ROAD_BUMP_NONE, ROAD_OT_MIN, NULL);

    OnRoadSens_RE_R90_1 = RoadNewRoadEval (Env.Road, ROAD_BUMP_NONE, ROAD_OT_MIN, NULL);
    OnRoadSens_RE_R90_2 = RoadNewRoadEval (Env.Road, ROAD_BUMP_NONE, ROAD_OT_MIN, NULL);
    OnRoadSens_RE_R60_1 = RoadNewRoadEval (Env.Road, ROAD_BUMP_NONE, ROAD_OT_MIN, NULL);
    OnRoadSens_RE_R60_2 = RoadNewRoadEval (Env.Road, ROAD_BUMP_NONE, ROAD_OT_MIN, NULL);
    OnRoadSens_RE_R30_1 = RoadNewRoadEval (Env.Road, ROAD_BUMP_NONE, ROAD_OT_MIN, NULL);
    OnRoadSens_RE_R30_2 = RoadNewRoadEval (Env.Road, ROAD_BUMP_NONE, ROAD_OT_MIN, NULL);

    LSMarkerPos_Add_00 = 2.;
    LSMarkerPos_Add_L90 = 2.;
    LSMarkerPos_Add_L60 = 2.;
    LSMarkerPos_Add_L30 = 2.;
    LSMarkerPos_Add_R90 = 2.;
    LSMarkerPos_Add_R60 = 2.;
    LSMarkerPos_Add_R30 = 2.;
    #ifdef BEAM_FCT
        tInfos *inf = SimCore.Vhcl.Inf;
        double tmpvec[3];
        int i, nRows;
        char key[255];

        tResetPos *repos =	&User.ResetPos;

        /*** Repositioning ***/
        User.SensorID = InertialSensor_FindIndexForName("BeamRef");

        /** Search for x-Offset of Sensor of UserPgm
        *  UserPgm Sensor lays (in this example) on middle
        *  of rear axle on the ground (Y = Z = 0) **/
        if (User.SensorID < 0) {
        LogWarnF(EC_General,"Vhcl_repos: Can't identify body sensor 'BeamRef' in vehicle dataset!\n=> Car.* signal values used (small offset regarding to Sensor.Inertial.BeamRef.* signal values)");
        } else {

        VEC_Assign(tmpvec,Null3x1);

        /* -> RefPt difference UserPgm/CarMaker (middle rear axle on ground) */
        sprintf (key, "Sensor.Inertial.%d.pos", User.SensorID);
        i = iGetTable (inf, key, tmpvec, 3, 1, &nRows);
        if (i < 0) {
            LogWarnF (EC_General, "Vhcl_repos: Can't read body sensor 'BeamRef' position!\n=> Try to replace it with rear left wheel carrier position...");

            i = iGetTable (inf, "WheelCarrier.rl.pos", tmpvec, 3, 1, &nRows);
            if (i < 0) {
            LogErrF (EC_General, "Vhcl_repos: Can't read rear left wheel carrier position!\n");
            } else {
            repos->X_whlcrr = tmpvec[0];
            Log ("Vhcl_repos PreProc: WheelCarrier.rl.pos = %.3fm\n", repos->X_whlcrr);
            }

        } else {
            repos->X_whlcrr = tmpvec[0];
            Log ("Vhcl_repos PreProc: Sensor.Inertial.%d.pos.x = %.3fm\n", User.SensorID, repos->X_whlcrr);
        }
        }
    #endif /* BEAM_FCT */

    get_rlddict_selection ();
    uaq_json = cJSON_CreateArray();

    return 0;
}



/*
** User_TestRun_Start_StaticCond_Calc ()
**
** called in non RT context
*/

int
User_TestRun_Start_StaticCond_Calc (void)
{
    return 0;
}



/*
** User_TestRun_Start_Finalize ()
**
** called in RT context
*/

int
User_TestRun_Start_Finalize (void)
{

    return 0;
}



/*
** User_TestRun_RampUp ()
**
** Perform a smooth transition of variables (e.g. I/O)
** from their current state  to the new testrun.
** This function is called repeatedly, once during each cycle, until
** it returns true (or issues an error message), so the function should
** return true if transitioning is done, false otherwise.
**
** In case of an error the function should issue an apropriate
** error message and return false;
**
** Called in RT context, in state SCState_StartSim,
** after preprocessing is done, before starting the engine.
** Please note, that in this early initialization state no calculation
** of the vehicle model takes place.
*/

int
User_TestRun_RampUp (double dt)
{
    int IsReady = 1;

    return IsReady;
}



/*
** User_TestRun_End_First ()
**
** Invoked immediately after the end of a simulation is initiated,
** but before data storage ends and before transitioning into SCState_Idle.
** - Send Scratchpad-note
** - ...
**
** Call:
** - in main task, in the main loop (real-time conditions!)
** - when a test run is finished (SimCore.State is SCState_End)
*/

int
User_TestRun_End_First (void)
{
    #ifdef BEAM_FCT
        tResetPos *repos =		&User.ResetPos;

        /** Beam funtion **/
        repos->Order.DVA = 0;
        repos->Order.now = 0;
        repos->Order.old = 0;
    #endif

    ddict2json ();
    cJSON_Delete(uaq_json);
    free(RL_Agent.uaq_txt);
    return 0;
}

void
ddict2json (void)
{
    char fn[255];
    int ep = RL_Agent.Episodes + 1;
    char *string = cJSON_Print(uaq_json);
    if (string == NULL)
    {
        LogErrStr(EC_General, "Failed to print monitor.\n");
    }
    FILE * fPtr;
    sprintf(fn, "SimOutput/rl_uaq_store/ep_%d.json", ep);
    fPtr = fopen(fn, "w");
    if(fPtr == NULL)
    {
        /* File not created hence exit */
        LogErrStr(EC_General, "Unable to create log-file.\n");
    }
    fputs(string, fPtr);
    cJSON_free(string);
    fclose(fPtr);
    cJSON_Delete(uaq_json);
    uaq_json = cJSON_CreateArray();
}

void
get_rlddict_selection (void)
{
    tInfos *rl = InfoNew();
    tErrorMsg *err;

    int w = InfoRead(&err, rl, "Data/Config/RL_Data");
    if (w < 0) 
        LogWarnStr(EC_General, "Could not read info file for rl output quants.\n");

    char **uaq_txt = iGetTxt(rl, "DStore.Quantities.normal");
    int i = 0;
    while (uaq_txt[i] != NULL)
        i++;
    RL_Agent.uaq_txt = malloc(i * sizeof * RL_Agent.uaq_txt);
    Log("\n\nRL Data UAQs:\n");
    int j;
    for(j = 0; j < i; j++)
    {
        RL_Agent.uaq_txt[j] = strdup(uaq_txt[j]);
        Log("   %s\n",RL_Agent.uaq_txt[j]);
    }
    RL_Agent.uaq_txt[i] = NULL;
}

void
Add2Json_Object (void)
{
    cJSON *cycle = cJSON_CreateObject();
    for (int i = 0; RL_Agent.uaq_txt[i] != NULL; i++)
    {
        tDDictEntry *dentry = DDictGetEntry(RL_Agent.uaq_txt[i]);
        double val = dentry->GetFunc(dentry->Var);
        cJSON_AddNumberToObject(cycle, RL_Agent.uaq_txt[i], val);
    }
    cJSON_AddItemToArray(uaq_json, cycle);
}


/*
** User_TestRun_End ()
**
** Special things after the end of a simulation like
** - switch off an air compressor
** - Write something to a file
** - ...
**
** Call:
** - in separate thread (no realtime conditions)
** - when a test run is finished (SimCore.State is SCState_End<xyz>)
*/

int
User_TestRun_End (void)
{
    RoadDeleteRoadEval(CheckPath_at_s);

    RoadDeleteRoadEval(OnRoadSens_RE_00_1);
    RoadDeleteRoadEval(OnRoadSens_RE_00_2);
    RoadDeleteRoadEval(OnRoadSens_RE_L90_1);
    RoadDeleteRoadEval(OnRoadSens_RE_L90_2);
    RoadDeleteRoadEval(OnRoadSens_RE_L60_1);
    RoadDeleteRoadEval(OnRoadSens_RE_L60_2);
    RoadDeleteRoadEval(OnRoadSens_RE_L30_1);
    RoadDeleteRoadEval(OnRoadSens_RE_L30_2);
    RoadDeleteRoadEval(OnRoadSens_RE_R90_1);
    RoadDeleteRoadEval(OnRoadSens_RE_R90_2);
    RoadDeleteRoadEval(OnRoadSens_RE_R60_1);
    RoadDeleteRoadEval(OnRoadSens_RE_R60_2);
    RoadDeleteRoadEval(OnRoadSens_RE_R30_1);
    RoadDeleteRoadEval(OnRoadSens_RE_R30_2);

    Reset_sRoad_Distance ();

    return 0;
}



/*
** User_In ()
**
** Assign quantities of the i/o vector to model variables
**
** Call:
** - in the main loop
** - pay attention to realtime condition
** - just after IO_In()
*/

void
User_In (const unsigned CycleNo)
{
    #ifdef BEAM_FCT
        tResetPos *repos =	&User.ResetPos;
    #endif

    if (SimCore.State != SCState_Simulate)
    return;

    #ifdef BEAM_FCT
        /* control flag */
        repos->Order.old = repos->Order.now;
        repos->Order.now = repos->Order.DVA;
        /* - Flag set, when Init button pressed in UserPgm
        *   at that moment new valid init coordinates sent
        * - Flag reset, when Start button pressed in UserPgm */

        /* --- Initialisation phase --- */
        if (repos->Order.now == 1 && repos->Order.old == 0) {
            /*** Vehicle repositioning ***/
            /* input new position to reach: x-y-rz, rx-ry, z */
            /*** vec(O001) = vec(O0S) - vec(O1S)
             *** -> vec(O1S) = [rot(Yaw, vec(Z1))]^-1 * t[rot(Pitch, vec(Y2))]^-1 * repos->X_whlcrr * vec(X2) */
            repos->New.PosOffset[0] = repos->X_whlcrr * cos(repos->New.Ang[1]) * cos(repos->New.Ang[2]);
            repos->New.PosOffset[1] = repos->X_whlcrr * cos(repos->New.Ang[1]) * sin(repos->New.Ang[2]);
            repos->New.PosOffset[2] = repos->X_whlcrr *-sin(repos->New.Ang[1]);

            VEC_SubS(repos->New.Pos,repos->New.PosOffset);
        }
    #endif /* BEAM_FCT */
}



/*
** User_DrivMan_Calc ()
**
** called
** - in RT context
** - after DrivMan_Calc()
*/

int
User_DrivMan_Calc (double dt)
{
    /* Rely on the Vehicle Operator within DrivMan module to get
       the vehicle in driving state using the IPG's
       PowerTrain Control model 'Generic' or similar */
    if (Vehicle.OperationState != OperState_Driving)
	    return 0;

    return 0;
}

void
ReposVhcl (void)
{
    int w;
    w = DVA_WriteRequest("VC.Lights.IndL", OWMode_Abs, 5000, 0, 0, 2, NULL);
    if (w<0)
        Log("No DVA write to VC.Lights.IndL possible\n");

    // Reset pos via DVA to Start Pos:
    tRoadRouteIn CheckST;
    CheckST.st[0] = (double)rand()/(double)(RAND_MAX/Env.Route.Length);;
    CheckST.st[1] = 0;
    CheckST.st[0] = 5;

    tRoadRouteOutExt Out_CheckST;
    w = RoadRouteEvalExt (CheckPath_at_s, NULL, RIT_ST, &CheckST, &Out_CheckST);
    if (w<0)
        Log("RoadRouteEval Error #%d\n",w);

    double rx, ry, rz;
    NEV2FreiZYX (&Out_CheckST.suv[0], &Out_CheckST.suv[1], &Out_CheckST.suv[2], &rx,&ry, &rz);

    tResetPos *rstps = &User.ResetPos;
    rstps->New.Pos[0] = Out_CheckST.xyz[0];
    rstps->New.Pos[1] = Out_CheckST.xyz[1];
    rstps->New.Pos[2] = Out_CheckST.xyz[2];
    rstps->New.Ang[0] = Out_CheckST.nuv[0];
    rstps->New.Ang[1] = Out_CheckST.nuv[1];
    rstps->New.Ang[2] = rz;
    rstps->Order.DVA = 1;
}

/*
** Send_State (int action)
**
** called
** - in User_VehicleControl_Calc
** 
** action:
** 0 = send state (default)
** 1 = give IPG Driver control
** 2 = Restart TestRun
*/
const char* 
Send_State (int action)
{
    float State;
    char out_msg[4095] = {'\0'};
    char *loc = out_msg;
    size_t out_msg_BufferSpace = 4096;
    size_t tempLen;

    switch(action) {
        case 0: State = SimCore.Time; break;
        case 1: 
            State = 4.04; 
            ddict2json ();
            ReposVhcl ();
            break;
        case 2: State = 3.03; break;
        default: State = SimCore.Time; break;
    }

    /* Connect to server */
    zsock_t *requester = zsock_new_pair(ipc_adress);

    double LongSlip =   (Vehicle.FR.LongSlip + Vehicle.FL.LongSlip + \
                            Vehicle.RR.LongSlip + Vehicle.RL.LongSlip)/4;

    /* Define message to server */
    double state_array[] = { 
                            State,
                            sRoad_Distance,
                            Vehicle.v/42.,
                            Car.ConBdy1.SideSlipAngle/0.075,
                            InertialSensor[0].Acc_0[0]/1.,
                            InertialSensor[0].Acc_0[1]/-3.4,
                            Car.YawRate/-0.07,
                            Steering.IF.Ang/0.06,
                            Steering.IF.AngVel,
                            Steering.IF.AngAcc/500,
                            Steering.IF.TrqStatic/-1.76,
                            LongSlip/0.0094,
                            Car.FARoadSensor.Route.Deviation.Ang/0.00027,
                            LSMarkerPos_Add_00 / 84,
                            LSMarkerPos_Add_L90 / 9.6,
                            LSMarkerPos_Add_L60 / 11,
                            LSMarkerPos_Add_L30 / 19,
                            LSMarkerPos_Add_R90 / 9.6,
                            LSMarkerPos_Add_R60 / 11,
                            LSMarkerPos_Add_R30 / 19,
                            RoadSensor[0].Route.CurveXY /-0.004,
                            //RoadSensor[1].Route.CurveXY * 40,
                            //RoadSensor[2].Route.CurveXY * 40,
                            RoadSensor[3].Route.CurveXY /-0.004,
                            //RoadSensor[4].Route.CurveXY * 40,
                            //RoadSensor[5].Route.CurveXY * 40,
                            RoadSensor[6].Route.CurveXY /-0.004,
                            //RoadSensor[7].Route.CurveXY * 40,
                            //RoadSensor[8].Route.CurveXY * 40,
                            RoadSensor[9].Route.CurveXY /-0.004,
                            RoadSensor[10].Route.CurveXY /-0.004
    };

    int i;
    for(i = 0; i < DIM(state_array); ++i)
    {
        snprintf(loc, out_msg_BufferSpace, "%.16lf ", state_array[i]);
        tempLen = strlen(loc);
        loc += tempLen;
    }

    /* Send messsage to server */
    //printf("Sending: ");
    zstr_send(requester, out_msg);
    //printf("%s\n", out_msg);

    /* Recieve messsage from server */
    
    zsock_set_rcvtimeo(requester, 250);
    //printf("Incoming Message: ");
    char *buf = zstr_recv(requester);

    if (buf == NULL) {
        // printf("No msg\n");
        zstr_free(&buf);
        // printf("Free\n");
        zsock_destroy(&requester);
        // printf("Destroyed\n");
        char *buf1 = "0 0";
        // printf("buf1 created\n");
        return buf1;
    }
    else {
        // printf("%s\n", buf);
        zsock_destroy(&requester);
        return buf;
    }    
}

/*
** Update_RoadSensorPrevDist ()
**
** called
** - in RT context
** - in User_VehicleControl_Calc()
** 
** Changes PreviewDist of RoadSensor 0-9
** according to Vehicle.v
*/
void
Update_RoadSensorPrevDist () 
{
    RoadSensor[0].PreviewDist = 1 + Vehicle.v * 1.0;
    RoadSensor[1].PreviewDist = 1 + Vehicle.v * 1.2;
    RoadSensor[2].PreviewDist = 1 + Vehicle.v * 1.4;
    RoadSensor[3].PreviewDist = 1 + Vehicle.v * 1.6;
    RoadSensor[4].PreviewDist = 1 + Vehicle.v * 1.8;
    RoadSensor[5].PreviewDist = 1 + Vehicle.v * 2.0;
    RoadSensor[6].PreviewDist = 1 + Vehicle.v * 2.2;
    RoadSensor[7].PreviewDist = 1 + Vehicle.v * 2.4;
    RoadSensor[8].PreviewDist = 1 + Vehicle.v * 2.6;
    RoadSensor[9].PreviewDist = 1 + Vehicle.v * 2.8;
    RoadSensor[10].PreviewDist = 1 + Vehicle.v * 3.4;
}

int
CheckRoadBorderDist ()
{
    double MinDist = 1.5;
    int res;

    if (LSMarkerPos_Add_00 < 2) res = -1;
    else if (LSMarkerPos_Add_L30 < MinDist) res =  -1;
    else if (LSMarkerPos_Add_L60 < MinDist) res =  -1;
    else if (LSMarkerPos_Add_L90 < MinDist) res =  -1;
    else if (LSMarkerPos_Add_R30 < MinDist) res =  -1;
    else if (LSMarkerPos_Add_R60 < MinDist) res =  -1;
    else if (LSMarkerPos_Add_R90 < MinDist) res =  -1;
    else res =  0;

    return res;
}

/*
** User_VehicleControl_Calc ()
**
** called
** - in RT context
** - after VehicleControl_Calc()
*/
int
User_VehicleControl_Calc (double dt)
{
    /* Rely on the Vehicle Operator within DrivMan module to get
       the vehicle in driving state using the IPG's
       PowerTrain Control model 'Generic' or similar */
    if (Vehicle.OperationState != OperState_Driving)
	    return 0;

    if (SimCore.State != SCState_Simulate)
	    return 0;

    Update_RoadSensorPrevDist ();
    int evry_n = 125;
    int w;

    if(counter % 25 == 0) {
        if (ovrwrt_drvr && SimCore.Time > 1) {
            if (CheckRoadBorderDist() == -1 || fabs(Car.FARoadSensor.Route.Deviation.Ang) > 1.) {
                //printf("Porting car 1\n");

                ovrwrt_drvr = false;
                Send_State(1);
            }
        }
        else if (!ovrwrt_drvr) {
            w = DVA_WriteRequest("User.ResetPos.Order.DVA", OWMode_Abs, 0, 0, 0, 0, NULL);
                if (w<0)
                    Log("No DVA write to User.ResetPos.Order.DVA possible\n");

            ovrwrt_drvr = true;
            Reset_sRoad_Distance();
            return 0;
        }
    }

    if (!RL_Agent.On) {
        return 0;
    }

    /*  Send UAQs to RL-server and request new control input
        Run only every n cycle for performance */
    if(counter % evry_n == 0 && ovrwrt_drvr) {
        int r;

        
        double user_gas;
        double user_brake;
        double user_steer_acc_roh;

        char* pEnd;
        user_gas = strtod (Send_State(0), &pEnd);
        user_steer_acc_roh = strtod (pEnd, &pEnd);
        user_steer_acc = user_steer_acc_roh;

        RL_Agent.gas = user_gas;
        RL_Agent.steer = user_steer_acc_roh;

        if (user_gas > 4 && user_steer_acc_roh > 4) {
            ovrwrt_drvr = false;
            Send_State(1);
            return 0;
        }


        // printf("Values: %f %f\n", user_gas, user_steer_acc);

        /* Overwrite vehicle control */

        if (user_gas >= 0) {
            r = DVA_WriteRequest("DM.Gas", OWMode_Abs, evry_n, 0, 0, user_gas, NULL);
            if (r<0)
                Log("No DVA write to DM.Gas possible\n");
            r = DVA_WriteRequest("DM.Brake", OWMode_Abs, evry_n, 0, 0, 0, NULL);
            if (r<0)
                Log("No DVA write to DM.Brake possible\n");
        }
        else {
            user_brake = -user_gas;
            r = DVA_WriteRequest("DM.Brake", OWMode_Abs, evry_n, 0, 0, user_brake, NULL);
            if (r<0)
                Log("No DVA write to DM.Brake possible\n");
            r = DVA_WriteRequest("DM.Gas", OWMode_Abs, evry_n, 0, 0, 0, NULL);
            if (r<0)
                Log("No DVA write to DM.Gas possible\n");
        }
        
        Add2Json_Object ();
    }

    return 0;
}


/*
** User_Brake_Calc ()
**
** called
** - in RT context
** - after Brake_Calc() in Vhcl_Calc()
*/

int
User_Brake_Calc (double dt)
{
    /* Modify the total brake torque from the brake system model Brake.Trq_tot[]
       or the target drive source torque from the brake control unit
       Brake.HydBrakeCU_IF.Trq_DriveSrc_trg[]
    */

    return 0;
}



/*
** User_Traffic_Calc ()
**
** called
** - in RT context
** - after Traffic_Calc()
*/

int
User_Traffic_Calc (double dt)
{
    if (SimCore.State != SCState_Simulate)
	return 0;

    return 0;
}



/*
** User_Calc ()
**
** called in RT context
*/

int
User_Calc (double dt)
{
    if (SimCore.State != SCState_Simulate) return 0;

    if(counter % 25 == 0) {
        RoadBorderDist(OnRoadSens_RE_L90_1, OnRoadSens_RE_L90_2, &LSMarkerPos_Add_L90, 90.);
        RoadBorderDist(OnRoadSens_RE_L60_1, OnRoadSens_RE_L60_2, &LSMarkerPos_Add_L60, 60.);
        RoadBorderDist(OnRoadSens_RE_L30_1, OnRoadSens_RE_L30_2, &LSMarkerPos_Add_L30, 30.);
        RoadBorderDist(OnRoadSens_RE_R90_1, OnRoadSens_RE_R90_2, &LSMarkerPos_Add_R90, -90.);
        RoadBorderDist(OnRoadSens_RE_R60_1, OnRoadSens_RE_R60_2, &LSMarkerPos_Add_R60, -60.);
        RoadBorderDist(OnRoadSens_RE_R30_1, OnRoadSens_RE_R30_2, &LSMarkerPos_Add_R30, -30.);
        RoadBorderDist(OnRoadSens_RE_00_1, OnRoadSens_RE_00_2, &LSMarkerPos_Add_00, 0.);
    }

    Calc_sRoad_Distance();

    if (!RL_Agent.On) 
        return 0;
    
    int r;
    r = DVA_WriteRequest("DM.Steer.Trq", OWMode_Abs, 1, 0, 0, RL_Agent.steer, NULL);
    if (r<0)
        Log("No DVA write to DM.Steer.Trq possible\n");
    


    counter++;

    return 0;
}


/*
** User_Check_IsIdle ()
**
** Checking, if the simulation model is in idle conditions (stand still,
** steeringwheel angle zero, cluch pedal pressed, ...).
** If reached idle state, the calculation of vehicle model and driving
** manoevers is stopped.
** Ready for start new simulation.
**
** Return:
** 1  idle state reached
** 0  else
**
** Call:
** - in main task, in the main loop
** - pay attention to realtime condition
** - while SimCore.State==SCState_EndIdleGet
*/

int
User_Check_IsIdle (int IsIdle)
{
    double val;

    /*** ECU / carmodel signals */

    /* vehicle and wheels: stand still */
    val = 0.5*kmh2ms;
    if (Vehicle.v > val
     || fabs(Vehicle.Wheel[0]->vBelt) > val || fabs(Vehicle.Wheel[1]->vBelt) > val
     || fabs(Vehicle.Wheel[2]->vBelt) > val || fabs(Vehicle.Wheel[3]->vBelt) > val) {
	IsIdle = 0;
    }

    /* SteerAngle: drive  straight forward position */
    val = 1.0*deg2rad;
    if (Vehicle.Steering.Ang > val || Vehicle.Steering.Ang < -val)
	IsIdle = 0;

    return IsIdle;
}



/*
** User_Out ()
**
** Assigns model quantities to variables of the i/o vector
**
** call:
** - in the main loop
** - pay attention to realtime condition
** - just before IO_Out();
*/

void
User_Out (const unsigned CycleNo)
{
#if !defined(LABCAR)
    RBS_OutMap(CycleNo);
#endif

    if (SimCore.State != SCState_Simulate)
	return;
}



/*
** User_ApoMsg_Eval ()
**
** Communication between the application and connected GUIs.
** Evaluate messages from GUIs
**
** Call:
** - in the main loop
** - pay attention to realtime condition
** - near the end of the main loop, if the function SimCore_ApoMsg_Eval()
**    skips the message
**
** Return:
**   0 : message evaluated
**  -1 : message not handled
*/

int
User_ApoMsg_Eval (int Ch, char *Msg, int len, int who)
{
#if defined(CM_HIL)
    /*** FailSafeTester */
    if (Ch == ApoCh_CarMaker) {
	if (FST_ApoMsgEval(Ch, Msg, len) <= 0)
	    return 0;
    }

#endif
    return -1;
}


/*
** User_ApoMsg_Send ()
**
** Communication between the application and connected GUIs.
** Sends messages to GUIs
**
** Call:
** - near the end of the main loop, in MainThread_FinishCycle()
** - pay attention to realtime condition
*/

void
User_ApoMsg_Send (double T, const unsigned CycleNo)
{
}



/*
** User_ShutDown ()
**
** Prepare application for shut down
**
** Call:
** - at end of program
** - no realtime conditions
*/

int
User_ShutDown (int ShutDownForced)
{
    int IsDown = 0;

    /* Prepare application for shutdown and return that
       shutdown conditions are reached */
    if (1) {
	IsDown = 1;
    }

    return IsDown;
}



/*
** User_End ()
**
** End all models of the user module
**
** Call:
** - one times at end of program
** - no realtime conditions
*/

int
User_End (void)
{
    return 0;
}



/*
** User_Cleanup ()
**
** Cleanup function of the User module
**
** Call:
** - one times at end of program, just before exit
** - no realtime conditions
*/

void
User_Cleanup (void)
{

}
