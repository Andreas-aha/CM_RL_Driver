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

#ifdef BEAM_FCT
#include <Vehicle/Sensor_Inertial.h>
#endif

/* @@PLUGIN-BEGIN-INCLUDE@@ - Automatically generated code - don't edit! */
/* @@PLUGIN-END@@ */

#define DIM(x) (sizeof(x)/sizeof((x)[0]))

const char* Send_State (int action);

int UserCalcCalledByAppTestRunCalc = 0;
int counter = 1;
int drvr_counter;
int ovrwrt_drvr = true;

char ipc_adress[] = "ipc:///tmp/99999";

tUser	User;

#ifdef BEAM_FCT
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

	Log ("Step0@CycleNo %d: Saving freeze position\n", SimCore.CycleNo);
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

	if (FreezeOnce == 0)
	    Log ("Setting speeds to zero\n");
	/* -> velocities */
	VEC_Assign(IF->v_0,Null3x1);
	VEC_Assign(IF->rv_zyx,Null3x1);
	
	if (FreezeOnce == 0) {
	    Log ("Step1@CycleNo %d: Freezing at pos: %0.3fm %0.3fm %0.3fm - %0.3fdeg %0.3fdeg %0.3fdeg\n",
		SimCore.CycleNo,
		rstps->Freeze.Pos[0], 
		rstps->Freeze.Pos[1],
		rstps->Freeze.Pos[2],
		rstps->Freeze.Ang[0]*rad2deg,
		rstps->Freeze.Ang[1]*rad2deg,
		rstps->Freeze.Ang[2]*rad2deg);
	    Log ("Desire to move to pos: %0.3fm %0.3fm %0.3fm - %0.3fdeg %0.3fdeg %0.3fdeg\n",
		rstps->New.Pos[0],
		rstps->New.Pos[1],
		rstps->New.Pos[2],
		rstps->New.Ang[0]*rad2deg,
		rstps->New.Ang[1]*rad2deg,
		rstps->New.Ang[2]*rad2deg);
	}
	
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
	
	Log ("Starting new simulation in UserPgm\n");

	Log ("Setting speeds to zero\n");
	/* -> velocities */
	VEC_Assign(IF->v_0,Null3x1);
	VEC_Assign(IF->rv_zyx,Null3x1);
	
	Log ("Step2@CycleNo %d: Switching to pos: %0.3fm %0.3fm %0.3fm - %0.3fdeg %0.3fdeg %0.3fdeg\n",
	    SimCore.CycleNo,
	    rstps->New.Pos[0],
	    rstps->New.Pos[1],
	    rstps->New.Pos[2],
	    rstps->New.Ang[0]*rad2deg,
	    rstps->New.Ang[1]*rad2deg,
	    rstps->New.Ang[2]*rad2deg);
	/* -> positions */
	VEC_Assign(IF->t_0,rstps->New.Pos);
	VEC_Assign(IF->r_zyx,rstps->New.Ang);
    }
    
}
#endif /* BEAM_FCT */


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
        
    return 0;
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
    char out_msg[256] = {'\0'};
    char *loc = out_msg;
    size_t out_msg_BufferSpace = 256;
    size_t tempLen;

    switch(action) {
        case 0: State = SimCore.Time; break;
        case 1: State = 4.04; break;
        case 2: State = 3.03; break;
        default: State = SimCore.Time; break;
    }

    /* Connect to server */
    zsock_t *requester = zsock_new_pair(ipc_adress);

    float LongSlip =   (Vehicle.FR.LongSlip + Vehicle.FL.LongSlip + \
                            Vehicle.RR.LongSlip + Vehicle.RL.LongSlip)/4;

    /* Define message to server */
    float state_array[] = { \
                            State, \
                            Vehicle.v/60, \
                            Car.FARoadSensor.Path.Deviation.Dist / 10, \
                            Car.FARoadSensor.Path.Deviation.Ang / M_PI, \
                            RoadSensor[10].Path.LongSlope, \
                            RoadSensor[10].Route.CurveXY * 4, \
                            RoadSensor[10].Path.Deviation.Dist / 10, \
                            RoadSensor[10].Act.Width/10, \
                            RoadSensor[11].Route.CurveXY * 4, \
                            RoadSensor[11].Path.Deviation.Dist / 10, \
                            RoadSensor[11].Act.Width/10, \
                            RoadSensor[0].Path.Deviation.Dist / 10, \
                            RoadSensor[0].Route.CurveXY * 4, \
                            RoadSensor[0].Act.Width/10, \
                            RoadSensor[1].Route.CurveXY * 4, \
                            RoadSensor[1].Path.Deviation.Dist / 10, \
                            RoadSensor[1].Act.Width/10, \
                            RoadSensor[2].Route.CurveXY * 4, \
                            RoadSensor[2].Path.Deviation.Dist / 10, \
                            RoadSensor[2].Act.Width/10, \
                            RoadSensor[3].Route.CurveXY * 4, \
                            RoadSensor[3].Path.Deviation.Dist / 10, \
                            RoadSensor[3].Act.Width/10, \
                            RoadSensor[4].Route.CurveXY * 4, \
                            RoadSensor[5].Route.CurveXY * 4, \
                            RoadSensor[6].Route.CurveXY * 4, \
                            RoadSensor[7].Route.CurveXY * 4, \
                            RoadSensor[8].Route.CurveXY * 4, \
                            RoadSensor[9].Route.CurveXY * 4, \
                            Steering.IF.Ang / (3.5 * M_PI), \
                            LongSlip / 4, \
                            Car.ConBdy1.SideSlipAngle / M_PI_2 \
    };



    int i;
    for(i = 0; i < DIM(state_array); ++i)
    {
        snprintf(loc, out_msg_BufferSpace, "%.4f ", state_array[i]);
        tempLen = strlen(loc);
        loc += tempLen;
    }

    /* Send messsage to server */
    //printf("Sending: ");
    zstr_send(requester, out_msg);
    //printf("%s\n", out_msg);

    /* Recieve messsage from server */
    
    zsock_set_rcvtimeo(requester, 1000);
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

    int evry_n;
    int w;

    if (ovrwrt_drvr && SimCore.Time > 1) {

        if (Car.FARoadSensor.Act.Width <= abs(Car.FARoadSensor.Path.Deviation.Dist)*2 || abs(Car.FARoadSensor.Path.Deviation.Ang) > 1) {

            if (ovrwrt_drvr) {
                // printf("Control given to IPG Driver because left road\n");

                ovrwrt_drvr = false;
                Send_State(1);

                w = DVA_WriteRequest("VC.Lights.IndL", OWMode_Abs, 5000, 0, 0, 2, NULL);
                if (w<0)
                    Log("No DVA write to VC.Lights.IndL possible\n");
            }

        }
    }

    if (!ovrwrt_drvr) {

        w = DVA_WriteRequest("Car.SlotCar.Deviation", OWMode_Abs, 500, 0, 0, Car.FARoadSensor.Act.Width/2, NULL);
            if (w<0)
                Log("No DVA write to Car.SlotCar.Deviation possible\n");

        w = DVA_WriteRequest("Car.SlotCar.State", OWMode_Abs, 500, 0, 0, true, NULL);
            if (w<0)
                Log("No DVA write to Car.SlotCar.Deviation possible\n");   

        // printf("Checking if car in good pos\n");
        if ((Car.FARoadSensor.Act.Width*0.2 >= abs(Car.FARoadSensor.Path.Deviation.Dist)*2) && (abs(Car.FARoadSensor.Path.Deviation.Ang) < 0.04)) {
          
            drvr_counter++;

            if (drvr_counter > 5000) {
                w = DVA_WriteRequest("DM.Gas", OWMode_Abs, 100, 0, 0, 0, NULL);
                if (w<0)
                    Log("No DVA write to DM.Gas possible\n");


            if (Vehicle.v <= 20) {
                drvr_counter = 0;
                ovrwrt_drvr = true;

                w = DVA_WriteRequest("VC.Lights.IndL", OWMode_Abs, 5000, 0, 0, 0, NULL);
                if (w<0)
                    Log("No DVA write to VC.Lights.IndL possible\n");
                
                // printf("RF Learning again\n");
            }
            }

            
        
        }
        
    }

    if (ovrwrt_drvr) {
        evry_n = 125;
    }

    /*  Send UAQs to RL-server and request new control input
        Run only every n cycle for performance */
    if(counter % evry_n == 0 && ovrwrt_drvr) {

        w = DVA_WriteRequest("Car.SlotCar.State", OWMode_Abs, evry_n, 0, 0, false, NULL);
            if (w<0)
                Log("No DVA write to Car.SlotCar.Deviation possible\n");  

        w = DVA_WriteRequest("Car.SlotCar.Deviation", OWMode_Abs, evry_n, 0, 0, -1, NULL);
            if (w<0)
                Log("No DVA write to Car.SlotCar.Deviation possible\n");

        int r;

        
        double user_gas;
        double user_brake;
        double user_steer;
 
        char* pEnd;
        user_gas = strtod (Send_State(0), &pEnd);
        user_steer = strtod (pEnd, &pEnd);

        if (user_gas > 4 && user_steer > 4) {
            ovrwrt_drvr = false;
            user_gas = 0;
            user_steer = 0;
        }


        // printf("Values: %f %f\n", user_gas, user_steer);

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

        double steerang;
        steerang = Steering.IF.Ang + user_steer;
        if (steerang > 9) {
            steerang = 9;
        }
        else if (steerang < -9)
        {
            steerang = -9;
        }
        

        r = DVA_WriteRequest("DM.Steer.Ang", OWMode_Abs, evry_n, 0, 0, steerang, NULL);
        if (r<0)
            Log("No DVA write to DM.Steer possible\n");
        
    }

    counter++;

    if (counter > 10000) {
        counter = 1;
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
    /* Starting with CM 6.0 User_Calc() will be invoked in EVERY simulation
       state. Uncomment the following line in order to restore the behaviour
       of CM 5.1 and earlier. */
    /*if (!UserCalcCalledByAppTestRunCalc) return 0;*/

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
