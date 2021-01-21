//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ert_main.cpp
//
// Code generated for Simulink model 'Dc_motor_Dev'.
//
// Model version                  : 1.32
// Simulink Coder version         : 9.2 (R2019b) 18-Jul-2019
// C/C++ source code generated on : Thu Jan 21 16:22:31 2021
//
// Target selection: ert.tlc
// Embedded hardware selection: AMD->x86-64 (Linux 64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include <stdio.h>
#include <stdlib.h>
#include "Dc_motor_Dev.h"
#include "Dc_motor_Dev_private.h"
#include "rtwtypes.h"
#include "limits.h"
#include "rt_nonfinite.h"
#include "linuxinitialize.h"
#define UNUSED(x)                      x = x

static RT_MODEL_Dc_motor_Dev_T Dc_motor_Dev_M_;
static RT_MODEL_Dc_motor_Dev_T *const Dc_motor_Dev_MPtr = &Dc_motor_Dev_M_;// Real-time model 
static B_Dc_motor_Dev_T Dc_motor_Dev_B;// Observable signals
static X_Dc_motor_Dev_T Dc_motor_Dev_X;// Observable continuous states
static ExtU_Dc_motor_Dev_T Dc_motor_Dev_U;// External inputs
static ExtY_Dc_motor_Dev_T Dc_motor_Dev_Y;// External outputs

#define NAMELEN                        16

// Function prototype declaration
void exitFcn(int sig);
void *terminateTask(void *arg);
void *baseRateTask(void *arg);
void *subrateTask(void *arg);
volatile boolean_T stopRequested = false;
volatile boolean_T runModel = true;
sem_t stopSem;
sem_t baserateTaskSem;
pthread_t schedulerThread;
pthread_t baseRateThread;
void *threadJoinStatus;
int terminatingmodel = 0;
void *baseRateTask(void *arg)
{
  runModel = (rtmGetErrorStatus(Dc_motor_Dev_M) == (NULL)) &&
    !rtmGetStopRequested(Dc_motor_Dev_M);
  while (runModel) {
    sem_wait(&baserateTaskSem);
    Dc_motor_Dev_step(Dc_motor_Dev_M, &Dc_motor_Dev_U, &Dc_motor_Dev_Y);

    // Get model outputs here
    stopRequested = !((rtmGetErrorStatus(Dc_motor_Dev_M) == (NULL)) &&
                      !rtmGetStopRequested(Dc_motor_Dev_M));
    runModel = !stopRequested;
  }

  runModel = 0;
  terminateTask(arg);
  pthread_exit((void *)0);
  return NULL;
}

void exitFcn(int sig)
{
  UNUSED(sig);
  rtmSetErrorStatus(Dc_motor_Dev_M, "stopping the model");
}

void *terminateTask(void *arg)
{
  UNUSED(arg);
  terminatingmodel = 1;

  {
    runModel = 0;
  }

  // Disable rt_OneStep() here

  // Terminate model
  Dc_motor_Dev_terminate(Dc_motor_Dev_M);
  sem_post(&stopSem);
  return NULL;
}

int main(int argc, char **argv)
{
  UNUSED(argc);
  UNUSED(argv);
  void slros_node_init(int argc, char** argv);
  slros_node_init(argc, argv);
  rtmSetErrorStatus(Dc_motor_Dev_M, 0);

  // Pack model data into RTM
  Dc_motor_Dev_M->blockIO = &Dc_motor_Dev_B;
  Dc_motor_Dev_M->contStates = &Dc_motor_Dev_X;

  // Initialize model
  Dc_motor_Dev_initialize(Dc_motor_Dev_M, &Dc_motor_Dev_U, &Dc_motor_Dev_Y);

  // Call RTOS Initialization function
  myRTOSInit(0.01, 0);

  // Wait for stop semaphore
  sem_wait(&stopSem);

#if (MW_NUMBER_TIMER_DRIVEN_TASKS > 0)

  {
    int i;
    for (i=0; i < MW_NUMBER_TIMER_DRIVEN_TASKS; i++) {
      CHECK_STATUS(sem_destroy(&timerTaskSem[i]), 0, "sem_destroy");
    }
  }

#endif

  return 0;
}

//
// File trailer for generated code.
//
// [EOF]
//
