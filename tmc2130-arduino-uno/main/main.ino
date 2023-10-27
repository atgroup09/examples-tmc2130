/* @page TMC2130 example 1
 * @date 2023-10-25
 * @note MCU Arduino UNO, SDK Arduino 1.8.18
 *       Step motor driver BigTreeTech TMC2130 v3.0
 */

#include <TMC2130Stepper.h>
#include <arduino-timer.h>


/* @def GPIO map
*/
#define TMC2130_DIR_PIN   7   // DIR  (Direction)
#define TMC2130_STEP_PIN  6   // STP  (Step)
                              // NC
#define TMC2130_MISO_PIN  12  // MISO (SPI MISO - TMC.DataOut)
#define TMC2130_CS_PIN    10  // CS   (SPI Chip select)
#define TMC2130_SCK_PIN   13  //      (SPI SCK Software Clock)
#define TMC2130_MOSI_PIN  11  //      (SPI MOSI - TMC.DataIn)
#define TMC2130_EN_PIN    8   // EN   (Enable)


/* @def STEP pulse task types
*/
#define TMC2130_STEP_TASK_TIMER  1
#define TMC2130_STEP_TASK_LOOP   2

/* @def STEP pulse task type (set needed)
*/
#define TMC2130_STEP_TASK        TMC2130_STEP_TASK_TIMER


/* @def STEP pulse period (usec)
*/
#define TMC2130_STEP_PERIOD      100

/* @def Diagnostic info monitoring period (usec)
*/
#define TMC2130_DIAGMON_PERIOD   2000000

/* @def debug-log control bits
*/
#define DEBUG_LOG_OFF    0
#define DEBUG_LOG_PRINT  1


/* @var Timer-instance (2 task, usec tick)
*/
Timer<2, micros> TIMER1;

/* @var Timer TaskHandler IDs (pointer to task)
*/
void *TIMER1_DiagMon_TaskID;
void *TIMER1_StepGen_TaskID;

/* @var Timer TaskHandler states
*/
bool TIMER1_DiagMon_TaskState = false;
bool TIMER1_StepGen_TaskState = false;


/* @var TMC2130-instance (uninitialized)
*/
TMC2130Stepper TMC2130 = TMC2130Stepper(TMC2130_EN_PIN, TMC2130_DIR_PIN, TMC2130_STEP_PIN, TMC2130_CS_PIN);

/* @var TMC2130.STEP: current value
*/
bool TMC2130_Step = false;

/* @var TMC2130.SHAFT: current value (rotate direction)
*/
bool TMC2130_Dir = false;


/* @var UART Menu command
*/
String UART1_MenuCmd;
String UART1_MenuArg;


/** @brief  Driver on.
 *  @param  None.
 *  @return None.
 */
void TMC2130_DriverOn(void)
{
  digitalWrite(TMC2130_EN_PIN, LOW);
}

/** @brief  Driver off.
 *  @param  None.
 *  @return None.
 */
void TMC2130_DriverOff(void)
{
  digitalWrite(TMC2130_EN_PIN, HIGH);
}


/** @brief  DIR set.
 *  @param  None.
 *  @return None.
 */
void TMC2130_DirSet(void)
{
  TMC2130.shaft_dir(TMC2130_Dir);
}

/** @brief  DIR toggle.
 *  @param  LogIn - debug-log:
 *  @arg    = 0 - off
 *  @arg    = 1 - print
 *  @return None.
 */
void TMC2130_DirToggle(int LogIn)
{
  TMC2130_Dir = ((TMC2130_Dir) ? false : true);
  TMC2130_DirSet();
  
  if(LogIn)
  {
    Serial.print("direction [TOGGLE] (");
    Serial.print(TMC2130_Dir);
    Serial.print(")\r\n");
  }
}


/** @brief  STEP off.
 *  @param  None.
 *  @return None.
 */
void TMC2130_StepOff(void)
{
  digitalWrite(TMC2130_STEP_PIN, LOW);
  TMC2130_Step = false;
}

/** @brief  STEP on.
 *  @param  None.
 *  @return None.
 */
void TMC2130_StepOn(void)
{
  digitalWrite(TMC2130_STEP_PIN, HIGH);
  TMC2130_Step = true;
}

/** @brief  STEP toggle.
 *  @param  None.
 *  @return None.
 */
void TMC2130_StepToggle(void)
{
  if(!TMC2130_Step)
  {
    TMC2130_StepOn();
  }
  else
  {
    TMC2130_StepOff();
  }
}

/** @brief  Timer1 TaskHandler (step generator).
 *  @param  ArgIn - pointer to input arguments.
 *  @return True.
 */
bool TIMER1_StepGen_Task(void *)
{
  TMC2130_StepToggle();
  return (true); // repeat? true
}

/** @brief  Start task `StepGen`.
 *  @param  LogIn - debug-log:
 *  @arg    = 0 - off
 *  @arg    = 1 - print
 *  @return None.
 */
void TMC2130_StepGen_TaskStart(int LogIn)
{
  if(!TIMER1_StepGen_TaskState)
  {
#if (TMC2130_STEP_TASK == TMC2130_STEP_TASK_TIMER)
    TIMER1_StepGen_TaskID = TIMER1.every(TMC2130_STEP_PERIOD, TIMER1_StepGen_Task);
    TMC2130_StepOn();
#endif

    TIMER1_StepGen_TaskState = true;
    if(LogIn) Serial.print("step [START]\r\n");
  }
}

/** @brief  Stop task `StepGen`.
 *  @param  LogIn - debug-log:
 *  @arg    = 0 - off
 *  @arg    = 1 - print
 *  @return None.
 */
void TMC2130_StepGen_TaskStop(int LogIn)
{
  if(TIMER1_StepGen_TaskState)
  {
#if (TMC2130_STEP_TASK == TMC2130_STEP_TASK_TIMER)
    TIMER1.cancel(TIMER1_StepGen_TaskID);
    TMC2130_StepOff();
#endif

    TIMER1_StepGen_TaskState = false;
    if(LogIn) Serial.print("step [STOP]\r\n");
  }
}

/** @brief  Toggle task `StepGen`.
 *  @param  LogIn - debug-log:
 *  @arg    = 0 - off
 *  @arg    = 1 - print
 *  @return None.
 */
void TMC2130_StepGen_TaskToggle(int LogIn)
{
  if(!TIMER1_StepGen_TaskState)
  {
    TMC2130_StepGen_TaskStart(LogIn);
  }
  else
  {
    TMC2130_StepGen_TaskStop(LogIn);
  }
}


/** @brief  Print diagnostic information.
 *  @param  None.
 *  @return None.
 */
void TMC2130_DiagPrint(void)
{
  Serial.print("============\r\n");
  Serial.print("GCONF\r\n");
  Serial.print("SHAFT:");
  Serial.print(TMC2130.shaft(), DEC);
  Serial.print("\r\n");
  
  Serial.print("--------\r\n");
  Serial.print("GSTAT\r\n");
  Serial.print("RST:");
  Serial.print(TMC2130.reset(), DEC);
  Serial.print(" ERR:");
  Serial.print(TMC2130.drv_err(), DEC);
  Serial.print(" UV:");
  Serial.print(TMC2130.uv_cp(), DEC);
  Serial.print("\r\n");
  
  Serial.print("--------\r\n");
  Serial.print("IOIN\r\n");
  Serial.print("STEP:");
  Serial.print(TMC2130.step(), DEC);
  Serial.print(" DIR:");
  Serial.print(TMC2130.dir(), DEC);
  Serial.print(" CFG4:");
  Serial.print(TMC2130.dcen_cfg4(), DEC);
  Serial.print(" CFG5:");
  Serial.print(TMC2130.dcin_cfg5(), DEC);
  Serial.print(" CFG6:");
  Serial.print(TMC2130.drv_enn_cfg6(), DEC);
  Serial.print(" DCO:");
  Serial.print(TMC2130.dco(), DEC);
  Serial.print(" VER:");
  Serial.print(TMC2130.version(), DEC);
  Serial.print("\r\n");

  Serial.print("--------\r\n");
  Serial.print("LOST_STEPS:");
  Serial.println(TMC2130.LOST_STEPS(), DEC);

  Serial.print("--------\r\n");
  Serial.print("CUR.A:");
  Serial.println(TMC2130.cur_a(), DEC);
  Serial.print("--------\r\n");
  Serial.print("CUR.B:");
  Serial.println(TMC2130.cur_b(), DEC);
  Serial.print("--------\r\n");
  Serial.print("VDCMIN:");
  Serial.println(TMC2130.VDCMIN(), DEC);
  
  Serial.print("\r\n");
}

/** @brief  Timer1 TaskHandler (diagnostic info monitoring).
 *  @param  ArgIn - pointer to input arguments.
 *  @return True.
 */
bool TIMER1_DiagMon_Task(void *)
{
  TMC2130_DiagPrint();
  return (true); // repeat? true
}

/** @brief  Toggle task `DiagMon`.
 *  @param  LogIn - debug-log:
 *  @arg    = 0 - off
 *  @arg    = 1 - print
 *  @return None.
 */
void TMC2130_DiagMon_TaskToggle(int LogIn)
{
  TIMER1_DiagMon_TaskState = ((TIMER1_DiagMon_TaskState) ? false : true);
  if(LogIn) Serial.print("diagmon ");

  if(TIMER1_DiagMon_TaskState)
  {
    TIMER1_DiagMon_TaskID = TIMER1.every(TMC2130_DIAGMON_PERIOD, TIMER1_DiagMon_Task);
    if(LogIn) Serial.print("[START]\r\n");
  }
  else
  {
    TIMER1.cancel(TIMER1_DiagMon_TaskID);
    if(LogIn) Serial.print("[STOP]\r\n");
  }
}


/** @brief  Init. config.
 *  @param  LogIn - debug-log:
 *  @arg    = 0 - off
 *  @arg    = 1 - print
 *  @return None.
 */
void TMC2130_ConfigInit(int LogIn)
{
  TMC2130_StepGen_TaskStop(LogIn);

  TMC2130_DriverOff();
  delay(5);
  
  TMC2130.rms_current(200);
  TMC2130.stealthChop(1);
  TMC2130.VDCMIN(8);
  TMC2130_DirSet();

  TMC2130_DriverOn();
  delay(5);

  if(LogIn) Serial.print("config [INIT]\r\n");
}


/** @brief  Print Main Menu.
 *  @param  None.
 *  @return None.
 */
void UART1_MenuPrint(void)
{
  Serial.print("\r\n");
  Serial.print("== TMC2130 Example 1 ====\r\n");
  Serial.print("step    - start/stop step task\r\n");
  Serial.print("dir     - toggle direction\r\n");
  Serial.print("diag    - print diagnostic info\r\n");
  Serial.print("diagmon - start/stop diagnostic info task\r\n");
  Serial.print("conf    - stop step task and reinit config\r\n");
  Serial.print("menu    - print this menu\r\n");
  Serial.print("\r\n");
}


/** @brief  Pre-start setup.
 *  @param  None.
 *  @return None.
 */
void setup(void)
{
  //Init. UART (hardware)
  Serial.begin(9600);
  while(!Serial);

  //Init. SPI (hardware)
  SPI.begin();
  pinMode(MISO, INPUT_PULLUP);

  //Init. TMC2130-instance
  TMC2130.begin();
  TMC2130_ConfigInit(DEBUG_LOG_OFF);

  UART1_MenuPrint();
}


/** @brief  Main cycle.
 *  @param  None.
 *  @return None.
 */
void loop(void)
{
  TIMER1.tick();

#if (TMC2130_STEP_TASK == TMC2130_STEP_TASK_LOOP)
  if(TIMER1_StepGen_TaskState)
  {
    TMC2130_StepToggle();
    delayMicroseconds(TMC2130_STEP_PERIOD);
  }
#endif

  //Read Main Menu command
  if(Serial.available() > 0)
  {
    UART1_MenuCmd = Serial.readStringUntil('\n');

    if(UART1_MenuCmd == "step")
    {
      TMC2130_StepGen_TaskToggle(DEBUG_LOG_PRINT);
    }
    else if(UART1_MenuCmd == "dir")
    {
      TMC2130_DirToggle(DEBUG_LOG_PRINT);
    }
    else if(UART1_MenuCmd == "diag")
    {
      TMC2130_DiagPrint();
    }
    else if(UART1_MenuCmd == "diagmon")
    {
      TMC2130_DiagMon_TaskToggle(DEBUG_LOG_PRINT);
    }
    else if(UART1_MenuCmd == "conf")
    {
      TMC2130_ConfigInit(DEBUG_LOG_PRINT);
    }
    else
    {
      UART1_MenuPrint();
    }
  }
}
