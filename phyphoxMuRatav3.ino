#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <SCH16T.h>
#include <phyphoxBle.h>

// ============================================================
// DEVICE
// ============================================================

const int MURATA_ID = 1;
char bleName[16];


// ============================================================
// ESP32 -> Murata SCH16T-K01-PCB
// ============================================================

const int MISO_PIN = 19;
const int CS_PIN   = 5;
const int SCK_PIN  = 18;
const int MOSI_PIN = 23;

const int RESET_PIN = -1;

SCH16T_K01 imu(SPI, CS_PIN, RESET_PIN);

SCH16T_filter Filter;
SCH16T_sensitivity Sensitivity;
SCH16T_decimation Decimation;

SCH16T_raw_data raw;
SCH16T_result result;

bool sensorOK = false;


// ============================================================
// EARTH ROTATION RATE
// ============================================================

// 360 degrees / sidereal day

const double EARTH_RATE_DPS =
  360.0 / 86164.0905;


// ============================================================
// SENSOR SAMPLING
// ============================================================

// Scientific acquisition = 50 Hz

const uint32_t SAMPLE_INTERVAL_US = 20000;


// Average 5 raw samples for live display:
//
// 50 Hz -> 10 Hz

const int DISPLAY_AVERAGE_COUNT = 5;

uint32_t nextSampleUs = 0;

float displaySumX = 0.0f;
float displaySumY = 0.0f;
float displaySumZ = 0.0f;

int displayN = 0;


// ============================================================
// ABBA TIMING
// ============================================================

const uint32_t SETTLE_TIME_MS  = 5000;

const uint32_t MEASURE_TIME_MS = 10000;

const uint32_t ROTATE_TIME_MS  = 10000;


// ============================================================
// ABBA SEQUENCE
//
// Y is the primary axis.
//
// 0  Delete data
// 1  Ready
//
// 2  Y+ North settling
// 3  Measure A1
//
// 4  Rotate slowly to Y- North
//
// 5  Y- North settling
// 6  Measure B1
// 7  Measure B2
//
// 8  Rotate slowly to Y+ North
//
// 9  Y+ North settling
// 10 Measure A2
//
// 11 Finished
// ============================================================

enum AbbaState
{
  WAIT_CLEAR      = 0,
  READY_TO_START  = 1,

  A1_SETTLE       = 2,
  A1_MEASURE      = 3,

  ROTATE_TO_B     = 4,

  B1_SETTLE       = 5,
  B1_MEASURE      = 6,
  B2_MEASURE      = 7,

  ROTATE_TO_A     = 8,

  A2_SETTLE       = 9,
  A2_MEASURE      = 10,

  ABBA_FINISHED   = 11
};


AbbaState abbaState =
  WAIT_CLEAR;


uint32_t stateStartMs = 0;


// ============================================================
// PHYPHOX EVENTS
// ============================================================

volatile bool startRequested = false;

volatile bool pauseRequested = false;

volatile bool clearRequested = false;


bool phyphoxRunning = false;

bool armed = false;


// ============================================================
// STATISTICS
// ============================================================

struct AxisStats
{
  uint32_t n;

  double mean;

  double M2;
};


struct BlockStats
{
  AxisStats x;

  AxisStats y;

  AxisStats z;
};


struct BlockResult
{
  bool valid;

  uint32_t n;

  double x;
  double y;
  double z;

  double sdX;
  double sdY;
  double sdZ;
};


BlockStats currentStats;


BlockResult resultA1;

BlockResult resultB1;

BlockResult resultB2;

BlockResult resultA2;


// ============================================================
// FINAL RESULTS
// ============================================================

bool resultsValid = false;

bool finalPacketSent = false;


// Signed Earth components

double earthY = NAN;

double earthX = NAN;


// Combined horizontal Earth rotation

double earthHorizontal = NAN;


// Full 180-degree reversal

double reversalStep = NAN;


// Repeatability estimates

double repeatabilityY = NAN;

double repeatabilityX = NAN;

double repeatabilityHorizontal = NAN;


// Alignment of Y axis from N-S

double alignmentFromYDeg = NAN;


// Latitude magnitude

double latitudeDeg = NAN;

double latitudeRepeatabilityDeg = NAN;


// ============================================================
// STATISTICS FUNCTIONS
// ============================================================

void resetAxisStats(
  AxisStats &s
)
{
  s.n = 0;

  s.mean = 0.0;

  s.M2 = 0.0;
}


// ------------------------------------------------------------

void resetBlockStats()
{
  resetAxisStats(
    currentStats.x
  );


  resetAxisStats(
    currentStats.y
  );


  resetAxisStats(
    currentStats.z
  );
}


// ------------------------------------------------------------

void addAxisSample(
  AxisStats &s,
  double value
)
{
  s.n++;


  double delta =
    value -
    s.mean;


  s.mean +=
    delta /
    (double)s.n;


  double delta2 =
    value -
    s.mean;


  s.M2 +=
    delta *
    delta2;
}


// ------------------------------------------------------------

void addBlockSample(
  double gx,
  double gy,
  double gz
)
{
  addAxisSample(
    currentStats.x,
    gx
  );


  addAxisSample(
    currentStats.y,
    gy
  );


  addAxisSample(
    currentStats.z,
    gz
  );
}


// ------------------------------------------------------------

double getSD(
  const AxisStats &s
)
{
  if (
    s.n < 2
  )
  {
    return NAN;
  }


  return sqrt(
    s.M2 /
    (double)(
      s.n - 1
    )
  );
}


// ------------------------------------------------------------

BlockResult finishBlock()
{
  BlockResult r;


  r.valid =
    currentStats.y.n >
    10;


  r.n =
    currentStats.y.n;


  r.x =
    currentStats.x.mean;


  r.y =
    currentStats.y.mean;


  r.z =
    currentStats.z.mean;


  r.sdX =
    getSD(
      currentStats.x
    );


  r.sdY =
    getSD(
      currentStats.y
    );


  r.sdZ =
    getSD(
      currentStats.z
    );


  return r;
}


// ============================================================
// CLEAR STORED RESULTS
// ============================================================

void clearStoredResults()
{
  resultA1.valid = false;

  resultB1.valid = false;

  resultB2.valid = false;

  resultA2.valid = false;


  resultsValid = false;

  finalPacketSent = false;


  earthY = NAN;

  earthX = NAN;


  earthHorizontal = NAN;


  reversalStep = NAN;


  repeatabilityY = NAN;

  repeatabilityX = NAN;

  repeatabilityHorizontal = NAN;


  alignmentFromYDeg = NAN;


  latitudeDeg = NAN;

  latitudeRepeatabilityDeg = NAN;


  resetBlockStats();
}


// ============================================================
// DELETE DATA -> ARM NEW ABBA
// ============================================================

void armNewABBA()
{
  clearStoredResults();


  armed =
    true;


  phyphoxRunning =
    false;


  abbaState =
    READY_TO_START;


  stateStartMs =
    millis();


  Serial.println();

  Serial.println(
    "========================"
  );

  Serial.println(
    "ABBA READY"
  );

  Serial.println(
    "Y+ NORTH"
  );

  Serial.println(
    "PRESS START"
  );

  Serial.println(
    "========================"
  );
}


// ============================================================
// ABORT RUN
// ============================================================

void abortABBA()
{
  clearStoredResults();


  armed =
    false;


  phyphoxRunning =
    false;


  abbaState =
    WAIT_CLEAR;


  Serial.println();

  Serial.println(
    "========================"
  );

  Serial.println(
    "RUN CANCELLED"
  );

  Serial.println(
    "DELETE DATA TO RESET"
  );

  Serial.println(
    "========================"
  );
}


// ============================================================
// ENTER STATE
// ============================================================

void enterState(
  AbbaState newState
)
{
  abbaState =
    newState;


  stateStartMs =
    millis();


  Serial.println();


  switch (
    abbaState
  )
  {

    // ========================================================
    // A1 SETTLE
    // ========================================================

    case A1_SETTLE:

      Serial.println(
        "Y+ NORTH"
      );

      Serial.println(
        "SETTLING 5 s"
      );

      break;


    // ========================================================
    // A1 MEASURE
    // ========================================================

    case A1_MEASURE:

      resetBlockStats();


      Serial.println(
        "MEASURING A1"
      );

      Serial.println(
        "10 s"
      );

      break;


    // ========================================================
    // ROTATE TO B
    // ========================================================

    case ROTATE_TO_B:

      Serial.println(
        "************************"
      );

      Serial.println(
        "ROTATE SLOWLY"
      );

      Serial.println(
        "Y- NORTH"
      );

      Serial.println(
        "************************"
      );

      break;


    // ========================================================
    // B SETTLE
    // ========================================================

    case B1_SETTLE:

      Serial.println(
        "Y- NORTH"
      );

      Serial.println(
        "SETTLING 5 s"
      );

      break;


    // ========================================================
    // B1 MEASURE
    // ========================================================

    case B1_MEASURE:

      resetBlockStats();


      Serial.println(
        "MEASURING B1"
      );

      Serial.println(
        "10 s"
      );

      break;


    // ========================================================
    // B2 MEASURE
    // ========================================================

    case B2_MEASURE:

      resetBlockStats();


      Serial.println(
        "MEASURING B2"
      );

      Serial.println(
        "10 s"
      );

      break;


    // ========================================================
    // ROTATE TO A
    // ========================================================

    case ROTATE_TO_A:

      Serial.println(
        "************************"
      );

      Serial.println(
        "ROTATE SLOWLY"
      );

      Serial.println(
        "Y+ NORTH"
      );

      Serial.println(
        "************************"
      );

      break;


    // ========================================================
    // A2 SETTLE
    // ========================================================

    case A2_SETTLE:

      Serial.println(
        "Y+ NORTH"
      );

      Serial.println(
        "SETTLING 5 s"
      );

      break;


    // ========================================================
    // A2 MEASURE
    // ========================================================

    case A2_MEASURE:

      resetBlockStats();


      Serial.println(
        "MEASURING A2"
      );

      Serial.println(
        "10 s"
      );

      break;


    // ========================================================
    // FINISHED
    // ========================================================

    case ABBA_FINISHED:

      Serial.println();

      Serial.println(
        "========================"
      );

      Serial.println(
        "FINISHED"
      );

      Serial.println(
        "PRESS PAUSE"
      );

      Serial.println(
        "SEE RESULTS"
      );

      Serial.println(
        "========================"
      );

      break;


    default:

      break;
  }
}


// ============================================================
// IS ABBA CURRENTLY MEASURING?
// ============================================================

bool abbaIsMeasuring()
{
  return (
    abbaState ==
      A1_MEASURE ||

    abbaState ==
      B1_MEASURE ||

    abbaState ==
      B2_MEASURE ||

    abbaState ==
      A2_MEASURE
  );
}


// ============================================================
// COUNTDOWN
// ============================================================

float getCountdown()
{
  uint32_t duration =
    0;


  switch (
    abbaState
  )
  {

    case A1_SETTLE:

    case B1_SETTLE:

    case A2_SETTLE:

      duration =
        SETTLE_TIME_MS;

      break;


    case A1_MEASURE:

    case B1_MEASURE:

    case B2_MEASURE:

    case A2_MEASURE:

      duration =
        MEASURE_TIME_MS;

      break;


    case ROTATE_TO_B:

    case ROTATE_TO_A:

      duration =
        ROTATE_TIME_MS;

      break;


    default:

      return 0.0f;
  }


  uint32_t elapsed =
    millis() -
    stateStartMs;


  if (
    elapsed >=
    duration
  )
  {
    return 0.0f;
  }


  uint32_t remaining =
    duration -
    elapsed;


  return
    (float)(
      (
        remaining +
        999
      )
      /
      1000
    );
}


// ============================================================
// CALCULATE ABBA
// ============================================================

void calculateABBA()
{
  if (
    !resultA1.valid ||
    !resultB1.valid ||
    !resultB2.valid ||
    !resultA2.valid
  )
  {
    resultsValid =
      false;


    Serial.println(
      "ERROR: INCOMPLETE ABBA"
    );


    return;
  }


  // ==========================================================
  // MEAN A AND B VALUES
  // ==========================================================

  double meanAY =
    (
      resultA1.y +
      resultA2.y
    )
    /
    2.0;


  double meanBY =
    (
      resultB1.y +
      resultB2.y
    )
    /
    2.0;


  double meanAX =
    (
      resultA1.x +
      resultA2.x
    )
    /
    2.0;


  double meanBX =
    (
      resultB1.x +
      resultB2.x
    )
    /
    2.0;


  // ==========================================================
  // SIGNED EARTH COMPONENTS
  //
  // Earth = (A - B) / 2
  // ==========================================================

  earthY =
    (
      meanAY -
      meanBY
    )
    /
    2.0;


  earthX =
    (
      meanAX -
      meanBX
    )
    /
    2.0;


  // ==========================================================
  // HORIZONTAL EARTH ROTATION
  //
  // Uses both X and Y.
  //
  // This means a small azimuth alignment error does not
  // significantly affect the horizontal magnitude.
  // ==========================================================

  earthHorizontal =
    sqrt(
      earthX * earthX +
      earthY * earthY
    );


  reversalStep =
    2.0 *
    earthHorizontal;


// ============================================================
// WITHIN-RUN REPEATABILITY
// ============================================================

  double dAY =
    resultA1.y -
    resultA2.y;


  double dBY =
    resultB1.y -
    resultB2.y;


  repeatabilityY =
    0.25 *
    sqrt(
      dAY * dAY +
      dBY * dBY
    );


  double dAX =
    resultA1.x -
    resultA2.x;


  double dBX =
    resultB1.x -
    resultB2.x;


  repeatabilityX =
    0.25 *
    sqrt(
      dAX * dAX +
      dBX * dBX
    );


// ============================================================
// PROPAGATE REPEATABILITY TO HORIZONTAL MAGNITUDE
// ============================================================

  if (
    earthHorizontal >
    1.0e-10
  )
  {
    double wx =
      earthX /
      earthHorizontal;


    double wy =
      earthY /
      earthHorizontal;


    repeatabilityHorizontal =
      sqrt(
        (
          wx *
          repeatabilityX
        )
        *
        (
          wx *
          repeatabilityX
        )
        +
        (
          wy *
          repeatabilityY
        )
        *
        (
          wy *
          repeatabilityY
        )
      );
  }

  else
  {
    repeatabilityHorizontal =
      NAN;
  }


// ============================================================
// Y-AXIS NORTH ALIGNMENT
//
// 0 degrees = Y is North-South
//
// 90 degrees = X is North-South
// ============================================================

  if (
    earthHorizontal >
    1.0e-10
  )
  {
    alignmentFromYDeg =
      atan2(
        fabs(
          earthX
        ),
        fabs(
          earthY
        )
      )
      *
      180.0 /
      PI;
  }

  else
  {
    alignmentFromYDeg =
      NAN;
  }


// ============================================================
// LATITUDE MAGNITUDE
//
// omega_horizontal = omega_Earth cos(latitude)
//
// latitude = acos(horizontal / Earth rate)
//
// Uses combined X/Y horizontal signal.
//
// Assumes the sensor is level.
// ============================================================

  double ratio =
    earthHorizontal /
    EARTH_RATE_DPS;


  if (
    ratio >= 0.0 &&
    ratio <= 1.0
  )
  {
    double phiRad =
      acos(
        ratio
      );


    latitudeDeg =
      phiRad *
      180.0 /
      PI;


    double sinPhi =
      sin(
        phiRad
      );


    if (
      fabs(
        sinPhi
      )
      >
      1.0e-6
      &&
      isfinite(
        repeatabilityHorizontal
      )
    )
    {
      latitudeRepeatabilityDeg =
        (
          repeatabilityHorizontal
          /
          (
            EARTH_RATE_DPS *
            fabs(
              sinPhi
            )
          )
        )
        *
        180.0 /
        PI;
    }

    else
    {
      latitudeRepeatabilityDeg =
        NAN;
    }
  }

  else
  {
    latitudeDeg =
      NAN;


    latitudeRepeatabilityDeg =
      NAN;
  }


  resultsValid =
    true;


// ============================================================
// SERIAL RESULTS
// ============================================================

  Serial.println();

  Serial.println(
    "========== ABBA RESULTS =========="
  );


  Serial.print(
    "A1 Y = "
  );

  Serial.print(
    resultA1.y,
    7
  );

  Serial.print(
    "  SD = "
  );

  Serial.println(
    resultA1.sdY,
    7
  );


  Serial.print(
    "B1 Y = "
  );

  Serial.print(
    resultB1.y,
    7
  );

  Serial.print(
    "  SD = "
  );

  Serial.println(
    resultB1.sdY,
    7
  );


  Serial.print(
    "B2 Y = "
  );

  Serial.print(
    resultB2.y,
    7
  );

  Serial.print(
    "  SD = "
  );

  Serial.println(
    resultB2.sdY,
    7
  );


  Serial.print(
    "A2 Y = "
  );

  Serial.print(
    resultA2.y,
    7
  );

  Serial.print(
    "  SD = "
  );

  Serial.println(
    resultA2.sdY,
    7
  );


  Serial.println();


  Serial.print(
    "Y primary component = "
  );

  Serial.print(
    earthY,
    7
  );

  Serial.println(
    " deg/s"
  );


  Serial.print(
    "X cross-axis component = "
  );

  Serial.print(
    earthX,
    7
  );

  Serial.println(
    " deg/s"
  );


  Serial.print(
    "Horizontal Earth rate = "
  );

  Serial.print(
    earthHorizontal,
    7
  );

  Serial.println(
    " deg/s"
  );


  Serial.print(
    "180 degree reversal = "
  );

  Serial.print(
    reversalStep,
    7
  );

  Serial.println(
    " deg/s"
  );


  Serial.print(
    "ABBA repeatability +/- = "
  );

  Serial.print(
    repeatabilityHorizontal,
    7
  );

  Serial.println(
    " deg/s"
  );


  Serial.print(
    "Alignment from Y = "
  );

  Serial.print(
    alignmentFromYDeg,
    2
  );

  Serial.println(
    " deg"
  );


  Serial.print(
    "Latitude magnitude = "
  );

  Serial.print(
    latitudeDeg,
    2
  );

  Serial.println(
    " deg"
  );


  Serial.print(
    "Latitude repeatability +/- = "
  );

  Serial.print(
    latitudeRepeatabilityDeg,
    2
  );

  Serial.println(
    " deg"
  );


  Serial.println(
    "=================================="
  );
}


// ============================================================
// ABBA STATE MACHINE
// ============================================================

void updateABBA()
{

// ============================================================
// DELETE DATA
// ============================================================

  if (
    clearRequested
  )
  {
    clearRequested =
      false;


    armNewABBA();


    return;
  }


// ============================================================
// PAUSE
// ============================================================

  if (
    pauseRequested
  )
  {
    pauseRequested =
      false;


    phyphoxRunning =
      false;


    // Finished run:
    // keep results.

    if (
      abbaState ==
      ABBA_FINISHED
    )
    {
      Serial.println(
        "RESULT FROZEN"
      );


      return;
    }


    // Paused while waiting:
    // harmless.

    if (
      abbaState ==
      WAIT_CLEAR
      ||
      abbaState ==
      READY_TO_START
    )
    {
      return;
    }


    // Pause during ABBA:
    // discard run.

    abortABBA();


    return;
  }


// ============================================================
// START
// ============================================================

  if (
    startRequested
  )
  {
    startRequested =
      false;


    if (
      armed
      &&
      abbaState ==
      READY_TO_START
    )
    {
      phyphoxRunning =
        true;


      enterState(
        A1_SETTLE
      );
    }


    else if (
      abbaState ==
      WAIT_CLEAR
    )
    {
      Serial.println(
        "DELETE DATA FIRST"
      );
    }


    return;
  }


// ============================================================
// DO NOT ADVANCE IF PAUSED
// ============================================================

  if (
    !phyphoxRunning
  )
  {
    return;
  }


  uint32_t elapsed =
    millis() -
    stateStartMs;


// ============================================================
// A1 SETTLE
// ============================================================

  if (
    abbaState ==
    A1_SETTLE
    &&
    elapsed >=
    SETTLE_TIME_MS
  )
  {
    enterState(
      A1_MEASURE
    );


    return;
  }


// ============================================================
// A1 MEASURE
// ============================================================

  if (
    abbaState ==
    A1_MEASURE
    &&
    elapsed >=
    MEASURE_TIME_MS
  )
  {
    resultA1 =
      finishBlock();


    enterState(
      ROTATE_TO_B
    );


    return;
  }


// ============================================================
// ROTATE TO Y-
// ============================================================

  if (
    abbaState ==
    ROTATE_TO_B
    &&
    elapsed >=
    ROTATE_TIME_MS
  )
  {
    enterState(
      B1_SETTLE
    );


    return;
  }


// ============================================================
// B SETTLE
// ============================================================

  if (
    abbaState ==
    B1_SETTLE
    &&
    elapsed >=
    SETTLE_TIME_MS
  )
  {
    enterState(
      B1_MEASURE
    );


    return;
  }


// ============================================================
// B1 MEASURE
// ============================================================

  if (
    abbaState ==
    B1_MEASURE
    &&
    elapsed >=
    MEASURE_TIME_MS
  )
  {
    resultB1 =
      finishBlock();


    // No movement:
    // B2 begins immediately.

    enterState(
      B2_MEASURE
    );


    return;
  }


// ============================================================
// B2 MEASURE
// ============================================================

  if (
    abbaState ==
    B2_MEASURE
    &&
    elapsed >=
    MEASURE_TIME_MS
  )
  {
    resultB2 =
      finishBlock();


    enterState(
      ROTATE_TO_A
    );


    return;
  }


// ============================================================
// ROTATE BACK TO Y+
// ============================================================

  if (
    abbaState ==
    ROTATE_TO_A
    &&
    elapsed >=
    ROTATE_TIME_MS
  )
  {
    enterState(
      A2_SETTLE
    );


    return;
  }


// ============================================================
// A2 SETTLE
// ============================================================

  if (
    abbaState ==
    A2_SETTLE
    &&
    elapsed >=
    SETTLE_TIME_MS
  )
  {
    enterState(
      A2_MEASURE
    );


    return;
  }


// ============================================================
// A2 MEASURE
// ============================================================

  if (
    abbaState ==
    A2_MEASURE
    &&
    elapsed >=
    MEASURE_TIME_MS
  )
  {
    resultA2 =
      finishBlock();


    calculateABBA();


    phyphoxRunning =
      false;


    enterState(
      ABBA_FINISHED
    );


    return;
  }
}


// ============================================================
// PHYPHOX EVENT HANDLER
//
// 0 = PAUSE
// 1 = START
// 2 = CLEAR
// ============================================================

void phyphoxEvent()
{
  uint8_t event =
    PhyphoxBLE::eventType;


  if (
    event == 0
  )
  {
    pauseRequested =
      true;
  }


  else if (
    event == 1
  )
  {
    startRequested =
      true;
  }


  else if (
    event == 2
  )
  {
    clearRequested =
      true;
  }
}


// ============================================================
// BUILD PHYPHOX EXPERIMENT
//
// NORMAL PACKET:
//
// CH1 = live X
// CH2 = live Y
// CH3 = live Z
// CH4 = ABBA step
// CH5 = countdown
//
// FINAL PACKET:
//
// CH1 = horizontal Earth rate
// CH2 = repeatability
// CH3 = estimated latitude
// CH4 = signed Y component
// CH5 = signed X component
// ============================================================

void setupPhyphox()
{
  snprintf(
    bleName,
    sizeof(
      bleName
    ),
    "Murata%d",
    MURATA_ID
  );


  PhyphoxBLE::start(
    bleName
  );


  PhyphoxBLE::experimentEventHandler =
    &phyphoxEvent;


  PhyphoxBleExperiment experiment;


  experiment.setTitle(
    "Murata Earth Rotation"
  );


  experiment.setCategory(
    "Physics"
  );


  experiment.setDescription(
    "SCH16T Earth rotation"
  );


// ============================================================
// LIVE TAB
// ============================================================

  PhyphoxBleExperiment::View liveView;


  liveView.setLabel(
    "Live"
  );


// ------------------------------------------------------------
// X
// ------------------------------------------------------------

  PhyphoxBleExperiment::Value valueX;


  valueX.setLabel(
    "X"
  );


  valueX.setUnit(
    "deg/s"
  );


  valueX.setPrecision(
    6
  );


  valueX.setChannel(
    1
  );


// ------------------------------------------------------------
// Y
// ------------------------------------------------------------

  PhyphoxBleExperiment::Value valueY;


  valueY.setLabel(
    "Y"
  );


  valueY.setUnit(
    "deg/s"
  );


  valueY.setPrecision(
    6
  );


  valueY.setChannel(
    2
  );


// ------------------------------------------------------------
// Z
// ------------------------------------------------------------

  PhyphoxBleExperiment::Value valueZ;


  valueZ.setLabel(
    "Z"
  );


  valueZ.setUnit(
    "deg/s"
  );


  valueZ.setPrecision(
    6
  );


  valueZ.setChannel(
    3
  );


// ------------------------------------------------------------
// X GRAPH
// ------------------------------------------------------------

  PhyphoxBleExperiment::Graph graphX;


  graphX.setLabel(
    "Gyroscope X"
  );


  graphX.setUnitX(
    "s"
  );


  graphX.setUnitY(
    "deg/s"
  );


  graphX.setLabelX(
    "Time"
  );


  graphX.setLabelY(
    "Angular rate"
  );


  graphX.setXPrecision(
    1
  );


  graphX.setYPrecision(
    6
  );


  graphX.setChannel(
    0,
    1
  );


// ------------------------------------------------------------
// Y GRAPH
// ------------------------------------------------------------

  PhyphoxBleExperiment::Graph graphY;


  graphY.setLabel(
    "Gyroscope Y"
  );


  graphY.setUnitX(
    "s"
  );


  graphY.setUnitY(
    "deg/s"
  );


  graphY.setLabelX(
    "Time"
  );


  graphY.setLabelY(
    "Angular rate"
  );


  graphY.setXPrecision(
    1
  );


  graphY.setYPrecision(
    6
  );


  graphY.setChannel(
    0,
    2
  );


// ------------------------------------------------------------
// Z GRAPH
// ------------------------------------------------------------

  PhyphoxBleExperiment::Graph graphZ;


  graphZ.setLabel(
    "Gyroscope Z"
  );


  graphZ.setUnitX(
    "s"
  );


  graphZ.setUnitY(
    "deg/s"
  );


  graphZ.setLabelX(
    "Time"
  );


  graphZ.setLabelY(
    "Angular rate"
  );


  graphZ.setXPrecision(
    1
  );


  graphZ.setYPrecision(
    6
  );


  graphZ.setChannel(
    0,
    3
  );


  liveView.addElement(
    valueX
  );


  liveView.addElement(
    valueY
  );


  liveView.addElement(
    valueZ
  );


  liveView.addElement(
    graphX
  );


  liveView.addElement(
    graphY
  );


  liveView.addElement(
    graphZ
  );


  experiment.addView(
    liveView
  );


// ============================================================
// ABBA TAB
// ============================================================

  PhyphoxBleExperiment::View abbaView;


  abbaView.setLabel(
    "ABBA"
  );


// ------------------------------------------------------------
// TITLE
// ------------------------------------------------------------

  PhyphoxBleExperiment::InfoField abbaTitle;


  abbaTitle.setInfo(
    "GUIDED ABBA"
  );


  abbaTitle.setColor(
    "39a2ff"
  );


  abbaTitle.setXMLAttribute(
    "size=\"1.5\""
  );


// ------------------------------------------------------------
// START INSTRUCTIONS
// ------------------------------------------------------------

  PhyphoxBleExperiment::InfoField clearInfo;


  clearInfo.setInfo(
    "1. DELETE DATA"
  );


  PhyphoxBleExperiment::InfoField startInfo;


  startInfo.setInfo(
    "2. Y+ NORTH, then START"
  );


// ------------------------------------------------------------
// CURRENT STEP
// ------------------------------------------------------------

  PhyphoxBleExperiment::Value stepValue;


  stepValue.setLabel(
    "CURRENT STEP"
  );


  stepValue.setPrecision(
    0
  );


  stepValue.setChannel(
    4
  );


  stepValue.setColor(
    "ffffff"
  );


  stepValue.setXMLAttribute(
    "size=\"2.5\""
  );


// ------------------------------------------------------------
// COUNTDOWN
// ------------------------------------------------------------

  PhyphoxBleExperiment::Value countdownValue;


  countdownValue.setLabel(
    "Seconds remaining"
  );


  countdownValue.setUnit(
    "s"
  );


  countdownValue.setPrecision(
    0
  );


  countdownValue.setChannel(
    5
  );


  countdownValue.setColor(
    "ffffff"
  );


  countdownValue.setXMLAttribute(
    "size=\"2\""
  );


// ============================================================
// STEP GUIDE
// ============================================================

  PhyphoxBleExperiment::InfoField step2;


  step2.setInfo(
    "2 SETTLE Y+ NORTH"
  );


  step2.setColor(
    "ffffff"
  );


// ------------------------------------------------------------

  PhyphoxBleExperiment::InfoField step3;


  step3.setInfo(
    "3 MEASURE A1"
  );


  step3.setColor(
    "ffffff"
  );


// ------------------------------------------------------------

  PhyphoxBleExperiment::InfoField step4;


  step4.setInfo(
    "4 ROTATE SLOWLY TO Y-"
  );


  step4.setColor(
    "ff3030"
  );


  step4.setXMLAttribute(
    "size=\"1.3\""
  );


// ------------------------------------------------------------

  PhyphoxBleExperiment::InfoField step5;


  step5.setInfo(
    "5 SETTLE Y- NORTH"
  );


  step5.setColor(
    "ffffff"
  );


// ------------------------------------------------------------

  PhyphoxBleExperiment::InfoField step6;


  step6.setInfo(
    "6 MEASURE B1"
  );


  step6.setColor(
    "ffffff"
  );


// ------------------------------------------------------------

  PhyphoxBleExperiment::InfoField step7;


  step7.setInfo(
    "7 MEASURE B2"
  );


  step7.setColor(
    "ffffff"
  );


// ------------------------------------------------------------

  PhyphoxBleExperiment::InfoField step8;


  step8.setInfo(
    "8 ROTATE SLOWLY TO Y+"
  );


  step8.setColor(
    "ff3030"
  );


  step8.setXMLAttribute(
    "size=\"1.3\""
  );


// ------------------------------------------------------------

  PhyphoxBleExperiment::InfoField step9;


  step9.setInfo(
    "9 SETTLE Y+ NORTH"
  );


  step9.setColor(
    "ffffff"
  );


// ------------------------------------------------------------

  PhyphoxBleExperiment::InfoField step10;


  step10.setInfo(
    "10 MEASURE A2"
  );


  step10.setColor(
    "ffffff"
  );


// ------------------------------------------------------------

  PhyphoxBleExperiment::InfoField step11;


  step11.setInfo(
    "11 FINISHED - PAUSE"
  );


  step11.setColor(
    "39a2ff"
  );


  step11.setXMLAttribute(
    "size=\"1.3\""
  );


// ------------------------------------------------------------
// LIVE Y
// ------------------------------------------------------------

  PhyphoxBleExperiment::Value liveY;


  liveY.setLabel(
    "Live Y"
  );


  liveY.setUnit(
    "deg/s"
  );


  liveY.setPrecision(
    6
  );


  liveY.setChannel(
    2
  );


  abbaView.addElement(
    abbaTitle
  );


  abbaView.addElement(
    clearInfo
  );


  abbaView.addElement(
    startInfo
  );


  abbaView.addElement(
    stepValue
  );


  abbaView.addElement(
    countdownValue
  );


  abbaView.addElement(
    step2
  );


  abbaView.addElement(
    step3
  );


  abbaView.addElement(
    step4
  );


  abbaView.addElement(
    step5
  );


  abbaView.addElement(
    step6
  );


  abbaView.addElement(
    step7
  );


  abbaView.addElement(
    step8
  );


  abbaView.addElement(
    step9
  );


  abbaView.addElement(
    step10
  );


  abbaView.addElement(
    step11
  );


  abbaView.addElement(
    liveY
  );


  experiment.addView(
    abbaView
  );


// ============================================================
// RESULTS TAB
// ============================================================

  PhyphoxBleExperiment::View resultsView;


  resultsView.setLabel(
    "Results"
  );


// ------------------------------------------------------------
// TITLE
// ------------------------------------------------------------

  PhyphoxBleExperiment::InfoField resultTitle;


  resultTitle.setInfo(
    "EARTH ROTATION RESULT"
  );


  resultTitle.setColor(
    "39a2ff"
  );


  resultTitle.setXMLAttribute(
    "size=\"1.5\""
  );


// ------------------------------------------------------------
// HORIZONTAL EARTH ROTATION
// ------------------------------------------------------------

  PhyphoxBleExperiment::Value earthValue;


  earthValue.setLabel(
    "Horizontal Earth rotation"
  );


  earthValue.setUnit(
    "deg/s"
  );


  earthValue.setPrecision(
    6
  );


  earthValue.setChannel(
    1
  );


  earthValue.setColor(
    "39a2ff"
  );


  earthValue.setXMLAttribute(
    "size=\"2\""
  );


// ------------------------------------------------------------
// REPEATABILITY
// ------------------------------------------------------------

  PhyphoxBleExperiment::Value errorValue;


  errorValue.setLabel(
    "ABBA repeatability +/-"
  );


  errorValue.setUnit(
    "deg/s"
  );


  errorValue.setPrecision(
    6
  );


  errorValue.setChannel(
    2
  );


// ------------------------------------------------------------
// LATITUDE
// ------------------------------------------------------------

  PhyphoxBleExperiment::Value latitudeValue;


  latitudeValue.setLabel(
    "Estimated latitude"
  );


  latitudeValue.setUnit(
    "deg"
  );


  latitudeValue.setPrecision(
    1
  );


  latitudeValue.setChannel(
    3
  );


  latitudeValue.setXMLAttribute(
    "size=\"1.7\""
  );


// ------------------------------------------------------------
// Y PRIMARY COMPONENT
// ------------------------------------------------------------

  PhyphoxBleExperiment::Value yResult;


  yResult.setLabel(
    "Y primary component"
  );


  yResult.setUnit(
    "deg/s"
  );


  yResult.setPrecision(
    6
  );


  yResult.setChannel(
    4
  );


// ------------------------------------------------------------
// X CROSS-AXIS COMPONENT
// ------------------------------------------------------------

  PhyphoxBleExperiment::Value xResult;


  xResult.setLabel(
    "X cross-axis component"
  );


  xResult.setUnit(
    "deg/s"
  );


  xResult.setPrecision(
    6
  );


  xResult.setChannel(
    5
  );


// ------------------------------------------------------------
// SHORT NOTES
//
// Keep short to avoid setInfo ERR_01.
// ------------------------------------------------------------

  PhyphoxBleExperiment::InfoField yNote;


  yNote.setInfo(
    "Y should dominate."
  );


// ------------------------------------------------------------

  PhyphoxBleExperiment::InfoField xNote;


  xNote.setInfo(
    "X checks alignment."
  );


// ------------------------------------------------------------

  PhyphoxBleExperiment::InfoField vectorNote;


  vectorNote.setInfo(
    "X + Y give horizontal rate."
  );


// ------------------------------------------------------------

  PhyphoxBleExperiment::InfoField errorNote;


  errorNote.setInfo(
    "Repeatability: within-run."
  );


  resultsView.addElement(
    resultTitle
  );


  resultsView.addElement(
    earthValue
  );


  resultsView.addElement(
    errorValue
  );


  resultsView.addElement(
    latitudeValue
  );


  resultsView.addElement(
    yResult
  );


  resultsView.addElement(
    xResult
  );


  resultsView.addElement(
    yNote
  );


  resultsView.addElement(
    xNote
  );


  resultsView.addElement(
    vectorNote
  );


  resultsView.addElement(
    errorNote
  );


  experiment.addView(
    resultsView
  );


// ============================================================
// EXPORT
// ============================================================

  PhyphoxBleExperiment::ExportSet exportSet;


  exportSet.setLabel(
    "Murata Data"
  );


// ------------------------------------------------------------
// TIME
// ------------------------------------------------------------

  PhyphoxBleExperiment::ExportData exportTime;


  exportTime.setLabel(
    "Time (s)"
  );


  exportTime.setDatachannel(
    0
  );


// ------------------------------------------------------------
// CHANNEL 1
// ------------------------------------------------------------

  PhyphoxBleExperiment::ExportData export1;


  export1.setLabel(
    "X / final horizontal"
  );


  export1.setDatachannel(
    1
  );


// ------------------------------------------------------------
// CHANNEL 2
// ------------------------------------------------------------

  PhyphoxBleExperiment::ExportData export2;


  export2.setLabel(
    "Y / final repeatability"
  );


  export2.setDatachannel(
    2
  );


// ------------------------------------------------------------
// CHANNEL 3
// ------------------------------------------------------------

  PhyphoxBleExperiment::ExportData export3;


  export3.setLabel(
    "Z / final latitude"
  );


  export3.setDatachannel(
    3
  );


// ------------------------------------------------------------
// CHANNEL 4
// ------------------------------------------------------------

  PhyphoxBleExperiment::ExportData export4;


  export4.setLabel(
    "Step / final Y"
  );


  export4.setDatachannel(
    4
  );


// ------------------------------------------------------------
// CHANNEL 5
// ------------------------------------------------------------

  PhyphoxBleExperiment::ExportData export5;


  export5.setLabel(
    "Countdown / final X"
  );


  export5.setDatachannel(
    5
  );


  exportSet.addElement(
    exportTime
  );


  exportSet.addElement(
    export1
  );


  exportSet.addElement(
    export2
  );


  exportSet.addElement(
    export3
  );


  exportSet.addElement(
    export4
  );


  exportSet.addElement(
    export5
  );


  experiment.addExportSet(
    exportSet
  );


// ============================================================
// ADD EXPERIMENT
// ============================================================

  PhyphoxBLE::addExperiment(
    experiment
  );
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(
    115200
  );


  delay(
    200
  );


  Serial.println();

  Serial.println(
    "=============================="
  );

  Serial.println(
    "MURATA EARTH ROTATION"
  );

  Serial.println(
    "Y-AXIS GUIDED ABBA"
  );

  Serial.println(
    "=============================="
  );


// ============================================================
// START PHYPHOX
// ============================================================

  setupPhyphox();


  Serial.print(
    "BLE name: "
  );


  Serial.println(
    bleName
  );


// ============================================================
// SPI
// ============================================================

  SPI.begin(
    SCK_PIN,
    MISO_PIN,
    MOSI_PIN,
    CS_PIN
  );


  delay(
    100
  );


// ============================================================
// SCH16T FILTER
// ============================================================

  Filter.Rate12 =
    13;


  Filter.Acc12 =
    13;


  Filter.Acc3 =
    13;


// ============================================================
// SENSITIVITY
// ============================================================

  Sensitivity.Rate1 =
    3200;


  Sensitivity.Rate2 =
    3200;


  Sensitivity.Acc1 =
    3200;


  Sensitivity.Acc2 =
    3200;


  Sensitivity.Acc3 =
    3200;


// ============================================================
// DECIMATION
// ============================================================

  Decimation.Rate2 =
    4;


  Decimation.Acc2 =
    4;


// ============================================================
// INITIALISE SENSOR
// ============================================================

  int status =
    imu.begin(
      Filter,
      Sensitivity,
      Decimation,
      false
    );


  Serial.print(
    "SCH16T init = "
  );


  Serial.println(
    status
  );


  if (
    status ==
    SCH16T_OK
  )
  {
    sensorOK =
      true;


    Serial.println(
      "*** SCH16T READY ***"
    );
  }

  else
  {
    sensorOK =
      false;


    Serial.println(
      "*** SCH16T FAILED ***"
    );
  }


// ============================================================
// INITIAL STATE
// ============================================================

  clearStoredResults();


  abbaState =
    WAIT_CLEAR;


  nextSampleUs =
    micros();


  Serial.println();


  Serial.print(
    "Earth rate = "
  );


  Serial.print(
    EARTH_RATE_DPS,
    8
  );


  Serial.println(
    " deg/s"
  );


  Serial.println(
    "50 Hz acquisition"
  );


  Serial.println(
    "10 Hz live display"
  );


  Serial.println(
    "5 s settling"
  );


  Serial.println(
    "10 s measurement"
  );


  Serial.println(
    "Y+ NORTH -> Y- NORTH"
  );


  Serial.println();
}


// ============================================================
// SEND LIVE PACKET
//
// CH1 X
// CH2 Y
// CH3 Z
// CH4 state
// CH5 countdown
// ============================================================

void sendLivePacket(
  float avgX,
  float avgY,
  float avgZ
)
{
  float state =
    (float)
    abbaState;


  float countdown =
    getCountdown();


  PhyphoxBLE::write(
    avgX,
    avgY,
    avgZ,
    state,
    countdown
  );
}


// ============================================================
// SEND FINAL PACKET
//
// IMPORTANT:
// PhyphoxBLE::write() in your installed library expects
// float REFERENCES.
//
// Therefore these MUST be named float variables.
//
// CH1 = horizontal Earth rate
// CH2 = repeatability
// CH3 = latitude
// CH4 = signed Y
// CH5 = signed X
// ============================================================

void sendFinalPacket()
{
  if (
    finalPacketSent
    ||
    !resultsValid
  )
  {
    return;
  }


  float finalEarth =
    (float)
    earthHorizontal;


  float finalRepeatability =
    (float)
    repeatabilityHorizontal;


  float finalLatitude =
    (float)
    latitudeDeg;


  float finalY =
    (float)
    earthY;


  float finalX =
    (float)
    earthX;


  PhyphoxBLE::write(
    finalEarth,
    finalRepeatability,
    finalLatitude,
    finalY,
    finalX
  );


  finalPacketSent =
    true;


  Serial.println(
    "Final result packet sent."
  );
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  PhyphoxBLE::poll();


// ============================================================
// UPDATE ABBA STATE MACHINE
// ============================================================

  updateABBA();


// ============================================================
// FINISHED
//
// Send results once and freeze.
// ============================================================

  if (
    abbaState ==
    ABBA_FINISHED
  )
  {
    sendFinalPacket();


    return;
  }


// ============================================================
// CHECK SENSOR
// ============================================================

  if (
    !sensorOK
  )
  {
    delay(
      100
    );


    return;
  }


// ============================================================
// 50 Hz SENSOR TIMING
// ============================================================

  uint32_t now =
    micros();


  if (
    (int32_t)(
      now -
      nextSampleUs
    )
    <
    0
  )
  {
    return;
  }


// Avoid catch-up burst after a long interruption

  if (
    (uint32_t)(
      now -
      nextSampleUs
    )
    >
    5 *
    SAMPLE_INTERVAL_US
  )
  {
    nextSampleUs =
      now;
  }


  nextSampleUs +=
    SAMPLE_INTERVAL_US;


// ============================================================
// READ SCH16T
// ============================================================

  imu.getData(
    &raw
  );


  if (
    raw.frame_error
  )
  {
    return;
  }


  imu.convertData(
    &raw,
    &result
  );


// SCH16T library already returns deg/s

  float gx =
    result.Rate1[0];


  float gy =
    result.Rate1[1];


  float gz =
    result.Rate1[2];


// ============================================================
// ABBA SCIENTIFIC DATA
//
// EVERY valid 50 Hz sample is included during:
//
// A1
// B1
// B2
// A2
//
// Rotation and settling periods are excluded.
// ============================================================

  if (
    abbaIsMeasuring()
  )
  {
    addBlockSample(
      gx,
      gy,
      gz
    );
  }


// ============================================================
// LIVE DISPLAY
//
// 50 Hz -> average 5 -> 10 Hz
// ============================================================

  displaySumX +=
    gx;


  displaySumY +=
    gy;


  displaySumZ +=
    gz;


  displayN++;


  if (
    displayN <
    DISPLAY_AVERAGE_COUNT
  )
  {
    return;
  }


  float avgX =
    displaySumX /
    displayN;


  float avgY =
    displaySumY /
    displayN;


  float avgZ =
    displaySumZ /
    displayN;


// ============================================================
// SEND LIVE DATA
// ============================================================

  sendLivePacket(
    avgX,
    avgY,
    avgZ
  );


// ============================================================
// RESET LIVE AVERAGING
// ============================================================

  displaySumX =
    0.0f;


  displaySumY =
    0.0f;


  displaySumZ =
    0.0f;


  displayN =
    0;
}