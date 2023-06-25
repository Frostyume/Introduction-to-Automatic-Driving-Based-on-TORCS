#ifdef _WIN32
#include <windows.h>
#endif

#include <math.h>
#include "driver_parking.h"
#include <cmath>
#include "class_Visualization.h"
#include "stdio.h"
#include <ostream>
#include <fstream>
#define PI 3.141592653589793238462643383279

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*/
//Do NOT modify the code below!

static void userDriverGetParam(float lotX, float lotY, float lotAngle, bool bFrontIn, float carX, float carY, float caryaw, float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam(bool* bFinished, float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void* pt);

// Module Entry Point
extern "C" int driver_parking(tModInfo * modInfo)
{
    memset(modInfo, 0, 10 * sizeof(tModInfo));
    modInfo[0].name = "driver_parking";	// name of the module (short).
    modInfo[0].desc = "user module for CyberParking";	// Description of the module (can be long).
    modInfo[0].fctInit = InitFuncPt;			// Init function.
    modInfo[0].gfId = 0;
    modInfo[0].index = 0;
    return 0;
}

// Module interface initialization
static int InitFuncPt(int, void* pt)
{
    tUserItf* itf = (tUserItf*)pt;
    itf->userDriverGetParam = userDriverGetParam;
    itf->userDriverSetParam = userDriverSetParam;
    printf("OK!\n");
    return 0;
}

//Do Not modify the code above!
/*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/

static float _midline[200][2];
static float _yaw, _yawrate, _speed, _acc, _width, _rpm, _lotX, _lotY, _lotAngle, _carX, _carY, _caryaw;
static int _gearbox;
static bool _bFrontIn;
float lastCaryaw;
static float lastVerticalDis = 0;
static void userDriverGetParam(float lotX, float lotY, float lotAngle, bool bFrontIn, float carX, float carY, float caryaw, float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm) {
    /* write your own code here */

    _lotX = lotX;
    _lotY = lotY;
    _lotAngle = lotAngle;
    _bFrontIn = bFrontIn;
    _carX = carX;
    _carY = carY;
    _caryaw = caryaw;
    for (int i = 0; i < 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
    _yaw = yaw;
    _yawrate = yawrate;
    _speed = speed;
    _acc = acc;
    _width = width;
    _rpm = rpm;
    _gearbox = gearbox;
    printf("caryaw %.6f  lotAngle %.6f", _caryaw, _lotAngle);
    printf("carX %.6f  carY %.6f", _carX, _carY);
    printf("lotX %.6f  lotY %.6f", _lotX, _lotY);
}

double constrain(double lowerBoundary, double upperBoundary, double input) {
    if (input > upperBoundary)
        return upperBoundary;
    else if (input < lowerBoundary)
        return lowerBoundary;
    return input;
}

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█ Define your variables here											█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
static int nCrtStage = 0;	//Current stage

enum ListCrtStage {			//Stage list
    StageApproaching,
    //StageTurnRound,
    //StageBacking,
    //StageKeepRight,
    StageDrifting1,
    StageBacking,
    StageEnterLot,
    StageLeaveLot,
    StageDrifting2,
    StageLeaving,
    //StageLeave
};

typedef struct Circle
{
    double r;
    int sign;
}circle;

circle getR(float x1, float y1, float x2, float y2, float x3, float y3)
{
    double a, b, c, d, e, f;
    double r, x, y;

    a = 2 * (x2 - x1);
    b = 2 * (y2 - y1);
    c = x2 * x2 + y2 * y2 - x1 * x1 - y1 * y1;
    d = 2 * (x3 - x2);
    e = 2 * (y3 - y2);
    f = x3 * x3 + y3 * y3 - x2 * x2 - y2 * y2;
    x = (b * f - e * c) / (b * d - e * a);
    y = (d * c - a * f) / (b * d - e * a);
    r = sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));
    x = constrain(-1000.0, 1000.0, x);
    y = constrain(-1000.0, 1000.0, y);
    r = constrain(1.0, 500.0, r);
    int sign = (x > 0) ? 1 : -1;
    circle tmpresult = { r,sign };
    return tmpresult;
}
class PIDController
{
private:
    double kp, ki, kd;		// PID控制器的参数
    double targetValue;		// 目标值
    double lastError;		// 上一次误差值
    double errorIntegral;	// 误差积分值

public:
    double get_lastError()
    {
        return lastError;
    }
    void initial(double p, double i, double d, double target)
    {
        kp = p;
        ki = i;
        kd = d;
        targetValue = target;
        lastError = 0;
        errorIntegral = 0;
    }

    double calculate(double input)
    {
        double error = targetValue - input;
        double derivative = error - lastError;
        errorIntegral += error;
        lastError = error;
        return kp * error + ki * errorIntegral + kd * derivative;
    }
};
PIDController angleController;
cls_VISUAL cls_visual;
bool flag = true;
int counter = 60;
bool isFirstFrame = true;
double leftline[50][2];
double rightline[50][2];
float deltaX[50];
float deltaY[50];
double theta[50];
static double alpha;
static double lotlineX, lotlineY, errX, errY;
static double currentAngleError = 0.0;
static double targetAngleError = 0.0;
static float direction;
static float lotAngle, carAngle;
double bias1 = 0.0, bias2 = 0.0;
bool isRight = false, isForward = false;
int preview = 5;
static circle nextCurve;

static void userDriverSetParam(bool* bFinished, float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear)
{
    /*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
    █ Write your own code here												█
    \*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
    float fDis2Lot = sqrt((_carX - _lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY));		//Distance to the parking lot
    float fVerticalDis2Lot;																		//Vertical distance to the parking lot
    float fLateralDis2Lot;
    float minCurve = 500.0;
    circle myCurve;

    angleController.initial(8.5, 0, 0, targetAngleError);

    if (abs(_lotAngle) > (PI / 2 - 0.02) && abs(_lotAngle) < (PI / 2 + 0.02)) {
        fVerticalDis2Lot = abs(_carX - _lotX);
        fLateralDis2Lot = abs(_carY - _lotY);
        isRight = _carX < _lotX;
        isForward = _carY > _lotY;
    }
    else if (abs(_lotAngle) > (PI - 0.03) && abs(_lotAngle) < (PI + 0.03)) {
        fVerticalDis2Lot = abs(_carY - _lotY);
        fLateralDis2Lot = abs(_carX - _lotX);
        isRight = _carY > _lotY;
        isForward = _carX > _lotX;
    }
    else {
        float k1 = tan(_lotAngle);
        float b1 = -k1 * _lotX + _lotY;
        float k2 = -1 / k1;
        float b2 = -k2 * _lotX + _lotY;
        fVerticalDis2Lot = abs(k1 * _carX - _carY + b1) / sqrt(k1 * k1 + 1);
        fLateralDis2Lot = abs(k2 * _carX - _carY + b2) / sqrt(k2 * k2 + 1);
        isRight = k1 * _carX - _carY + b1 < 0;
        isForward = lotAngle < PI ? (k2 * _carX - _carY + b2 > 0) : (k2 * _carX - _carY + b2 < 0);
    }


    for (int fStep = 0; fStep < 30; fStep++)
    {
        myCurve = getR(_midline[fStep][0], _midline[fStep][1], _midline[fStep + 4][0], _midline[fStep + 4][1], _midline[fStep + 8][0], _midline[fStep + 8][1]);
        if (myCurve.r < minCurve)
        {
            minCurve = myCurve.r;
        }
    }
    bias1 = 0.2;
    bias2 = 0.3;
    //计算道路左右边线，重新规划巡线路径
    for (int i = 1; i < 50; i++) {
        deltaX[i] = abs(_midline[i][0] - _midline[i - 1][0]);
        deltaY[i] = abs(_midline[i][1] - _midline[i - 1][1]);
        theta[i] = atan2(deltaX[i], deltaY[i]);
        leftline[i][0] = _midline[i][0] - cos(theta[i]) * _width * bias1;
        leftline[i][1] = _midline[i][1] + sin(theta[i]) * _width * bias1;
    }
    for (int i = 1; i < 50; i++) {
        deltaX[i] = abs(_midline[i][0] - _midline[i - 1][0]);
        deltaY[i] = abs(_midline[i][1] - _midline[i - 1][1]);
        theta[i] = atan2(deltaX[i], deltaY[i]);
        rightline[i][0] = _midline[i][0] + cos(theta[i]) * _width * bias2;
        rightline[i][1] = _midline[i][1] - sin(theta[i]) * _width * bias2;
    }
    currentAngleError = atan2(leftline[15][0], leftline[15][1]);

    carAngle = _caryaw;
    lotAngle = _lotAngle;

    if (_caryaw < 0)
    {
        carAngle = 2 * PI + _caryaw;
    }
    if (_lotAngle < 0)
    {
        lotAngle = 2 * PI + _lotAngle;
    }
    direction = lotAngle - carAngle;
    if (direction > PI)
    {
        direction -= 2 * PI;
    }
    else if (direction < -PI)
    {
        direction += 2 * PI;
    }

    alpha = isRight ? PI / 2 + carAngle - lotAngle : PI / 2 - carAngle + lotAngle;
    alpha = alpha > PI/2 ? PI - alpha : alpha;
    errX = abs(fVerticalDis2Lot / sin(alpha));
    if ((isRight && carAngle > lotAngle) || (!isRight && carAngle <= lotAngle)) {
        errY = preview - fVerticalDis2Lot / tan(alpha);
        lotlineX = errX + errY * cos(alpha);
    }
    else if ((isRight && carAngle <= lotAngle) || (!isRight && carAngle > lotAngle)) {
        errY = preview + fVerticalDis2Lot / tan(alpha);
        lotlineX = errX - errY * cos(alpha);
    }
    if (!isRight)
    {
        lotlineX = -lotlineX;
    }
    lotlineY = errY * sin(alpha);

    printf("Stage: %d\n", nCrtStage);		//You can see the stage in the Console
    switch (nCrtStage)
    {
    case StageApproaching:
        *cmdGear = 1;
        *cmdSteer = constrain(-1.0, 1.0, angleController.calculate(currentAngleError));
        *cmdAcc = constrain(0.05, 1.0, 1.0 - tan(PI * 0.65 * abs(*cmdSteer)));
        *cmdBrake = 0.0;
        if (_speed > 91) *cmdAcc = 0;
        if (fVerticalDis2Lot < 22.5 && fDis2Lot < 50) {
            lastCaryaw = carAngle;
            nCrtStage++;
            nextCurve = getR(_midline[5][0], _midline[5][1], _midline[10][0], _midline[10][1], _midline[15][0], _midline[15][1]);
        }
        break;
        /*
        case StageTurnRound:
            if(_speed>20 && flag){
                *cmdSteer = -1;
                *cmdGear = 1;
                *cmdAcc = 0.0;
                *cmdBrake = 1.0;
                lastCaryaw = _caryaw;
            }
            else {
                flag = false;
                *cmdSteer = -1;
                *cmdGear = 1;
                *cmdAcc = 1.0;
                *cmdBrake = 0.0;
            }
            if(abs(_caryaw-lastCaryaw)>0.5)
                nCrtStage++;
            break;
        case StageBacking:
            *cmdSteer = constrain(-1.0, 1.0, angleController.calculate(currentAngleError));
            *cmdGear = -1;
            *cmdAcc = 1.0; *cmdBrake = 0.0;
            if (fDis2Lot < 5000)
                nCrtStage++;
            break;
        /*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
        █ Keep right to get more place to turn									█
        \*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
        /*case StageKeepRight:
            if(_speed < 5) {
                RightAngleError = atan2(leftline[15][0],leftline[15][1]);
                *cmdSteer = constrain(-0.3, 0.3, angleController.calculate(currentAngleError));
                *cmdAcc = 0.1;
                *cmdBrake = 0.0;
            }
            else if(_speed < 20){
                RightAngleError = atan2(leftline[15][0],leftline[15][1]);
                *cmdSteer = constrain(-0.6, 0.6, angleController.calculate(currentAngleError));
                *cmdAcc = 0.2;
                *cmdBrake = 0.0;
            }
            else{
                RightAngleError = atan2(leftline[15][0],leftline[15][1]);
                *cmdSteer = constrain(-1.0, 1.0, angleController.calculate(currentAngleError));
                *cmdAcc = 0.3;
                *cmdBrake = 0.0;
            }
            if (fVerticalDis2Lot > 7 && _carX>_lotX)
                nCrtStage++;
            break;*/
            /*        case StageSlowing:
                        *cmdSteer = -0.1;
                        *cmdGear = 0;
                        *cmdAcc = 0.0;
                        *cmdBrake = 0.25;
                        if (fVerticalDis2Lot > 13 && _speed < 60) //用速度区分车辆在车库的左右
                        {
                            *cmdBrake = 0.7;
                        }
                        if (_speed< 1)
                            nCrtStage++;
                        break;*/
    case StageDrifting1:
        if (abs(carAngle - lastCaryaw) < PI / 10 || abs(carAngle + 2 * PI - lastCaryaw) < PI / 10) {
            *cmdSteer = -1;
            /*if (nextCurve.sign == -1 && nextCurve.r < 150)  第一种
            {
                *cmdSteer = -0.9;
            }
            else if (nextCurve.sign == -1 && nextCurve.r < 300)
            {
                *cmdSteer = -0.95;
            }
            else
            {
                *cmdSteer = -1;
            }*/
            if (abs(lotAngle - lastCaryaw) < 2 * PI / 5)
            {
                *cmdSteer = -0.9;
            }
            else if (abs(lotAngle - lastCaryaw) < PI / 3)
            {
                *cmdSteer = -0.8;
            }
            else if (abs(lotAngle - lastCaryaw) < PI / 4)
            {
                *cmdSteer = -0.7;
            }
            *cmdAcc = 0;
            *cmdBrake = 0;
        }
        else {
            *cmdSteer = 0;
            *cmdAcc = 0;
            *cmdBrake = 1;
        }
        if (_speed < 10)
            nCrtStage++;
        break;
    case StageBacking:
        *cmdGear = -1;
        angleController.initial(20, 0, 0, targetAngleError);
        currentAngleError = atan2(lotlineX, lotlineY);
        *cmdSteer = constrain(-1.0, 1.0, -angleController.calculate(currentAngleError));
        if (abs(_speed) < 25)
        {
            *cmdAcc = 0.3;
            *cmdBrake = 0.0;
        }
        else
        {
            *cmdAcc = 0;
        }
        //lastVerticalDis = fVerticalDis2Lot;
        if (fDis2Lot < 4)
            nCrtStage++;
        break;
    case StageEnterLot:
        angleController.initial(30, 0, 0, targetAngleError);
        // currentAngleError = asin(fVerticalDis2Lot / fDis2Lot);
        currentAngleError = atan2(lotlineX, lotlineY);
        *cmdSteer = constrain(-1.0, 1.0, -angleController.calculate(currentAngleError));
        if (abs(_speed) < 0.2 && fLateralDis2Lot < 0.03) {
            *cmdSteer = 0;
            *bFinished = true;
            nCrtStage++;
        }
        else {
            if (fLateralDis2Lot > 0.5) {
                if (abs(_speed) > 5) {
                    *cmdAcc = 0;
                    *cmdBrake = constrain(0, 1.0, 0.25 / fDis2Lot);
                }
                else {
                    *cmdAcc = 0;
                    *cmdBrake = 0;
                }
            }
            else if (fLateralDis2Lot > 0.01) {
                if (abs(_speed) > 3) {
                    *cmdAcc = 0;
                    *cmdBrake = 0.5;
                }
                else {
                    *cmdGear = isForward ? 1 : -1;
                    *cmdAcc = 0.3;
                    *cmdBrake = 0;
                }
            }
            else {
                *cmdAcc = 0;
                *cmdBrake = 1;
            }
        }
        //lastVerticalDis = fVerticalDis2Lot;+
        break;
    case StageLeaveLot:
        currentAngleError = atan2(lotlineX, lotlineY);
        *cmdGear = 1;
        *cmdSteer = constrain(-1.0, 1.0, angleController.calculate(currentAngleError));
        *cmdAcc = 1;
        *cmdBrake = 0;
        if (fDis2Lot > 3) {
            lastCaryaw = carAngle;
            nCrtStage++;
        }
        break;
    case StageDrifting2:
        if (abs(carAngle - lastCaryaw) < PI / 8 || abs(carAngle + 2 * PI - lastCaryaw) < PI / 8) {
            *cmdSteer = 1;
            *cmdAcc = 1;
            *cmdBrake = 0;
        }
        else {
            *cmdSteer = 0;
            *cmdAcc = 0;
            *cmdBrake = 1;
        }
        if (_speed < 5)
            nCrtStage++;
        break;
    case StageLeaving:
        angleController.initial(8, 0, 0, targetAngleError);
        currentAngleError = atan2(rightline[5][0], rightline[5][1]);
        *cmdSteer = constrain(-1, 1, angleController.calculate(currentAngleError));
        *cmdAcc = constrain(0.05, 1.0, 1.0 - tan(PI * 0.65 * abs(*cmdSteer)));
        *cmdBrake = 0.0;
        break;
    default:
        break;
    }
    cls_visual.Fig2Y(1, 0, 300, 0, 500, 10, "steer", *cmdSteer, "v", _speed, "fDis2Lot", fDis2Lot);
    cls_visual.Fig2Y(2, 0, 300, 0, 500, 10, "fVerticalDis2Lot", fVerticalDis2Lot, "direction", direction, "nextCurve.r", nextCurve.r);
}