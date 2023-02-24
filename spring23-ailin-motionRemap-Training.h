#include "targetver.h"

#include <cstdlib>
#include <cmath>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <string>
#include <map>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <queue>
#include <string>


/**** BOOOST MULTITHREADED LIBRARY *********/
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>	//include asio in order to avoid the "winsock already declared problem"


#ifdef _WIN32
#include <windows.h>
//#include "glext.h" // opengl extensions for multisampling
#include <gl\gl.h>            // Header File For The OpenGL32 Library
#include <gl\glu.h>            // Header File For The GLu32 Library
#include "glut.h"            // Header File For The GLu32 Library
#include "freeglut.h"          // Header File For The GLu32 Library
#include <MMSystem.h>
#endif

/************ INCLUDE CNCSVISION LIBRARY HEADERS ****************/
#include "Mathcommon.h"
#include "GLUtils.h"
#include "VRCamera.h"
#include "CoordinatesExtractor.h"
#include "StimulusDrawer.h"
#include "GLText.h"

#include "ParametersLoader.h"
#include "Util.h"
#include "VRCamera.h"
#include "BalanceFactor.h"
#include "ParStaircase.h"
#include "Staircase.h"
#include "TrialGenerator.h"
#include "BrownPhidgets.h"
#include <direct.h>
#include "Optotrak2.h"
#include "Marker.h"
#include "BrownMotorFunctions.h"
#include <random>

/********* NAMESPACE DIRECTIVES ************************/
using namespace std;
using namespace mathcommon;
using namespace Eigen;
using namespace util;
using namespace BrownMotorFunctions;
using namespace BrownPhidgets;

/*************** Variable Declarations ********************/
static const bool gameMode = true;
const float DEG2RAD = M_PI / 180;

/********* VARIABLES OBJECTS  **********************/
VRCamera cam;
Optotrak2 optotrak;
Screen screen;
CoordinatesExtractor headEyeCoords, thumbCoords, indexCoords, thumbJointCoords, indexJointCoords;

/***** CALIBRATION FILE *****/
#include "Calibration_017B.h"
static const Vector3d center(0, 0, focalDistance);
double mirrorAlignment = 0.0, screenAlignmentY = 0.0, screenAlignmentZ = 0.0;

/********** EYES AND MARKERS **********************/
Vector3d eyeLeft, eyeRight, eyeMiddle;
vector <Marker> markers;
double interoculardistance = 60.0;
int screen1 = 19, screen2 = 20, screen3 = 21;
int mirror1 = 6, mirror2 = 22;

/********** FINGERS AND MARKERS **********************/
// finger markers
int ind1 = 13, ind2 = 14, ind3 = 16;
int thu1 = 15, thu2 = 17, thu3 = 18;
int calibration_T = 1, calibration_I = 2;

// Position variables for keeping track of finger movementm
Vector3d ind, indJoint, thm, thmJoint;
Vector3d homePos(0, 0, 0), thmTarget(0, 0, 0), visTarget(0, 0, 0), centeredObjEdge(0, 0, 0);
Vector3d vec_ind_thm, thmToHome, thmToTarget;
Vector3d indexCalibrationPoint(0, 0, 0), thumbCalibrationPoint(0, 0, 0), CalibrationPoint(0, 0, 0);
Vector3d indexCalibration_offset(-8, 0, -5.4), thumbCalibration_offset(-8, 0, 5.4);

bool allVisibleIndex = false;
bool allVisibleThumb = false;
bool allVisibleFingers = false;
int fingersOccluded = 0;

// thumb to home
double old_dist_thm_home, dist_thm_home = 0;
double vel_dist_home;
// threshold
double thresholdDist_near_home = 70;
bool handNearHome = false;
int holdCount_home = 0;
int threshHoldCout_home = 50;

// threshold
double thresholdVelthm_steady = 1.6;
bool handSteady = false;

// GA
double old_grip_aperture, grip_aperture = 0;
double vel_grip_change;
// threshold
double thresholdGA_small = 12, thresholdVelGA_steady = 1.2;
bool gripSmall = false;
bool gripSteady = false;

// MSE
double grip_aperture_MSE_first = 0;
double grip_aperture_MSE_second = 0;
bool attemped_MSE = false;



/********** TRIAL SPECIFIC PARAMETERS ***************/
ParametersLoader parameters;
ParametersLoader parameters_extra;
BalanceFactor<double> trial; //if using costant stimuli
//TrialGenerator<double> trial;//if using staircase: 


/*************************** INPUT AND OUTPUT ****************************/
// experiment directory
string experiment_directory = "C:/Users/labdomin/Documents/data/ailin/spring23-ailin-motionRemap/";

// paramters file directory and name
ifstream parametersFile;
string parametersFileName = experiment_directory + "parameters_spring23-ailin-motionRemap-Training.txt";
ifstream parametersFile_extra;
string parametersFileName_extra = experiment_directory + "parameters_spring23-ailin-motionRemap-Training-extra.txt";

// response file
ofstream responseFile;
string responseFile_headers = "subjName\tIOD\tblockN\ttrialN\tDepth\tDepthDelta\tDepth_text\tDepth_disp\treinforceTexture\tdisplayDistance\tMSE1\tMSE2\tRT_MSE1\tRT_MSE2\tcalibNum\tvisualAngle\tshapeHeight\tshapeWidth\tTexDotsDensity\tTexDotRadius\tTexDotSepRatio\tmtFlSpeedMod\trockMod\tmtFlTime\tRandomTraining";

string subjectName;

/**********	TRIALS **************/
int sessionNum = 0;
bool session_full_vs_extra = true;

int totalBlkNum = 1;
int blkNum = 1;
int trialNum = 0;
int trialNum_max = 1000;

int trainNum_cap = 20;

double percentComplete = 0;
int repetition = 4;
int totalTrNum = 6 * 8 * repetition;

int occlusionFrames_MSE = 0;

/********** STIMULUS SHAPE ***************/
// stimulus shape
double display_distance;
double visual_angle = 8.0; // stim diangonal size

//height and width
double stimulus_height = 70; //tan((DEG2RAD * visual_angle)/2) * 2 * (abs(display_distance));
double stimulus_width = 70; //ratio_bgwidth_height * stimulus_height;
double stimulus_visiblewidth = 70; //ratio_visiblewidth_height * stimulus_height;
double ratio_width_height = 1.3;//1.3;
double ratio_visiblewidth_height = 1.1;//1.1;

// depths of visual stimuli
double depth_mean = 40;
double depth_delta = 0;
double depth_text = 40;
double depth_disp = 40;
double l_curve_text, l_curve_disp;

// training
double depth_training_min = 20; // set in the subject file
int depth_training_range = 24; // set in the subject file
double depth_inc = 10;
// jitter in distance
double jitter_z = 0;
double display_distance_jittered = display_distance + jitter_z;
double visualTarget_X = 0, visualTarget_Y = 0;
double dist_toEye = -display_distance_jittered;

/********** STIMULUS VERTICES ***************/
struct CurveYLMap {
	std::vector<double> y_vec;
	std::vector<double> l_vec;
	double curve_depth;
	double curve_height;
	double step_size;
};

struct CurvePtsData {
	std::vector<double> y_vec;
	std::vector<double> z_vec;
	std::vector<double> l_vec;
	double curve_depth;
	double curve_height;
	double step_size;
};

struct Vec2 {
	float x, y;
};

struct ProjTexDots_ResizeMap {
	std::vector<float> l_vec;
	std::vector<Vec2> R_resize_vec;
	double depth_proj;
	double depth_origin;
	double del_l;
	double R_Rsz_y_max;
};


struct TextureDotsData {
	std::vector<Vec2> dot_center_vec;
	std::vector<Vec2> R_resize_vec;
	Vec2 TexMapSize;
	float Radius;
	float Radius_y_max;
	int nr_D_L;
	int nr_D_H;
	int nr_S;
};

struct VerticesData {
	std::vector<GLfloat> vertices_vec;
	std::vector<GLfloat> colors_vec;
	std::vector<GLfloat> light_normals_vec;
	std::vector<GLuint> indices_draw_triangle_vec;

};
VerticesData my_verts_static, my_verts_moving;

struct ContourData {
	std::vector<Vector3f> vert_Rcontour;
	std::vector<Vector3f> vert_Lcontour;
};
ContourData my_contour_data;

struct AllTimeColorsVec {
	vector<vector<GLfloat>> colors_vec_allTimeVec;
};
AllTimeColorsVec AllTimeColorsVec_Moving;

/********** VERTEX RESOLUTION ***************/
int nr_curve_map = 10001;
int i_map_mid = (nr_curve_map - 1) / 2;

double del_l = 0.4;
int nr_points_width = 301; // nr of points in x direction
int nr_points_height_default = 251; // default
int nr_points_height = nr_points_height_default;
int total_ind = 0;

/********* TEXTURE *********/
float Tex_dot_density = 0.015; //0.02;
float Tex_dot_radius = 2.3; // 2.2;
float Tex_dot_separation_ratio = 1.45;

//blur edge
double drop_off_rate = 0.42;
double R_blur_fac = 2 / (1 + drop_off_rate);//1.28;

/********** LIGHTING ***************/
float max_intensity = 0.8;
float amb_intensity = 0.4;
float lightDir_z = 0.5;

/********** MOVEMENT ***************/
int move_cnt = 0;
int nr_mvpts_max = 20;
double speed_moderator = 7;
double speed_moderator_Text = 7.5;
double speed_moderator_Disp = 5.5;
int rock_movement_divider = 2;
double updateEveryMs = 30;
double cycle_time = 1200;
double motionFlowTime = cycle_time;
double mv_num = 3;

/********** ONLINE CONTROL ***************/
enum Stages { exp_initializing, stimulus_preview, stimulus_previewMotion, prep_trial, trial_fixate, trial_viewStatic, trial_MSE_first, trial_MSE_reset_first, trial_viewMtFlow, trial_MSE_second, trial_MSE_reset_second, break_time, exp_completed };
Stages current_stage = exp_initializing; // if just want to look at the stimuli, use the constant present stage

enum expInitSteps { to_GetCalibrationPoints, to_CalibrateFingerTips, to_CalibrateFingerJoints, to_MarkHomePos, to_MoveApparatus, to_confirmReady };
expInitSteps currentInitStep = to_GetCalibrationPoints;
int calibrationNum = 0;

bool resetScreen_betweenRuns = false;
bool reinforce_texture_disparity = true;
bool training = true;
bool visibleInfo = true;
bool fingerCalibration_TIPandJOINT = false; // if false, calibration two points - tip cand joint
bool Fingers_Calibrated = false;
bool Exp_Initialized = false;
bool screenBack = false; //has the screen been placed at projection distance
bool stimulus_built = false;
bool roll_rock = false;
bool task_guide_info = true;
bool trainingCueIsRandom = false;

enum errorStates { no_error, training_exceeded };
errorStates current_error_state = no_error;

// Timer variable, set for each trial 
Timer trial_timer;
double ElapsedTime;
double last_time = 0;

double timestamp_MSEstart1, timestamp_MSEend1;
double timestamp_MSEstart2, timestamp_MSEend2;
double timestamp_mtFlow;
double fixateTime = 600;

/*********** for DEBUGGING **********/
float time_var = 200;
bool testVisualStimuliOnly = false;
/*************************** FUNCTIONS ***********************************/
void initOptotrak();
void initMotors();
void initRendering();
void initVariables();
void initStreams();
void handleResize(int w, int h);
void initProjectionScreen(double _focalDist, const Affine3d& _transformation = Affine3d::Identity(), bool synchronous = true);
void updateTheMarkers();
void online_apparatus_alignment();
void cleanup();
void shutdown();
void beepOk(int tone);

void initMotionFlow();
void initSurface();
void initBlock();
void initTrial();
void onlineTrial();
void advanceTrial();

void drawStimulus();
void drawSurface(double distShapeToEye, const VerticesData& vertices_data, const ContourData& contours_vert);
void drawContours(const ContourData& contours_vert);
void drawProgressBar();
void drawFixation(double displayDist);

void buildSurface_congruent(double shapeWidth, double shapeHeight, double shapeDepth, double distShapeToEye, double contourPanelSeparation);
void buildSurface_incongruent(double shapeWidth, double shapeHeight, double dispDepth, double textDepth, double distShapeToEye, double contourPanelSeparation);
void buildSurface_TexOnDisp(double shapeWidth, const CurvePtsData& dispYCurve, const CurvePtsData& textYCurve, double distShapeToEye, TextureDotsData& TexDotsOnDisp, VerticesData& vertices_data);
void buildSurface_TexOnText(double shapeWidth, const CurvePtsData& dispYCurve, const CurvePtsData& textYCurve, double distShapeToEye, TextureDotsData& TexDotsOnText, VerticesData& vertices_data);
void buildContour(double ContourWidth, const CurvePtsData& dispYCurve, const CurvePtsData& textYCurve, float distShapeToEye, ContourData& new_contours_vert);
void buildAllColorsVec(const VerticesData& vertices_data, AllTimeColorsVec& colorsVecs);
void updateVerticesData(int timeID);

void scanCurve(double shapeHeight, double shapeDepth, CurveYLMap& output_curve_ylmap);
void projectCurve(const CurveYLMap& curve_map_proj, double distShapeToEye, const CurvePtsData& origin_curve, CurvePtsData& output_curve_proj);
int buildCurve_byDelY(const CurveYLMap& input_curve_ylmap, CurvePtsData& output_curve);
int buildCurve_byDelL(const CurveYLMap& input_curve_ylmap, CurvePtsData& output_curve);
void generateTexDots(float TM_X, float TM_Y, float dotDensity, float dotRadius, float dotSeparationRatio, TextureDotsData& outputTexDots);
void sampleTexDotsResize(const CurveYLMap& textCurveYLMap, const CurveYLMap& dispCurveYLMap, double distShapeToEye, ProjTexDots_ResizeMap& output_RszMap);
void projectTexDots(double distShapeToEye, const CurveYLMap& YLMap_origin, const CurveYLMap& YLMap_proj, const TextureDotsData& input_TexDots_origin, const ProjTexDots_ResizeMap& RszMap_proj, TextureDotsData& output_TexDots_proj);

double getZ(double shapeHeight, double shapeDepth, double vertexY);
double getTg(double shapeHeight, double shapeDepth, double Y);
double mapLtoY(const CurveYLMap& inputYLMap, double TargetL, int i_int = i_map_mid);
double mapYtoL(const CurveYLMap& inputYLMap, double input_y);
double SolveForZ_projected(double theHeight, double newDepth, double l, double y0, double z0);

void drawInfo();
void drawInfo_alignment(GLText curText);
void drawInfo_fingers(GLText curText);
void drawInfo_calibrationMarkers(GLText curText);
void drawTaskGuide();

void calibrate_system();
void calibrate_fingers();
void draw_thumb_dots();
void online_fingers();
void drawMarker(int Marker_ID);
void drawFingersOcclusion();


