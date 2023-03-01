// this script aims to use probe adjustment task to measure the gain/strength of either texture or disparity as a depth cue
#include "spring23-ailin-motionRemap-Training.h"


double normalCDF(double value)
{
	double M_SQRT1_2 = sqrt(0.5);
	return 0.5 * erfc(-value * M_SQRT1_2);
}


double blurEdge(double dropoff, double pointDist, double intersectDist, double baseCol, double maxCol) {

	// given an Radius, I will choose the intersectDist to be 2/(1+dropoff)
	double addCol = maxCol - baseCol;
	return(((pointDist / intersectDist) < dropoff) ? baseCol : (baseCol + addCol * normalCDF(((pointDist / intersectDist - dropoff) / (1 - dropoff) * 6) - 3)));
}

double getZ(double shapeHeight, double shapeDepth, double Y) {

	double Z;

	Z = shapeDepth * cos(M_PI * Y / shapeHeight);

	return (Z);
}

double getTg(double shapeHeight, double shapeDepth, double Y) {
	return (-shapeDepth * sin(M_PI * Y / shapeHeight) * M_PI / shapeHeight);
}



double NewtonSolver_fz(double z, double Depth, double zCoeff, double distShapeToEye) {
	double val = z / Depth - cos(zCoeff * (z - distShapeToEye));
	return val;
}

double NewtonSolver_dfz(double z, double Depth, double zCoeff, double distShapeToEye) {
	double val = 1 / Depth + sin(zCoeff * (z - distShapeToEye)) * zCoeff;
	return val;
}

double SolveForZ_projected(double theHeight, double newDepth, double distShapeToEye, double y0, double z0) {

	double z_new, z, f_z, df_z;
	double C = M_PI * y0 / (theHeight * (z0 - distShapeToEye));

	z = z0;

	for (int i = 0; i < 100; i++) {

		f_z = NewtonSolver_fz(z, newDepth, C, distShapeToEye);
		df_z = NewtonSolver_dfz(z, newDepth, C, distShapeToEye);

		if (abs(f_z) < 1e-10) {

			break;
		}
		else if (abs(df_z) < 1e-10) {

			break;
		}
		else {
			z_new = z - f_z / df_z;
			z = z_new;
		}
	}

	if (abs(z - z0) > 40)
		z = 0;

	return z;

}

Vector3d projectPoint(double shapeHeight, double newDepth, double distShapeToEye, Vector3d fromPoint) {

	Vector3d ToPoint = fromPoint;
	if (abs(abs(fromPoint.y()) - shapeHeight / 2) > 0.01) {
		double z = SolveForZ_projected(shapeHeight, newDepth, distShapeToEye, fromPoint.y(), fromPoint.z());
		double w = (z - distShapeToEye) / (fromPoint.z() - distShapeToEye);
		ToPoint = Vector3d(w * fromPoint.x(), w * fromPoint.y(), z);
	}

	return ToPoint;
}


double mapLtoY(const CurveYLMap& inputYLMap, double TargetL, int i_int) {

	int i_c = i_int;
	double depth = inputYLMap.curve_depth;
	double height = inputYLMap.curve_height;
	double step_y_ylmap = inputYLMap.step_size;
	double y, l_diff, dldy;

	for (int i = 0; i < 100; i++) {

		l_diff = TargetL - inputYLMap.l_vec[i_c];

		y = inputYLMap.y_vec[i_c];

		double k = getTg(height, depth, y);
		dldy = sqrt(1 + k * k);

		if (abs(l_diff) <= step_y_ylmap * dldy) {

			break;
		}
		else {
			y = y + l_diff / dldy;

			i_c = (y + height / 2) / step_y_ylmap;


			if (i_c > (nr_curve_map - 1))
				i_c = nr_curve_map - 1;


			if (i_c < 0)
				i_c = 0;

		}
	}

	y = y + l_diff / dldy;

	return y;
}


void drawSurface(double distShapeToEye, const VerticesData& vertices_data, const ContourData& contours_vert) {

	//setting the light
	glShadeModel(GL_SMOOTH); // enable Smooth Shading
	glEnable(GL_LIGHTING); // enable lighting
	glEnable(GL_LIGHT1);
	glEnable(GL_NORMALIZE); //so we don't need to normalize our normal for surfaces	

	// Light source parameters
	GLfloat LightAmbient[] = { amb_intensity, 0.0f, 0.0f, 1.0f }; // non-directional & overall light (r,g,b,alpha): dark part
	GLfloat LightDiffuse[] = { max_intensity - amb_intensity, 0.0f, 0.0f, 1.0f }; // light created by the light source (directional light; r,g,b,alpha): bright part
	GLfloat LightPosition[] = { 0.0f, 1.f, lightDir_z, 0.0f }; // Light Position (x, y, z, 1.0f); if w==0, directional; if w==1, positional lights. Attenuation can be applied only to the positional light 

	//setting the light
	glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient); //setup the ambient light
	glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse); //setup the diffuse light
	glLightfv(GL_LIGHT1, GL_POSITION, LightPosition); //position the light


	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);

	glPushMatrix();
	glLoadIdentity();
	glTranslated(0, 0, -distShapeToEye);
	// activate and specify pointer to vertex array
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);

	//using vector
	glVertexPointer(3, GL_FLOAT, 0, &vertices_data.vertices_vec[0]);
	glNormalPointer(GL_FLOAT, 0, &vertices_data.light_normals_vec[0]); //
	glColorPointer(3, GL_FLOAT, 0, &vertices_data.colors_vec[0]);
	glDrawElements(GL_TRIANGLES, vertices_data.indices_draw_triangle_vec.size(), GL_UNSIGNED_INT, &vertices_data.indices_draw_triangle_vec[0]);

	// deactivate vertex arrays after drawing
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);

	glDisable(GL_LIGHTING);

	drawContours(contours_vert);
	glPopMatrix();

}

void drawContours(const ContourData& contours_vert) {

	int n;
	float panel_width = 40;
	float panel_height_extra = 20;


	glTranslated(0, 0, 1);
	n = int(contours_vert.vert_Lcontour.size());
	glColor3f(0.0f, 0.0f, 0.0f);
	if (n > 0) {

		// Right panels
		glBegin(GL_QUAD_STRIP);

		glVertex3f(contours_vert.vert_Rcontour.at(0)[0] + panel_width, contours_vert.vert_Rcontour.at(0)[1] - panel_height_extra, contours_vert.vert_Rcontour.at(0)[2]); //0
		glVertex3f(contours_vert.vert_Rcontour.at(0)[0], contours_vert.vert_Rcontour.at(0)[1] - panel_height_extra, contours_vert.vert_Rcontour.at(0)[2]); //1

		for (int i = 0; i < n; i++)
		{
			glVertex3f(contours_vert.vert_Rcontour.at(i)[0] + panel_width, contours_vert.vert_Rcontour.at(i)[1], contours_vert.vert_Rcontour.at(i)[2]); //0
			glVertex3f(contours_vert.vert_Rcontour.at(i)[0], contours_vert.vert_Rcontour.at(i)[1], contours_vert.vert_Rcontour.at(i)[2]); //1

		}

		glVertex3f(contours_vert.vert_Rcontour.at(n - 1)[0] + panel_width, contours_vert.vert_Rcontour.at(n - 1)[1] + panel_height_extra, contours_vert.vert_Rcontour.at(n - 1)[2]); //0
		glVertex3f(contours_vert.vert_Rcontour.at(n - 1)[0], contours_vert.vert_Rcontour.at(n - 1)[1] + panel_height_extra, contours_vert.vert_Rcontour.at(n - 1)[2]); //1

		glEnd();

		// Left panels
		glBegin(GL_QUAD_STRIP);

		glVertex3f(contours_vert.vert_Lcontour.at(0)[0], contours_vert.vert_Lcontour.at(0)[1] - panel_height_extra, contours_vert.vert_Lcontour.at(0)[2]); //0
		glVertex3f(contours_vert.vert_Lcontour.at(0)[0] - panel_width, contours_vert.vert_Lcontour.at(0)[1] - panel_height_extra, contours_vert.vert_Lcontour.at(0)[2]); //1

		for (int i = 0; i < n; i++)
		{
			glVertex3f(contours_vert.vert_Lcontour.at(i)[0], contours_vert.vert_Lcontour.at(i)[1], contours_vert.vert_Lcontour.at(i)[2]); //0
			glVertex3f(contours_vert.vert_Lcontour.at(i)[0] - panel_width, contours_vert.vert_Lcontour.at(i)[1], contours_vert.vert_Lcontour.at(i)[2]); //1

		}

		glVertex3f(contours_vert.vert_Lcontour.at(n - 1)[0], contours_vert.vert_Lcontour.at(n - 1)[1] + panel_height_extra, contours_vert.vert_Lcontour.at(n - 1)[2]); //0
		glVertex3f(contours_vert.vert_Lcontour.at(n - 1)[0] - panel_width, contours_vert.vert_Lcontour.at(n - 1)[1] + panel_height_extra, contours_vert.vert_Lcontour.at(n - 1)[2]); //1

		glEnd();
	}

}


void drawFingersOcclusion() {


	if (!allVisibleIndex) {
		glColor3f(0.0f, 0.6f, 0.0f);
		glPushMatrix();
		glLoadIdentity();
		glTranslated(0, 60, display_distance - 3);
		double cross_length = 4;
		glBegin(GL_LINES);
		glVertex3d(-cross_length / 2, cross_length / 2, 0);
		glVertex3d(cross_length / 2, -cross_length / 2, 0);
		glVertex3d(cross_length / 2, cross_length / 2., 0);
		glVertex3d(-cross_length / 2, -cross_length / 2., 0);
		glEnd();

		glPopMatrix();
	}


	if (!allVisibleThumb) {
		glColor3f(0.0f, 0.6f, 0.0f);
		glPushMatrix();
		glLoadIdentity();
		glTranslated(0, 60, display_distance - 3);
		double T_length = 4;
		glBegin(GL_LINES);
		glVertex3d(-T_length / 2, T_length / 2, 0);
		glVertex3d(T_length / 2, T_length / 2, 0);
		glVertex3d(0, T_length / 2., 0);
		glVertex3d(0, -T_length / 2., 0);
		glEnd();

		glPopMatrix();
	}
}




void drawFixation(double displayDist) {
	// draws a small fixation cross at the center of the display
	glEnable(GL_LINE_SMOOTH);
	glColor3f(1.0f, 0.0f, 0.0f);
	glLineWidth(2.f);

	glPushMatrix();
	glLoadIdentity();
	glTranslated(0, 0, displayDist);
	double cross_length = 5;
	glBegin(GL_LINES);
	glVertex3d(cross_length / 2, 0, 0);
	glVertex3d(-cross_length / 2, 0, 0);
	glVertex3d(0, -cross_length / 2., 0);
	glVertex3d(0, cross_length / 2., 0);
	glEnd();
	glPopMatrix();
	glDisable(GL_LINE_SMOOTH);
}

void drawProgressBar() {

	glPushMatrix();
	glLoadIdentity();
	glTranslated(0, 0, display_distance);

	glColor3f(0.2, 0.2, 0.2);
	glBegin(GL_LINE_LOOP);
	glVertex3f(-50, 5, 0);
	glVertex3f(50, 5, 0);
	glVertex3f(50, -5, 0);
	glVertex3f(-50, -5, 0);
	glEnd();

	glColor3f(0.1, 0.3, 0.1);
	glBegin(GL_POLYGON);
	glVertex3f(-50, 5, 0);
	glVertex3f(-50 + percentComplete, 5, 0);
	glVertex3f(-50 + percentComplete, -5, 0);
	glVertex3f(-50, -5, 0);
	glEnd();
	glPopMatrix();
}


void drawStimulus()
{
	//enum Stages {stimulus_preview, prep_trial, trial_fixate_first, trial_present_first,
	//trial_fixate_second, trial_present_second, trial_respond, break_time, exp_completed};

	//draw_objEdge();


	switch (current_stage) {

	case stimulus_preview:
		// testing texture, build vertices for curved surface
		drawSurface(dist_toEye, my_verts_static, my_contour_data);
		if (!testVisualStimuliOnly)
			drawFingersOcclusion();
		break;

	case trial_fixate:

		drawFixation(display_distance_jittered);

		break;

	case trial_viewStatic:
	case trial_MSE_first:
	case trial_MSE_reset_first:
	case trial_MSE_second:
	case trial_MSE_reset_second:
		drawSurface(dist_toEye, my_verts_static, my_contour_data);
		if (!testVisualStimuliOnly)
			drawFingersOcclusion();
		break;

	case trial_viewMtFlow:

		drawSurface(dist_toEye, my_verts_moving, my_contour_data);

		break;

	case break_time:
		drawProgressBar();
		break;

	case stimulus_previewMotion:
		drawSurface(dist_toEye, my_verts_moving, my_contour_data);
		break;
	}
}

//Central function for projecting image onto the screen
void drawGLScene()
{
	glDrawBuffer(GL_BACK_RIGHT);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.0, 0.0, 0.0, 1.0);
	cam.setEye(eyeRight);
	//cam.setEye(eyeMiddle);


	drawStimulus();
	drawInfo();
	drawTaskGuide();


	// Draw right eye view
	glDrawBuffer(GL_BACK_LEFT);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.0, 0.0, 0.0, 1.0);
	cam.setEye(eyeLeft);
	//cam.setEye(eyeMiddle);


	drawStimulus();
	drawInfo();
	drawTaskGuide();


	glutSwapBuffers();
	glutPostRedisplay();
}

void update(int value)
{
	glutPostRedisplay();
	glutTimerFunc(TIMER_MS, update, 0);
}


void drawInfo_alignment(GLText curText) {

	curText.draw(" ");
	curText.draw(" ");
	curText.draw(" ");
	if (abs(mirrorAlignment - 45.0) < 0.2)
		glColor3fv(glGreen);
	else
		glColor3fv(glRed);
	curText.draw("# Mirror Alignment = " + stringify<double>(mirrorAlignment));

	// check if monitor is calibrated
	if (screenAlignmentY < 89.0)
		glColor3fv(glRed);
	else
		glColor3fv(glGreen);
	curText.draw("# Screen Alignment Y = " + stringify<double>(screenAlignmentY));
	if (abs(screenAlignmentZ) < 89.0)
		glColor3fv(glRed);
	else
		glColor3fv(glGreen);
	curText.draw("# Screen Alignment Z = " + stringify<double>(screenAlignmentZ));
}

void drawInfo_calibrationMarkers(GLText curText) {

	curText.draw(" ");
	curText.draw(" ");

	if (isVisible(markers[calibration_T].p))
		glColor3fv(glGreen);
	else
		glColor3fv(glRed);
	curText.draw("Thumb Calibration Point " + stringify< Eigen::Matrix<double, 1, 3> >(markers[calibration_T].p.transpose()));

	if (isVisible(markers[calibration_I].p))
		glColor3fv(glGreen);
	else
		glColor3fv(glRed);
	curText.draw("Index Calibration Point " + stringify< Eigen::Matrix<double, 1, 3> >(markers[calibration_I].p.transpose()));

	curText.draw(" ");

	if (allVisibleThumb)
		glColor3fv(glGreen);
	else
		glColor3fv(glRed);
	curText.draw("thumb");


	if (allVisibleIndex)
		glColor3fv(glGreen);
	else
		glColor3fv(glRed);
	curText.draw("index");

}


void drawInfo_fingers(GLText curText) {
	curText.draw(" ");
	curText.draw(" ");
	curText.draw("--------------------");
	if (allVisibleIndex)
		glColor3fv(glGreen);
	else
		glColor3fv(glRed);
	curText.draw("Index= " + stringify< Eigen::Matrix<double, 1, 3> >(ind.transpose()));
	if (allVisibleThumb)
		glColor3fv(glGreen);
	else
		glColor3fv(glRed);
	curText.draw("Thumb= " + stringify< Eigen::Matrix<double, 1, 3> >(thm.transpose()));

	glColor3fv(glRed);
	curText.draw("--------------------");
	curText.draw(" ");
	curText.draw("# dist thm <-> home = " + stringify<double>(dist_thm_home));
	curText.draw("# handNearHome = " + stringify<bool>(handNearHome));
	curText.draw(" ");

	curText.draw(" ");
	curText.draw("# Vel thresh = " + stringify<double>(thresholdVelthm_steady));
	curText.draw("# handSteady = " + stringify<bool>(handSteady));

	curText.draw(" ");
	curText.draw("# GA = " + stringify<double>(grip_aperture));
	curText.draw("# gripSmall = " + stringify<bool>(gripSmall));

	curText.draw(" ");
	curText.draw("# Vel GA tresh = " + stringify<double>(thresholdVelGA_steady));
	curText.draw("# gripSteady = " + stringify<bool>(gripSteady));
	curText.draw(" ");

}


void drawTaskGuide() {
	if (task_guide_info) {
		glDisable(GL_COLOR_MATERIAL);
		glDisable(GL_BLEND);
		GLText text;
		if (gameMode)
			text.init(SCREEN_WIDTH, SCREEN_HEIGHT, glWhite, GLUT_BITMAP_HELVETICA_18);
		else
			text.init(640, 480, glWhite, GLUT_BITMAP_HELVETICA_12);


		text.enterTextInputMode();

		switch (current_stage) {

		case trial_viewStatic:
			glColor3fv(glRed);
			if (!gripSmall)
				text.draw("                                                                           T --> <-- X");
			if (!handNearHome)
				text.draw("                                                                           | | > H < | |");
			if (attemped_MSE && (!gripSteady) || (!handSteady) )
				text.draw("                                                                           Hold ...");

			break;
		case trial_MSE_first:
		case trial_MSE_second:
		{
			glColor3fv(glWhite);
			text.draw("                                                                           ESTIMATE:");
			text.draw("                                                                           Press + to enter");
			glColor3fv(glRed);

			if (attemped_MSE && (!gripSteady) || (!handSteady))
				text.draw("                                                                           Hold ...");
		}
		break;

		case trial_MSE_reset_first:
		{
			glColor3fv(glWhite);
			text.draw("                                                                           RESET:");
			glColor3fv(glRed);
			if (!gripSmall)
				text.draw("                                                                           T --> <-- X");
			if (attemped_MSE && (!gripSteady) || (!handSteady))
				text.draw("                                                                           Hold ...");
		}
		break;


		case trial_MSE_reset_second:
		{

			glColor3fv(glWhite);
			text.draw("                                                                           RESET & Home:");

			glColor3fv(glRed);
			if (!gripSmall)
				text.draw("                                                                           T --> <-- X");
			if (!handNearHome)
				text.draw("                                                                           | | > H < | |");
			if (attemped_MSE && (!gripSteady) || (!handSteady))
				text.draw("                                                                           Hold ...");

		}
		break;


		}
		text.leaveTextInputMode();
		glEnable(GL_COLOR_MATERIAL);
		glEnable(GL_BLEND);
	}
}

void drawInfo()
{
	// displays relevant information to the screen
	if (visibleInfo)
	{
		glDisable(GL_COLOR_MATERIAL);
		glDisable(GL_BLEND);
		GLText text;
		if (gameMode)
			text.init(SCREEN_WIDTH, SCREEN_HEIGHT, glWhite, GLUT_BITMAP_HELVETICA_18);
		else
			text.init(640, 480, glWhite, GLUT_BITMAP_HELVETICA_12);


		text.enterTextInputMode();

		switch (current_stage) {

		case exp_initializing:

			glColor3fv(glWhite);

			switch (currentInitStep) {

			case to_GetCalibrationPoints:

				text.draw("Press F to record calibration point positions.");
				//drawInfo_alignment(text);
				drawInfo_calibrationMarkers(text);

				break;

			case to_CalibrateFingerTips:
				text.draw("Press F with index and thumb TIPS on calibration points.");
				drawInfo_calibrationMarkers(text);
				break;

			case to_CalibrateFingerJoints:
				text.draw("Press F with index and thumb JOINTS on calibration points.");
				drawInfo_calibrationMarkers(text);
				break;

			case to_MarkHomePos:
				text.draw("Press F with hand at home position.");
				drawInfo_calibrationMarkers(text);
				break;

			case to_MoveApparatus:
				text.draw("Press F to set apparatus in place.");
				drawInfo_calibrationMarkers(text);
				break;

			case to_confirmReady:
				text.draw("Press F to begin!");
				//drawInfo_fingers(text);
				drawInfo_alignment(text);
				//drawInfo_calibrationMarkers(text);
				break;
			}

			break;

		case stimulus_preview:
		case stimulus_previewMotion:
			glColor3fv(glWhite);
			text.draw("Welcome! press + to start training");
			text.draw("# Name: " + subjectName);
			text.draw("# IOD: " + stringify<double>(interoculardistance));
			glColor3fv(glRed);
			if (!trainingCueIsRandom) {
				if (reinforce_texture_disparity) {
					text.draw("----------------------------- P ----------");
				}
				else {
					text.draw("--------- E ------------------------------");
				}
			}
			//text.draw("# move cnt: " + stringify<int>(move_cnt));
			//text.draw("#reinforce_texture_disparity: " + stringify<bool>(reinforce_texture_disparity));
			text.draw("# depth texture: " + stringify<double>(depth_text));
			text.draw("# depth stereo: " + stringify<double>(depth_disp));
			//text.draw("# elasped time: " + stringify<double>(ElapsedTime));

			break;

		case trial_fixate:
		case trial_viewStatic:
		case trial_MSE_first:
		case trial_MSE_reset_first:
		case trial_viewMtFlow:
		case trial_MSE_second:
		case trial_MSE_reset_second:


			text.draw(" ");
			text.draw("# current stage: " + stringify<int>(current_stage));
			text.draw("# trial Num: " + stringify<int>(trialNum));

			//text.draw("# depth texture: " + stringify<double>(depth_text));
			//text.draw("# depth stereo: " + stringify<double>(depth_disp));
			text.draw("# elasped time: " + stringify<double>(ElapsedTime));
			text.draw("# thm to home: " + stringify<double>(dist_thm_home));
			text.draw("# GA: " + stringify<double>(grip_aperture));


			break;

		case break_time:

			switch (current_error_state) {
			case no_error:
				glColor3fv(glRed);
				text.draw("Break time! Press + to continue");
				break;

			case training_exceeded:
				glColor3fv(glWhite);
				text.draw("Pls call the experimenter!!!!!!!!!!!!! training exceed");
				break;
			}
			break;

		case exp_completed:
			glColor3fv(glWhite);
			text.draw("The experiment is over. Thank you! :)");
			break;
		}
		text.leaveTextInputMode();
		glEnable(GL_COLOR_MATERIAL);
		glEnable(GL_BLEND);
	}
}


// Funzione che gestisce il ridimensionamento della finestra
void handleResize(int w, int h)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glViewport(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

}




void onlineTrial() {
	//stimulus_preview, prep_trial, trial_fixate_first, trial_present_first, trial_fixate_second, trial_present_second, trial_respond,

	switch (current_stage) {

	case stimulus_previewMotion:
		// monitor the time and update vertices
		ElapsedTime = trial_timer.getElapsedTimeInMilliSec();
		if ((move_cnt * speed_moderator) / (4.0 * nr_mvpts_max) > 2 * mv_num) {
			//if ((ElapsedTime - timestamp_mtFlow) > motionFlowTime) {		
			current_stage = stimulus_preview;
		}
		else {
			if (ElapsedTime - last_time > updateEveryMs) {
				last_time = ElapsedTime;
				move_cnt++;
				updateVerticesData(move_cnt);
			}
		}
		break;

	case trial_fixate:

		if (ElapsedTime > fixateTime) {

			current_stage = trial_viewStatic;
		}
		break;

	case trial_viewStatic:

		if (testVisualStimuliOnly) {

			if (ElapsedTime > 2 * fixateTime) {
				beepOk(21);
				grip_aperture_MSE_first = 40;
				initMotionFlow();
				current_stage = trial_viewMtFlow;
			}
		}
		else {
			handIsReset = handSteady && gripSmall && gripSteady && handNearHome;
			// ready for MSE?
			if ((ElapsedTime > 2 * fixateTime) && handIsReset) {
				beepOk(21);
				timestamp_MSEstart1 = ElapsedTime;
				current_stage = trial_MSE_first;
			}
		}
		break;


	case trial_MSE_reset_first:
		handIsReset = handSteady && gripSmall && gripSteady && handNearHome;
		if (handIsReset) {
			attemped_MSE = false;
			initMotionFlow();
			current_stage = trial_viewMtFlow;
		}
		break;


	case trial_viewMtFlow:
		// monitor the time and update vertices
		ElapsedTime = trial_timer.getElapsedTimeInMilliSec();
		if ((move_cnt * speed_moderator) / (4.0 * nr_mvpts_max) > mv_num) {
			//if ((ElapsedTime - timestamp_mtFlow) > motionFlowTime) {
			motionFlowTime = ElapsedTime - timestamp_mtFlow;
			beepOk(21);
			current_stage = trial_MSE_second;
		}
		else {
			if (ElapsedTime - last_time > updateEveryMs) {
				last_time = ElapsedTime;
				move_cnt++;
				updateVerticesData(move_cnt);
			}
		}
		break;


	case trial_MSE_reset_second:

		if (testVisualStimuliOnly) {

			advanceTrial();
		}
		else {
			handIsReset = handSteady && gripSmall && gripSteady && handNearHome;
			if (handIsReset) {
				holdCount_home++;
			}

			if (holdCount_home > 20) {
				advanceTrial();
			}
		}
		break;


	}

}


void advanceTrial()
{
	responseFile << fixed <<
		subjectName << "\t" <<
		interoculardistance << "\t" <<
		blkNum << "\t" <<
		trialNum << "\t" <<
		depth_mean << "\t" <<
		depth_delta << "\t" <<
		depth_text << "\t" <<
		depth_disp << "\t" <<
		reinforce_texture_disparity << "\t" <<
		display_distance_jittered << "\t" <<
		grip_aperture_MSE_first << "\t" <<
		grip_aperture_MSE_second << "\t" <<
		timestamp_MSEend1 - timestamp_MSEstart1 << "\t" <<
		timestamp_MSEend2 - timestamp_MSEstart2 << "\t" <<
		thm_mse1.transpose() << "\t" <<
		ind_mse1.transpose() << "\t" <<
		thm_mse2.transpose() << "\t" <<
		ind_mse2.transpose() << "\t" <<
		calibrationNum << "\t" <<
		visual_angle << "\t" <<
		stimulus_height << "\t" <<
		stimulus_visiblewidth << "\t" <<
		Tex_dot_density << "\t" <<
		Tex_dot_radius << "\t" <<
		Tex_dot_separation_ratio << "\t" <<
		speed_moderator << "\t" <<
		rock_movement_divider << "\t" <<
		motionFlowTime << "\t" <<
		trainingCueIsRandom
		<< endl;

	//subjName\tIOD\tblockN\ttrialN\tdisplayDistance\tvisualAngle\tclyHorizontal\ttexnum\ttextNomralizer\ttestDepth\tprobeStart\tprobeDepth\ttime
	if (training) {
		if (trialNum < trainNum_cap) {
			beepOk(4);
			trialNum++;
			initTrial();
		}
		else {
			beepOk(24);
			trialNum++;
			current_error_state = training_exceeded;
			current_stage = break_time;
			visibleInfo = true;
		}

	}
	else {

		//trial.next(respond_cmp_deeper);	

		if (!trial.isEmpty() && (trialNum < trialNum_max)) {

			if (trialNum % 24 == 0) {
				beepOk(4);
				percentComplete = trialNum / (totalTrNum / 100.0);
				trial.next();
				current_stage = break_time;
				visibleInfo = true;

			}
			else {
				beepOk(4);
				trialNum++;
				trial.next();
				initTrial();
			}

		}
		else {
			beepOk(1);
			responseFile.close();
			visibleInfo = true;
			current_stage = exp_completed;
		}

	}


}



void handleKeypress(unsigned char key, int x, int y)
{
	switch (key) { // key presses that work regardless of the stage

	case 27:	//corrisponde al tasto ESC

		if (resetScreen_betweenRuns) {
			homeEverything(5000, 4500);
			shutdown();
		}
		else {
			shutdown();
		}

		break;


	case 'b': {
		initMotionFlow();
		trial_timer.reset();
		trial_timer.start();
		last_time = 0;
		ElapsedTime = 0;
		current_stage = stimulus_previewMotion;
	}
			break;

	case 'c':
		reinforce_texture_disparity = !reinforce_texture_disparity;
		initSurface();
		break;
	case 'a':
		grip_aperture_MSE_first = 999;
		advanceTrial();
		break;

	case 'i':
		visibleInfo = !visibleInfo;
		break;


	case '1':

		if (current_stage == stimulus_preview) {
			if (depth_text > depth_inc)
				depth_text = depth_text - depth_inc;

			initSurface();

		}

		break;

	case '2':

		if (current_stage == stimulus_preview) {
			depth_text = depth_text + depth_inc;

			initSurface();
		}

		break;

	case 'R':
	case 'r':

		switch (current_error_state) {
		case no_error:
			initTrial();
			break;
		}

		break;




	case 'f':
	case 'F':
	{
		switch (currentInitStep)
		{
		case to_GetCalibrationPoints:
		case to_CalibrateFingerTips:
		case to_CalibrateFingerJoints:

			calibrate_fingers();

			break;

		case to_MoveApparatus:
		case to_MarkHomePos:
		case to_confirmReady:

			calibrate_system();

			break;
		}
	}
	break;

	case 'q':
		Fingers_Calibrated = false;
		currentInitStep = to_GetCalibrationPoints;
		current_stage = exp_initializing;
		visibleInfo = true;
		break;

	case '+':
		switch (current_stage) {
		case stimulus_preview:

			beepOk(5);
			visibleInfo = false;
			initBlock();
			initTrial();
/*		
			initMotionFlow();
			trial_timer.reset();
			trial_timer.start();
			last_time = 0;
			ElapsedTime = 0;
			current_stage = stimulus_previewMotion;
*/	
			break;

		case trial_MSE_first:

			attemped_MSE = true;

			if ((grip_aperture < 200) && gripSteady) {

				beepOk(4);
				grip_aperture_MSE_first = grip_aperture;
				ind_mse1 = ind;
				thm_mse1 = thm;
				timestamp_MSEend1 = ElapsedTime;
				current_stage = trial_MSE_reset_first;
			}

			break;

		case trial_MSE_second:

			if (testVisualStimuliOnly) {
				beepOk(4);
				grip_aperture_MSE_second = 120;
				timestamp_MSEend2 = ElapsedTime;
				current_stage = trial_MSE_reset_second;
			}
			else {
				attemped_MSE = true;

				if ((grip_aperture < 200) && gripSteady) {

					beepOk(4);
					grip_aperture_MSE_second = grip_aperture;
					ind_mse2 = ind;
					thm_mse2 = thm;
					timestamp_MSEend2 = ElapsedTime;
					current_stage = trial_MSE_reset_second;
				}
			}

			break;

		case break_time:
			if (testVisualStimuliOnly) {
				beepOk(5);
				trialNum++;
				initTrial();
			}
			else {
				if (abs(mirrorAlignment - 45.0) < 0.2) {
					beepOk(5);
					visibleInfo = false;
					trialNum++;
					initTrial();
				}
				else {
					beepOk(20);
				}
			}
			break;
		}
		break;

	case 'T':
	case 't':
		if (training && (current_stage != stimulus_preview)) {
			training = false;
			task_guide_info = false;
			beepOk(6);
			trialNum = 0;

			if (current_error_state == training_exceeded) {
				current_error_state = no_error;
			}

			visibleInfo = true;
			current_stage = break_time;
		}
		break;

	case '4':

		if (current_stage == stimulus_preview) {

			if (depth_disp > depth_inc)
				depth_disp = depth_disp - depth_inc;


			initSurface();

		}
		else if (current_stage == trial_MSE_second) {
			initMotionFlow();
			current_stage = trial_viewMtFlow;
		}

		break;

	case '5':

		if (current_stage == stimulus_preview) {

			depth_disp = depth_disp + depth_inc;

			initSurface();

		}

		break;

	case 'h':
		task_guide_info = true;
		break;


	}

}

/***** SOUNDS *****/
void beepOk(int tone)
{

	switch (tone)
	{

	case 1: //high pitch beep
		PlaySound((LPCSTR)"C:\\cncsvision\\data\\beep\\beep-8_lowpass.wav", NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 4: //mellow and good for trials
		PlaySound((LPCSTR)"C:\\cncsvision\\data\\beep\\beep-440-pluck.wav", NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 2: //reject like buzz
		PlaySound((LPCSTR)"C:\\cncsvision\\data\\beep\\beep-10.wav", NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 3: //reject short
		PlaySound((LPCSTR)"C:\\cncsvision\\data\\beep\\beep-reject.wav", NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 5: //"go"
		PlaySound((LPCSTR)"C:\\cncsvision\\data\\beep\\spoken-go.wav", NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 6: //mellow and good for trials
		PlaySound((LPCSTR)"C:\\cncsvision\\data\\beep\\beep-lowBubblePop.wav", NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 8: //spoken mirror
		PlaySound((LPCSTR)"C:\\cncsvision\\data\\beep\\spoken-mirror.wav", NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 9: //mellow and good for trials
		PlaySound((LPCSTR)"C:\\cncsvision\\data\\beep\\beep-highBubblePop.wav", NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 15:
		PlaySound((LPCSTR)"C:\\cncsvision\\data\\beep\\beep-rising.wav",
			NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 16:
		PlaySound((LPCSTR)"C:\\cncsvision\\data\\beep\\beep-falling.wav",
			NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 17:
		PlaySound((LPCSTR)"C:\\cncsvision\\data\\beep\\beep-440-pluck-5below.wav",
			NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 18: // light click
		PlaySound((LPCSTR)"C:\\cncsvision\\data\\beep\\beep-click3MS.wav",
			NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 20: // alighnment
		PlaySound((LPCSTR)"C:\\cncsvision\\data\\beep\\spoken-alignment.wav",
			NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 21: // estimate
		PlaySound((LPCSTR)"C:\\cncsvision\\data\\beep\\spoken-estimate-short.wav",
			NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 22: // grasp
		PlaySound((LPCSTR)"C:\\cncsvision\\data\\beep\\spoken-grasp-short.wav",
			NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 23: // home
		PlaySound((LPCSTR)"C:\\cncsvision\\data\\beep\\spoken-home.wav",
			NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 24: // help
		PlaySound((LPCSTR)"C:\\cncsvision\\data\\beep\\spoken-help.wav",
			NULL, SND_FILENAME | SND_ASYNC);
		break;
	}
	return;
}



double mapYtoL(const CurveYLMap& inputYLMap, double input_y) {
	double mapStpSz = inputYLMap.step_size;
	double i_c_val = (input_y - inputYLMap.y_vec[0]) / mapStpSz;
	int i_c = round(i_c_val);
	int i_c0 = floor(i_c_val);
	int i_c1 = ceil(i_c_val);
	double l = inputYLMap.l_vec[i_c0] + (input_y - inputYLMap.y_vec[i_c0]) * (inputYLMap.l_vec[i_c1] - inputYLMap.l_vec[i_c0]) / mapStpSz;
	return l;
}

void scanCurve(double shapeHeight, double shapeDepth, CurveYLMap& output_curve_ylmap) {
	output_curve_ylmap = {};

	double y, z, l, y_prev, z_prev;
	double step_size = (shapeHeight / (nr_curve_map - 1));

	output_curve_ylmap.curve_depth = shapeDepth;
	output_curve_ylmap.curve_height = shapeHeight;
	output_curve_ylmap.step_size = step_size;

	// the first point
	y = -shapeHeight / 2;
	z = 0;
	l = 0;
	y_prev = y, z_prev = z;
	output_curve_ylmap.y_vec.push_back(y);
	output_curve_ylmap.l_vec.push_back(l);


	for (int j = 1; j < nr_curve_map; j++) {
		y = -shapeHeight / 2 + j * step_size;
		z = getZ(shapeHeight, shapeDepth, y);
		l = l + sqrt(pow(y - y_prev, 2) + pow(z - z_prev, 2));

		output_curve_ylmap.y_vec.push_back(y);
		output_curve_ylmap.l_vec.push_back(l);

		y_prev = y; z_prev = z;
	}


}

void generateTexDots(float TM_X, float TM_Y, float dotDensity, float dotRadius, float dotSeparationRatio, TextureDotsData& outputTexDots) {

	outputTexDots = {};

	outputTexDots.TexMapSize = Vec2{ TM_X, TM_Y };
	outputTexDots.Radius = dotRadius;

	std::uniform_real_distribution<float> dist(0.f, 1.f);

	// step 1: generate dots
	vector<Vec2> dc_vec;

	int num_dot = TM_X * TM_Y * dotDensity;

	float dot_separation = dotSeparationRatio * 2 * dotRadius;

	int genTexAttemps = 0;
	int num_runs = 0;
	int nr_D_L = 0;
	int nr_D_H = 0;

	for (int i_dot = 0; i_dot < num_dot; i_dot++) {

		num_runs++;

		if (num_runs > 10000) {

			genTexAttemps++;

			if (genTexAttemps > 4) {

				cout << "text depth: " << depth_text << endl;
				cout << "disp depth: " << depth_disp << endl;
				cerr << "can't generate texture" << endl;
				exit(0);
			}
			else {
				num_runs = 0;
				dc_vec.clear();
				i_dot = 0;
			}
		}

		// pick random xy values for the circle center
		float cx = dist(rng); // 0-1
		float cy = dist(rng); // 0-1
		cx *= TM_X;
		cy *= TM_Y;

		// checking whether the current circle intersects withe the previous circles already pushed
		bool intersect = false;
		for (int k = 0; k < dc_vec.size(); k++) {
			Vec2 prev_c = dc_vec[k];
			if ((cx - prev_c.x) * (cx - prev_c.x) + (cy - prev_c.y) * (cy - prev_c.y) < dot_separation * dot_separation) {
				intersect = true;
				break;
			}
		}

		// if intersect, then break and pick a new circle center
		if (intersect) {
			i_dot--;
			continue;
		}

		// if not intersect, add this circle to the circles vector
		dc_vec.push_back(Vec2{ cx, cy });

		if (cy < dotSeparationRatio * dotRadius) {

			dc_vec.push_back(Vec2{ cx, cy + TM_Y });
			nr_D_L++;
		}
		if (cy > (TM_Y - dotSeparationRatio * dotRadius)) {
			dc_vec.push_back(Vec2{ cx, cy - TM_Y });
			nr_D_H++;
		}

	}

	// sort the dot by their y
	int n_dots = dc_vec.size();

	vector<float> dc_y_vec;
	for (int k = 0; k < n_dots; k++) {
		dc_y_vec.push_back(dc_vec[k].y);
	}
	vector<int> sortedDC_ind_vec(n_dots);
	std::iota(sortedDC_ind_vec.begin(), sortedDC_ind_vec.end(), 0); //Initializing
	std::sort(sortedDC_ind_vec.begin(), sortedDC_ind_vec.end(), [&](int i, int j) {return dc_y_vec[i] < dc_y_vec[j]; });

	// fill in TexDots dot center vectors in order of low y to high y

	for (int k = 0; k < n_dots; k++) {
		outputTexDots.dot_center_vec.push_back(dc_vec[sortedDC_ind_vec[k]]);
	}

	outputTexDots.nr_D_H = nr_D_H;
	outputTexDots.nr_D_L = nr_D_L;
	outputTexDots.nr_S = num_dot - nr_D_H - nr_D_L;
	outputTexDots.Radius_y_max = dotSeparationRatio * dotRadius;

}


void sampleTexDotsResize(const CurveYLMap& CurveYLMap_origin, const CurveYLMap& CurveYLMap_proj, double distShapeToEye, ProjTexDots_ResizeMap& output_RszMap) {

	output_RszMap = {};

	double depth_origin = CurveYLMap_origin.curve_depth;
	output_RszMap.depth_origin = depth_origin;
	double depth = CurveYLMap_proj.curve_depth;
	output_RszMap.depth_proj = depth;

	double height = CurveYLMap_proj.curve_height;
	double ylmap_stpsz = CurveYLMap_origin.step_size;
	double y_min = CurveYLMap_origin.y_vec.front();

	double l_half = CurveYLMap_proj.l_vec.back() / 2.;
	double del_l = 0.2;
	int nr_half = l_half / del_l;
	double del_l_adjust = l_half / nr_half;
	double del_l_twice = 2 * del_l_adjust;

	int i_start = 0;
	double l_o_prev = 0;
	double RRsz_y_max = 0;
	// k = 0
	output_RszMap.l_vec.push_back(0);
	output_RszMap.R_resize_vec.push_back(Vec2{ 1, 0 }); // y is a filler

	// k = 1
	double l = 1 * del_l_adjust;
	double y = mapLtoY(CurveYLMap_proj, l, i_start);
	i_start = (y - y_min) / ylmap_stpsz;
	double z = getZ(height, depth, y);
	double z_o = SolveForZ_projected(height, depth_origin, distShapeToEye, y, z);
	double w = (z_o - distShapeToEye) / (z - distShapeToEye);
	double y_o = w * y;
	double l_o = mapYtoL(CurveYLMap_origin, y_o);


	for (int k = 1; k < nr_half + 1; k++) {

		double l_next = (k + 1) * del_l_adjust;
		double y_next = mapLtoY(CurveYLMap_proj, l_next, i_start);
		i_start = (y_next - y_min) / ylmap_stpsz;
		double z_next = getZ(height, depth, y_next);
		double z_o_next = SolveForZ_projected(height, depth_origin, distShapeToEye, y_next, z_next);
		double w_next = (z_o_next - distShapeToEye) / (z_next - distShapeToEye);
		double y_o_next = w_next * y_next;
		double l_o_next = mapYtoL(CurveYLMap_origin, y_o_next);

		double RRsz_y = del_l_twice / (l_o_next - l_o_prev);

		output_RszMap.l_vec.push_back(l);
		output_RszMap.R_resize_vec.push_back(Vec2{ (float)(1 / w), (float)RRsz_y });

		l_o_prev = l_o;
		l_o = l_o_next;
		l = l_next;
		w = w_next;

		if (RRsz_y > RRsz_y_max)
			RRsz_y_max = RRsz_y;
	}

	// correct the filler at 0
	output_RszMap.R_resize_vec[0].y = output_RszMap.R_resize_vec[1].y;

	// copy map to upper half
	for (int k = (nr_half + 1); k < (2 * nr_half + 1); k++) {

		l = k * del_l_adjust;
		output_RszMap.l_vec.push_back(l);
		output_RszMap.R_resize_vec.push_back(output_RszMap.R_resize_vec[2 * nr_half - k]);

	}

	output_RszMap.del_l = del_l_adjust;
	output_RszMap.R_Rsz_y_max = RRsz_y_max;

}


void projectTexDots(double distShapeToEye, const CurveYLMap& YLMap_origin, const CurveYLMap& YLMap_proj, const TextureDotsData& input_TexDots_origin, const ProjTexDots_ResizeMap& RszMap_proj, TextureDotsData& output_TexDots_proj) {

	output_TexDots_proj = input_TexDots_origin;
	output_TexDots_proj.dot_center_vec.clear();
	output_TexDots_proj.R_resize_vec.clear();

	float y_border_o = input_TexDots_origin.TexMapSize.y;
	float y_border_p = YLMap_proj.l_vec.back();
	output_TexDots_proj.TexMapSize.y = y_border_p;


	float x_offset = input_TexDots_origin.TexMapSize.x / 2.;

	double height_origin = YLMap_origin.curve_height;
	double depth_origin = YLMap_origin.curve_depth;
	double height_proj = YLMap_proj.curve_height;
	double depth_proj = YLMap_proj.curve_depth;

	int nr = input_TexDots_origin.dot_center_vec.size();
	int nr_D_L = input_TexDots_origin.nr_D_L;
	int nr_D_H = input_TexDots_origin.nr_D_H;
	int nr_S = input_TexDots_origin.nr_S;

	double step_l_RszMap = RszMap_proj.del_l;


	vector<Vec2> dc_vec_InBdr;
	vector<Vec2> RRsz_vec_InBdr;
	// these are dots that are inside the border

	for (int i_d = nr_D_H; i_d < nr - nr_D_L; i_d++) {

		Vec2 dc_o = input_TexDots_origin.dot_center_vec[i_d];

		float l_o = dc_o.y;
		float y_min = YLMap_proj.y_vec.front();

		// step 1: place the dot from TM to 3Dsurface, by finding the y first 
		// known l -> find y

		double x_o = dc_o.x - x_offset;
		double y_o = mapLtoY(YLMap_origin, l_o);

		double z_o = getZ(height_origin, depth_origin, y_o);
		//input_TexDots_origin.dc_vec.push_back(Vector3f(x_o, y_o, z_o));


		// step 2: project the dot to the new suface
		Vector3d proj_dot = projectPoint(height_proj, depth_proj, distShapeToEye, Vector3d(x_o, y_o, z_o));
		//output_TexDots_proj.dc_vec.push_back(Vector3f(proj_dot));

		double x_p = proj_dot.x();
		double y_p = proj_dot.y();
		double z_p = proj_dot.z();

		double l_p = mapYtoL(YLMap_proj, y_p);
		dc_vec_InBdr.push_back(Vec2{ (float)x_p + x_offset, (float)l_p });

		// look up Resize factors on map
		int i_c = round(l_p / step_l_RszMap);
		RRsz_vec_InBdr.push_back(RszMap_proj.R_resize_vec[i_c]);
	}

	int nr_dots_inBdr = dc_vec_InBdr.size();
	// project dots from the upper area of TM to below TexMap for periodic texture
	for (int k = (nr_dots_inBdr - nr_D_H); k < nr_dots_inBdr; k++) {
		Vec2 dc_temp = dc_vec_InBdr[k];
		output_TexDots_proj.dot_center_vec.push_back(Vec2{ dc_temp.x, dc_temp.y - y_border_p });
		output_TexDots_proj.R_resize_vec.push_back(RRsz_vec_InBdr[k]);
	}

	// area insie TM
	for (int k = 0; k < nr_dots_inBdr; k++) {
		output_TexDots_proj.dot_center_vec.push_back(dc_vec_InBdr[k]);
		output_TexDots_proj.R_resize_vec.push_back(RRsz_vec_InBdr[k]);
	}

	// project dots from the lower area of TM to above TexMap for periodic texture
	for (int k = 0; k < nr_D_L; k++) {
		Vec2 dc_temp = dc_vec_InBdr[k];
		output_TexDots_proj.dot_center_vec.push_back(Vec2{ dc_temp.x, dc_temp.y + y_border_p });
		output_TexDots_proj.R_resize_vec.push_back(RRsz_vec_InBdr[k]);
	}

	output_TexDots_proj.Radius_y_max = input_TexDots_origin.Radius_y_max * RszMap_proj.R_Rsz_y_max;

}


int buildCurve_byDelY(const CurveYLMap& input_curve_ylmap, CurvePtsData& output_curve) {

	output_curve = {};

	double depth = input_curve_ylmap.curve_depth;
	output_curve.curve_depth = depth;
	double height = input_curve_ylmap.curve_height;
	output_curve.curve_height = height;
	double y, l;

	double stpsz_J = height / (nr_points_height_default - 1);
	double stpsz_ycurve_precise = height / (nr_curve_map - 1);

	for (int j = 0; j < nr_points_height_default; j++) {
		int k = stpsz_J * j / stpsz_ycurve_precise;
		y = input_curve_ylmap.y_vec[k];
		output_curve.y_vec.push_back(y);
		output_curve.z_vec.push_back(getZ(height, depth, y));
		output_curve.l_vec.push_back(input_curve_ylmap.l_vec[k]);
	}

	return output_curve.y_vec.size();
}


int buildCurve_byDelL(const CurveYLMap& input_curve_ylmap, CurvePtsData& output_curve) {

	output_curve = {};

	double depth = input_curve_ylmap.curve_depth;
	output_curve.curve_depth = depth;
	double height = input_curve_ylmap.curve_height;
	output_curve.curve_height = height;
	double ylmap_stpsz = input_curve_ylmap.step_size;
	double y_min = input_curve_ylmap.y_vec[0];

	// get an even number of steps
	double l_total = input_curve_ylmap.l_vec.back();
	int nr_steps = (int)(l_total / del_l);
	if (nr_steps % 2 == 1)
		nr_steps++;


	double del_l_adjust = l_total / nr_steps;

	int ind_midway = nr_steps / 2;
	int i_temp = 0;

	double y, l;

	for (int k = 0; k < ind_midway; k++) {

		l = k * del_l_adjust;
		y = mapLtoY(input_curve_ylmap, l, i_temp);
		i_temp = (y - y_min) / ylmap_stpsz;
		output_curve.y_vec.push_back(y);
		output_curve.z_vec.push_back(getZ(height, depth, y));
		output_curve.l_vec.push_back(l);

	}

	l = l_total / 2.;
	y = 0;

	output_curve.y_vec.push_back(y);
	output_curve.z_vec.push_back(depth);
	output_curve.l_vec.push_back(l);

	for (int k = 1; k < ind_midway + 1; k++) {
		int i_c = ind_midway - k;
		l = l + del_l_adjust;

		output_curve.y_vec.push_back(-output_curve.y_vec[i_c]);
		output_curve.z_vec.push_back(output_curve.z_vec[i_c]);
		output_curve.l_vec.push_back(l);

	}

	output_curve.step_size = del_l_adjust;


	return output_curve.y_vec.size();
}


void projectCurve(const CurveYLMap& curve_map_proj, double distShapeToEye, const CurvePtsData& origin_curve, CurvePtsData& output_curve_proj) {

	output_curve_proj = {};

	double newDepth = curve_map_proj.curve_depth;
	double height = curve_map_proj.curve_height;
	double step_y_ylmap = curve_map_proj.step_size;

	output_curve_proj.curve_height = height;
	output_curve_proj.curve_depth = newDepth;

	double y_p, z_p, l_p, tg_p;


	for (int jj = 0; jj < origin_curve.y_vec.size(); jj++) {

		double y_o = origin_curve.y_vec[jj];
		double z_o = origin_curve.z_vec[jj];
		z_p = SolveForZ_projected(height, newDepth, distShapeToEye, y_o, z_o);
		double w = (z_p - distShapeToEye) / (z_o - distShapeToEye);
		y_p = w * y_o;
		int i_c = (y_p - curve_map_proj.y_vec[0]) / step_y_ylmap;
		l_p = curve_map_proj.l_vec[i_c];

		output_curve_proj.y_vec.push_back(y_p);
		output_curve_proj.z_vec.push_back(z_p);
		output_curve_proj.l_vec.push_back(l_p);

	}

}







void buildContour(double ContourWidth, const CurvePtsData& dispYCurve, const CurvePtsData& textYCurve, float distShapeToEye, ContourData& new_contours_vert) {

	new_contours_vert = {};

	for (int i_v = 0; i_v < dispYCurve.y_vec.size(); i_v++) {

		float x_t_L = -ContourWidth / 2;
		float x_t_R = ContourWidth / 2;

		float z_t = textYCurve.z_vec[i_v];

		float y_d = dispYCurve.y_vec[i_v];
		float z_d = dispYCurve.z_vec[i_v];

		float w = (distShapeToEye - z_d) / (distShapeToEye - z_t);
		//float w = (distShapeToEye - z_d) / (distShapeToEye - (z_t + z_d) / 2.0);
		float x_v_L = w * x_t_L;
		float x_v_R = w * x_t_R;

		new_contours_vert.vert_Lcontour.push_back(Vector3f(x_v_L, y_d, z_d));
		new_contours_vert.vert_Rcontour.push_back(Vector3f(x_v_R, y_d, z_d));
	}
}


void buildAllColorsVec(const VerticesData& vertices_data, AllTimeColorsVec& colorsVecs) {

	colorsVecs.colors_vec_allTimeVec.clear();

	vector<GLfloat> temp_colors_vec;
	temp_colors_vec = vertices_data.colors_vec;
	colorsVecs.colors_vec_allTimeVec.push_back(temp_colors_vec);

	int nr_col_I = 3 * nr_points_width;
	int nr_J = temp_colors_vec.size() / nr_col_I;

	temp_colors_vec.erase(temp_colors_vec.end() - nr_col_I, temp_colors_vec.end());

	for (int i_m = 1; i_m < (nr_J - 1); i_m++) {
		rotate(temp_colors_vec.begin(), temp_colors_vec.begin() + nr_col_I, temp_colors_vec.end());
		vector<GLfloat> next_colors_vec = temp_colors_vec;
		next_colors_vec.insert(next_colors_vec.end(), temp_colors_vec.begin(), temp_colors_vec.begin() + nr_col_I);
		colorsVecs.colors_vec_allTimeVec.push_back(next_colors_vec);
	}

}


void updateVerticesData(int timeID) {

	int movementID = int(timeID * speed_moderator) % (4 * nr_mvpts_max);

	if (movementID <= nr_mvpts_max) {
		my_verts_moving.colors_vec = AllTimeColorsVec_Moving.colors_vec_allTimeVec[movementID];
	}
	else if (movementID <= 2 * nr_mvpts_max) { // 20 - 40
		my_verts_moving.colors_vec = AllTimeColorsVec_Moving.colors_vec_allTimeVec[(2 * nr_mvpts_max - movementID)];
	}
	else if (movementID <= 3 * nr_mvpts_max) { // 40 - 60
		my_verts_moving.colors_vec = AllTimeColorsVec_Moving.colors_vec_allTimeVec[(nr_points_height - 1) + (2 * nr_mvpts_max - movementID)];
	}
	else { // 60 - 80
		my_verts_moving.colors_vec = AllTimeColorsVec_Moving.colors_vec_allTimeVec[(nr_points_height - 1) - (4 * nr_mvpts_max - movementID)];
	}

}


void buildSurface_TexOnText(double shapeWidth, const CurvePtsData& dispYCurve, const CurvePtsData& textYCurve, double distShapeToEye, TextureDotsData& TexDotsOnText, VerticesData& vertices_data) {

	vertices_data = {};

	GLuint i_ind = 0;
	int nr_J = dispYCurve.y_vec.size();
	double stpsz_I = shapeWidth / (nr_points_width - 1);
	double height = textYCurve.curve_height;
	double depth_text = textYCurve.curve_depth;

	// for texture colors
	float x_offset = shapeWidth / 2;
	float vertex_col = 1.0f;
	int nr_dots = TexDotsOnText.dot_center_vec.size();
	float R = TexDotsOnText.Radius;

	int TexDot_Ind_L = 0;
	int TexDot_Ind_H = 0;
	vector<Vec2> nearTexDots_dc_vec;

	for (int jj = 0; jj < nr_J; jj++) {

		float y_d = dispYCurve.y_vec[jj];
		float z_d = dispYCurve.z_vec[jj];

		float y_t = textYCurve.y_vec[jj];
		float z_t = textYCurve.z_vec[jj];

		float tg_t = getTg(height, depth_text, y_t);

		float w = (distShapeToEye - z_d) / (distShapeToEye - z_t);
		float x_d;

		double l_t = textYCurve.l_vec[jj];

		// find the dots that are near l_t
		while ((TexDotsOnText.dot_center_vec[TexDot_Ind_L].y < ((float)l_t - 1.5 * R)) && TexDot_Ind_L < (nr_dots - 1)) {
			TexDot_Ind_L++;
		}


		TexDot_Ind_H = TexDot_Ind_L;
		while ((TexDotsOnText.dot_center_vec[TexDot_Ind_H].y < ((float)l_t + 1.5 * R)) && TexDot_Ind_H < (nr_dots - 1)) {
			TexDot_Ind_H++;
		}

		nearTexDots_dc_vec.clear();
		for (int k = TexDot_Ind_L; k < TexDot_Ind_H + 1; k++) {
			nearTexDots_dc_vec.push_back(TexDotsOnText.dot_center_vec[k]);
		}
		int nr_dots_near = nearTexDots_dc_vec.size();


		for (int ii = 0; ii < nr_points_width; ii++) {

			double pt_x = stpsz_I * ii;
			double pt_y = l_t;
			vertex_col = 1.0f;



			for (int k = 0; k < nr_dots_near; k++) {

				double pt_val = sqrt((pt_x - nearTexDots_dc_vec[k].x) * (pt_x - nearTexDots_dc_vec[k].x) +
					(pt_y - nearTexDots_dc_vec[k].y) * (pt_y - nearTexDots_dc_vec[k].y));

				if (pt_val < R_blur_fac * R) {
					double vertex_col_tentative = blurEdge(drop_off_rate, pt_val, R_blur_fac * R, 0.0, 1.0);
					vertex_col = float(vertex_col_tentative);
					break;
				}
			}

			x_d = w * (pt_x - x_offset);


			vertices_data.vertices_vec.push_back(x_d);
			vertices_data.vertices_vec.push_back(y_d);
			vertices_data.vertices_vec.push_back(z_d);

			vertices_data.light_normals_vec.push_back(0);
			vertices_data.light_normals_vec.push_back(-tg_t);
			vertices_data.light_normals_vec.push_back(1);


			vertices_data.colors_vec.push_back(vertex_col);
			vertices_data.colors_vec.push_back(0);
			vertices_data.colors_vec.push_back(0);

			if (ii < nr_points_width - 1 && jj < nr_J - 1) {

				// using vector
				vertices_data.indices_draw_triangle_vec.push_back(i_ind);
				vertices_data.indices_draw_triangle_vec.push_back(i_ind + 1);
				vertices_data.indices_draw_triangle_vec.push_back(i_ind + nr_points_width);

				vertices_data.indices_draw_triangle_vec.push_back(i_ind + nr_points_width);
				vertices_data.indices_draw_triangle_vec.push_back(i_ind + 1);
				vertices_data.indices_draw_triangle_vec.push_back(i_ind + nr_points_width + 1);

			}

			i_ind++;
		}
	}

}


void buildSurface_TexOnDisp(double shapeWidth, const CurvePtsData& dispYCurve, const CurvePtsData& textYCurve, double distShapeToEye, TextureDotsData& TexDotsOnDisp, VerticesData& vertices_data) {
	vertices_data = {};

	GLuint i_ind = 0;
	int nr_J = dispYCurve.y_vec.size();
	double stpsz_I = shapeWidth / (nr_points_width - 1);
	double height = textYCurve.curve_height;
	double depth_text = textYCurve.curve_depth;
	// for texture colors
	float x_offset = shapeWidth / 2;
	double vertex_col = 1.0f;
	int nr_dots = TexDotsOnDisp.dot_center_vec.size();
	float R = TexDotsOnDisp.Radius;
	float R_sq = R * R;
	float R_y_max = TexDotsOnDisp.Radius_y_max;

	int TexDot_Ind_L = 0;
	int TexDot_Ind_H = 0;
	vector<Vec2> nearTexDots_dc_vec;
	vector<Vec2> nearTexDots_Rsz_vec;

	for (int jj = 0; jj < nr_J; jj++) {

		float y_d = dispYCurve.y_vec[jj];
		float z_d = dispYCurve.z_vec[jj];
		float y_t = textYCurve.y_vec[jj];
		float z_t = textYCurve.z_vec[jj];
		float tg_t = getTg(height, depth_text, y_t);

		float x_d;
		double l_d = dispYCurve.l_vec[jj];

		// find the dots that are near l_d
		nearTexDots_dc_vec.clear();
		nearTexDots_Rsz_vec.clear();
		while ((TexDotsOnDisp.dot_center_vec[TexDot_Ind_L].y < ((float)l_d - R_y_max)) && TexDot_Ind_L < (nr_dots - 1)) {
			TexDot_Ind_L++;
		}
		TexDot_Ind_H = TexDot_Ind_L;
		while ((TexDotsOnDisp.dot_center_vec[TexDot_Ind_H].y < ((float)l_d + R_y_max)) && TexDot_Ind_H < (nr_dots - 1)) {
			TexDot_Ind_H++;
		}

		for (int k = TexDot_Ind_L; k < TexDot_Ind_H + 1; k++) {
			nearTexDots_dc_vec.push_back(TexDotsOnDisp.dot_center_vec[k]);
			nearTexDots_Rsz_vec.push_back(TexDotsOnDisp.R_resize_vec[k]);
		}
		int nr_dots_near = nearTexDots_dc_vec.size();


		// go through vertice from left to right
		for (int ii = 0; ii < nr_points_width; ii++) {

			double pt_x = stpsz_I * ii;
			double pt_y = l_d;
			vertex_col = 1.0f;

			for (int k = 0; k < nr_dots_near; k++) {
				double pt_val_sq = (pt_x - nearTexDots_dc_vec[k].x) * (pt_x - nearTexDots_dc_vec[k].x) / (nearTexDots_Rsz_vec[k].x * nearTexDots_Rsz_vec[k].x) +
					(pt_y - nearTexDots_dc_vec[k].y) * (pt_y - nearTexDots_dc_vec[k].y) / (nearTexDots_Rsz_vec[k].y * nearTexDots_Rsz_vec[k].y);
				double pt_val = sqrt(pt_val_sq);

				if (pt_val < R_blur_fac * R) {
					double vertex_col_tentative = blurEdge(drop_off_rate, pt_val, R_blur_fac * R, 0.0, 1.0);
					vertex_col = float(vertex_col_tentative);
					break;
				}

			}

			x_d = pt_x - x_offset;

			vertices_data.vertices_vec.push_back(x_d);
			vertices_data.vertices_vec.push_back(y_d);
			vertices_data.vertices_vec.push_back(z_d);

			vertices_data.light_normals_vec.push_back(0);
			vertices_data.light_normals_vec.push_back(-tg_t);
			vertices_data.light_normals_vec.push_back(1);


			vertices_data.colors_vec.push_back((float)vertex_col);
			vertices_data.colors_vec.push_back(0);
			vertices_data.colors_vec.push_back(0);

			if (ii < nr_points_width - 1 && jj < nr_J - 1) {

				// using vector
				vertices_data.indices_draw_triangle_vec.push_back(i_ind);
				vertices_data.indices_draw_triangle_vec.push_back(i_ind + 1);
				vertices_data.indices_draw_triangle_vec.push_back(i_ind + nr_points_width);

				vertices_data.indices_draw_triangle_vec.push_back(i_ind + nr_points_width);
				vertices_data.indices_draw_triangle_vec.push_back(i_ind + 1);
				vertices_data.indices_draw_triangle_vec.push_back(i_ind + nr_points_width + 1);

			}

			i_ind++;
		}
	}

}


void buildSurface_congruent(double shapeWidth, double shapeHeight, double shapeDepth, double distShapeToEye, double contourPanelSeparation) {

	// part 1: generate TexDots
	CurveYLMap ylMap_Text;
	scanCurve(shapeHeight, shapeDepth, ylMap_Text);
	l_curve_text = ylMap_Text.l_vec.back();
	l_curve_disp = l_curve_text;
	float l_text = l_curve_text;
	TextureDotsData Tex_Dots_text;
	generateTexDots(shapeWidth, l_text, Tex_dot_density, Tex_dot_radius, Tex_dot_separation_ratio, Tex_Dots_text);

	// part 2: static image
	CurvePtsData y_curve_data_text, y_curve_data_disp;
	int nr_points_height_static = buildCurve_byDelY(ylMap_Text, y_curve_data_text);
	y_curve_data_disp = y_curve_data_text;
	buildSurface_TexOnText(stimulus_width, y_curve_data_disp, y_curve_data_text, distShapeToEye, Tex_Dots_text, my_verts_static);
	buildContour(contourPanelSeparation, y_curve_data_disp, y_curve_data_text, distShapeToEye, my_contour_data);

	// part 3: prebuild movement
	CurvePtsData y_curve_data_text_m, y_curve_data_disp_m;
	nr_points_height = buildCurve_byDelL(ylMap_Text, y_curve_data_text_m);
	y_curve_data_disp_m = y_curve_data_text_m;
	buildSurface_TexOnText(shapeWidth, y_curve_data_disp_m, y_curve_data_text_m, distShapeToEye, Tex_Dots_text, my_verts_moving);
	buildAllColorsVec(my_verts_moving, AllTimeColorsVec_Moving);

}


void buildSurface_incongruent(double shapeWidth, double shapeHeight, double dispDepth, double textDepth, double distShapeToEye, double contourPanelSeparation) {


	// part 1: generate TexDots and project to Disp Surface
	CurveYLMap ylMap_Text;
	scanCurve(shapeHeight, textDepth, ylMap_Text);
	l_curve_text = ylMap_Text.l_vec.back();
	float l_text = l_curve_text;

	TextureDotsData Tex_Dots_text, Tex_Dots_disp;
	generateTexDots(shapeWidth, l_text, Tex_dot_density, Tex_dot_radius, Tex_dot_separation_ratio, Tex_Dots_text);

	CurveYLMap ylMap_Disp;
	scanCurve(shapeHeight, dispDepth, ylMap_Disp);
	l_curve_disp = ylMap_Disp.l_vec.back();
	ProjTexDots_ResizeMap TexDots_RszMap_Disp;
	sampleTexDotsResize(ylMap_Text, ylMap_Disp, distShapeToEye, TexDots_RszMap_Disp);
	projectTexDots(distShapeToEye, ylMap_Text, ylMap_Disp, Tex_Dots_text, TexDots_RszMap_Disp, Tex_Dots_disp);

	// part 2: static surface
	CurvePtsData y_curve_data_text, y_curve_data_disp;
	int nr_points_height_static = buildCurve_byDelY(ylMap_Disp, y_curve_data_disp);
	projectCurve(ylMap_Text, distShapeToEye, y_curve_data_disp, y_curve_data_text);
	buildSurface_TexOnDisp(shapeWidth, y_curve_data_disp, y_curve_data_text, distShapeToEye, Tex_Dots_disp, my_verts_static);
	buildContour(contourPanelSeparation, y_curve_data_disp, y_curve_data_text, distShapeToEye, my_contour_data);

	// part 3: prebuild movement
	CurvePtsData y_curve_data_text_m, y_curve_data_disp_m;
	if (reinforce_texture_disparity) {
		nr_points_height = buildCurve_byDelL(ylMap_Text, y_curve_data_text_m);
		projectCurve(ylMap_Disp, distShapeToEye, y_curve_data_text_m, y_curve_data_disp_m);
		buildSurface_TexOnText(shapeWidth, y_curve_data_disp_m, y_curve_data_text_m, distShapeToEye, Tex_Dots_text, my_verts_moving);
	}
	else {
		nr_points_height = buildCurve_byDelL(ylMap_Disp, y_curve_data_disp_m);
		projectCurve(ylMap_Text, distShapeToEye, y_curve_data_disp_m, y_curve_data_text_m);
		buildSurface_TexOnDisp(shapeWidth, y_curve_data_disp_m, y_curve_data_text_m, distShapeToEye, Tex_Dots_disp, my_verts_moving);
	}
	buildAllColorsVec(my_verts_moving, AllTimeColorsVec_Moving);


}
void initSurface() {

	stimulus_built = false;


	if (reinforce_texture_disparity) {
		dist_toEye = -(display_distance_jittered - depth_disp + depth_text - 30);
	}
	else {
		dist_toEye = -(display_distance_jittered - 30);
	}

	//dist_toEye = -(display_distance_jittered - depth_disp);
	stimulus_height = tan((DEG2RAD * visual_angle) / 2) * 2 * dist_toEye;
	stimulus_visiblewidth = ratio_visiblewidth_height * stimulus_height;

	// generate the surface vertices
	if (abs(depth_disp - depth_text) < 0.1) {
		buildSurface_congruent(stimulus_width, stimulus_height, depth_text, dist_toEye, stimulus_visiblewidth);

	}
	else {
		buildSurface_incongruent(stimulus_width, stimulus_height, depth_disp, depth_text, dist_toEye, stimulus_visiblewidth);

	}

	stimulus_built = true;
}

void idle()
{
	if (!testVisualStimuliOnly) {
		updateTheMarkers();
		online_apparatus_alignment();
		online_fingers();
	}
	onlineTrial();
	ElapsedTime = trial_timer.getElapsedTimeInMilliSec();

}


// This function seems to be used to shut down the system after use
void shutdown() {
	responseFile.close(); // close this object
	cleanup();
	exit(0);
}
void cleanup()
{
	// Stop the optotrak
	optotrak.stopCollection();

}
void initProjectionScreen(double _focalDist, const Affine3d& _transformation, bool synchronous)
{
	focalDistance = _focalDist;
	screen.setWidthHeight(SCREEN_WIDE_SIZE, SCREEN_WIDE_SIZE * SCREEN_HEIGHT / SCREEN_WIDTH);
	screen.setOffset(alignmentX, alignmentY);
	screen.setFocalDistance(_focalDist);
	screen.transform(_transformation);
	cam.init(screen);
	if (synchronous)
		moveScreenAbsolute(_focalDist, homeFocalDistance, 4500);
	else
		moveScreenAbsoluteAsynchronous(_focalDist, homeFocalDistance, 4500);
}

void initOptotrak()
{
	initRotationM();
	optotrak.setTranslation(calibration);

	if (optotrak.init(LastAlignedFile, OPTO_NUM_MARKERS, OPTO_FRAMERATE, OPTO_MARKER_FREQ, OPTO_DUTY_CYCLE, OPTO_VOLTAGE) != 0)
	{
		cerr << "Something during Optotrak initialization failed, press ENTER to continue. A error log has been generated, look \"opto.err\" in this folder" << endl;
		cin.ignore(1E6, '\n');
		exit(0);
	}

	for (int i = 0; i < 10; i++) {
		updateTheMarkers();
	}

}

void updateTheMarkers()
{
	optotrak.updateMarkers();
	markers = optotrak.getAllMarkers();

	for (int i = 1; i <= OPTO_NUM_MARKERS; i++)
	{
		markers.at(i).p = rotationM * markers.at(i).p;
	}

}

// Initialize motors for moving screen around
void initMotors()
{
	//specify the speed for (objects,screen)
	if (resetScreen_betweenRuns) {
		homeEverything(5000, 4500);
	}


}

// Method that initializes the openGL parameters needed for creating the stimuli. 
// seems like this is not changed for each experiment (maybe for different experimental setup eg monitor)
void initRendering()
{

	glClearColor(0.0, 0.0, 0.0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	/* Set depth buffer clear value */
	glClearDepth(1.0);
	/* Enable depth test */
	glEnable(GL_DEPTH_TEST);
	/* Set depth function */
	glDepthFunc(GL_LEQUAL);
	// scommenta solo se vuoi attivare lo shading degli stimoli

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glLineWidth(1.5);

}

void initStreams()
{

	parametersFile.open(parametersFileName.c_str());
	parameters.loadParameterFile(parametersFile);

	interoculardistance = atof(parameters.find("IOD").c_str());
	display_distance = str2num<double>(parameters.find("dispDepth"));

	int targetCueID = str2num<int>(parameters.find("targetCue"));

	if (targetCueID == 33) {
		trainingCueIsRandom = true;
		targetCueID = rand() % 2;
		cout << "draw: " << targetCueID << endl;
	}


	if (targetCueID == 0) {
		reinforce_texture_disparity = true;
		mv_num = 3;
		speed_moderator = speed_moderator_Text;
	}
	else if (targetCueID == 1) {
		reinforce_texture_disparity = false;
		mv_num = 3;
		speed_moderator = speed_moderator_Disp;
	}
	else {
		string error_on_file_io = string("invalid targetCueID. has to be 0 1 or 33");
		cerr << error_on_file_io << endl;
		shutdown();
	}

	std::string st_cue = std::to_string(targetCueID);

	// Subject name
	subjectName = parameters.find("SubjectName");
	string responseFileName;
	sessionNum = str2num<int>(parameters.find("Session"));

	// trialFile directory
	string dirName = experiment_directory + subjectName;
	mkdir(dirName.c_str()); // windows syntax


	if (sessionNum == 0) {
		session_full_vs_extra = false;
		responseFileName = dirName + "/" + subjectName + "_training_init_cue" + st_cue + ".txt";
		// Principal streams files
		if ((util::fileExists(dirName + "/" + subjectName + "_training_init_cue0.txt") || util::fileExists(dirName + "/" + subjectName + "_training_init_cue1.txt")) && subjectName != "junk")
		{
			string error_on_file_io = string("file already exists");
			cerr << error_on_file_io << endl;
			MessageBox(NULL, (LPCSTR)"FILE ALREADY EXISTS\n Please check the parameters file.", NULL, NULL);
			shutdown();
		}
		parametersFile_extra.open(parametersFileName_extra.c_str());
		parameters_extra.loadParameterFile(parametersFile_extra);
	}
	else {
		session_full_vs_extra = true;
		responseFileName = dirName + "/" + subjectName + "_Training_MAIN.txt";
		// Principal streams files
		if (util::fileExists(dirName + "/" + subjectName + "_Training_MAIN.txt") && subjectName != "junk")
		{
			string error_on_file_io = string("file already exists");
			cerr << error_on_file_io << endl;
			MessageBox(NULL, (LPCSTR)"FILE ALREADY EXISTS\n Please check the parameters file.", NULL, NULL);
			shutdown();
		}
	}


	responseFile.open(responseFileName.c_str());
	responseFile << fixed << responseFile_headers << endl;

	//globalTimer.start();
}


void initVariables()
{

	// eye coordinates

	eyeRight = Vector3d(interoculardistance / 2, 0, 0);
	eyeLeft = Vector3d(-interoculardistance / 2, 0, 0);

	eyeMiddle = Vector3d(0, 0, 0);

	stimulus_height = tan((DEG2RAD * visual_angle) / 2) * 2 * (abs(display_distance));

	stimulus_width = ratio_width_height * stimulus_height;
	stimulus_visiblewidth = ratio_visiblewidth_height * stimulus_height;

	if (testVisualStimuliOnly) {

		initProjectionScreen(display_distance);
		display_distance_jittered = display_distance;
		initSurface();
		current_stage = stimulus_preview;
	}

}


void initBlock()
{


	// initialize the trial matrix
	if (session_full_vs_extra) {
		trial.init(parameters);
		repetition = 3;
		totalTrNum = 6 * 8 * repetition;

	}
	else {
		trial.init(parameters_extra);
		repetition = 4;
		totalTrNum = 12 * repetition;
	}
	trial.next();
	//trial.next(false);

	trialNum = 1;

}


void initMotionFlow() {

	move_cnt = 0;

	nr_mvpts_max = round((nr_points_height - 1) / 4 / rock_movement_divider);
	updateEveryMs = cycle_time / (nr_mvpts_max);
	if (reinforce_texture_disparity) {
		//nr_mvpts_max = round((nr_points_height - 1) / 4 / rock_movement_divider);		
		speed_moderator = speed_moderator_Text * l_curve_text / 60;
	}
	else {
		//nr_mvpts_max = round(l_curve_disp / l_curve_text * (nr_points_height - 1) / 4 / (rock_movement_divider));
		speed_moderator = speed_moderator_Disp * l_curve_disp / 60;
	}
	

	//nr_mvpts_max = round((nr_points_height - 1) / 4 / rock_movement_divider);


	last_time = trial_timer.getElapsedTimeInMilliSec();

	timestamp_mtFlow = trial_timer.getElapsedTimeInMilliSec();

}

void initTrial()
{
	current_stage = prep_trial;

	//initProjectionScreen(display_distance);
	timestamp_mtFlow = 0;
	holdCount_home = 0;
	attemped_MSE = false;

	if (training) {

		depth_mean = depth_training_min + 4.5 * (rand() % 6);

		depth_disp = depth_mean;
		depth_text = depth_mean;
	}
	else {

		depth_mean = trial.getCurrent()["meanDepth"] + (rand() % 3 - 1.0);
		if (sessionNum > 0) {
			depth_delta = trial.getCurrent()["DepthDelta"];

			depth_text = depth_mean + depth_delta;
			depth_disp = depth_mean - depth_delta;
		}
		else {
			depth_text = depth_mean;
			depth_disp = depth_mean;
		}

	}


	jitter_z = ((rand() % 21) - 10) / 2.0; // from -5 to 5
	display_distance_jittered = display_distance + jitter_z;

	initSurface();


	// init movement 
	move_cnt = 0;

	if (stimulus_built) {
		trial_timer.reset();
		trial_timer.start();
		ElapsedTime = 0;
		current_stage = trial_fixate;
	}

}


// Initialize the streams, open the file and write to it

/*** Online operations ***/
void online_apparatus_alignment()
{
	// mirror alignment check
	if (isVisible(markers.at(mirror1).p) && isVisible(markers.at(mirror2).p)) {
		mirrorAlignment = asin(
			abs((markers.at(mirror1).p.z() - markers.at(mirror2).p.z())) /
			sqrt(
				pow(markers.at(mirror1).p.x() - markers.at(mirror2).p.x(), 2) +
				pow(markers.at(mirror1).p.z() - markers.at(mirror2).p.z(), 2)
			)
		) * 180 / M_PI;
	}
	else {
		mirrorAlignment = 999;
	}

	// screen Y alignment check
	if (isVisible(markers.at(screen1).p) && isVisible(markers.at(screen3).p)) {
		screenAlignmentY = asin(
			abs((markers.at(screen1).p.y() - markers.at(screen3).p.y())) /
			sqrt(
				pow(markers.at(screen1).p.x() - markers.at(screen3).p.x(), 2) +
				pow(markers.at(screen1).p.y() - markers.at(screen3).p.y(), 2)
			)
		) * 180 / M_PI;
	}
	else {
		screenAlignmentY = 999;
	}

	// screen Z alignment check
	if (isVisible(markers.at(screen1).p) && isVisible(markers.at(screen2).p)) {
		screenAlignmentZ = asin(
			abs(markers.at(screen1).p.z() - markers.at(screen2).p.z()) /
			sqrt(
				pow(markers.at(screen1).p.x() - markers.at(screen2).p.x(), 2) +
				pow(markers.at(screen1).p.z() - markers.at(screen2).p.z(), 2)
			)
		) * 180 / M_PI *
			abs(markers.at(screen1).p.x() - markers.at(screen2).p.x()) /
			(markers.at(screen1).p.x() - markers.at(screen2).p.x());
	}
	else {
		screenAlignmentZ = 999;
	}
}



void calibrate_fingers()
{
	switch (currentInitStep)
	{
	case to_GetCalibrationPoints: // mark calibration reference points

		if (isVisible(markers[calibration_T].p) && isVisible(markers[calibration_I].p))
		{
			{
				// calibrate for grasping along z axis (update the offset number)
				thumbCalibrationPoint = markers.at(calibration_T).p + thumbCalibration_offset;// calibration point is the touching point
				indexCalibrationPoint = markers.at(calibration_I).p + indexCalibration_offset;// calibration point is the touching point
			}

			beepOk(1);
			//homePos = (thumbCalibrationPoint + indexCalibrationPoint) / 2 + Vector3d(-15, 25, 0);

			currentInitStep = to_CalibrateFingerTips;
		}
		else {
			beepOk(3);
		}

		break;

	case to_CalibrateFingerTips: // mark finger tips

		if (allVisibleFingers)
		{
			indexCoords.init(indexCalibrationPoint, markers.at(ind1).p, markers.at(ind2).p, markers.at(ind3).p);
			thumbCoords.init(thumbCalibrationPoint, markers.at(thu1).p, markers.at(thu2).p, markers.at(thu3).p);

			beepOk(1);
			if (fingerCalibration_TIPandJOINT) {
				currentInitStep = to_CalibrateFingerJoints;
			}
			else {
				calibrationNum++;
				if (Exp_Initialized) {
					Fingers_Calibrated = true;
					initTrial();
				}
				else {
					currentInitStep = to_MarkHomePos;
					Fingers_Calibrated = true;
				}
			}
		}
		else {
			beepOk(3);
		}

		break;

	case to_CalibrateFingerJoints: // mark finger joints

		if (allVisibleFingers)
		{
			indexJointCoords.init(indexCalibrationPoint, markers.at(ind1).p, markers.at(ind2).p, markers.at(ind3).p);
			thumbJointCoords.init(thumbCalibrationPoint, markers.at(thu1).p, markers.at(thu2).p, markers.at(thu3).p);

			beepOk(1);
			currentInitStep = to_MarkHomePos;
			Fingers_Calibrated = true;
		}
		else {
			beepOk(3);
		}
		break;


	}
}


void calibrate_system() {
	switch (currentInitStep) {
	case to_MarkHomePos:
		// ask hand to rest at a designated place and press F to mark that as the home position
		if (allVisibleFingers) {
			beepOk(1);
			homePos = (ind + thm) / 2;

			currentInitStep = to_MoveApparatus;
		}
		else {
			beepOk(3);
		}

		break;

	case to_MoveApparatus:

		beepOk(1);
		initProjectionScreen(display_distance);
		currentInitStep = to_confirmReady;
		// push the physical object back to clear the grasping space

		break;

	case to_confirmReady:
		if (abs(mirrorAlignment - 45.0) < 0.2) {
			beepOk(1);

			Exp_Initialized = true;
			initSurface();
			current_stage = stimulus_preview;
		}
		else {
			beepOk(8);
		}
		break;
	}

}

void online_fingers()
{
	allVisibleIndex = isVisible(markers.at(ind1).p) && isVisible(markers.at(ind2).p) && isVisible(markers.at(ind3).p);
	allVisibleThumb = isVisible(markers.at(thu1).p) && isVisible(markers.at(thu2).p) && isVisible(markers.at(thu3).p);
	allVisibleFingers = allVisibleIndex && allVisibleThumb;

	if (Fingers_Calibrated) {

		if (allVisibleThumb) {
			thumbCoords.update(markers.at(thu1).p, markers.at(thu2).p, markers.at(thu3).p);
			thm = thumbCoords.getP1();

			thmToHome = homePos - thm;
			//thmToTarget = thmTarget - thm;

			old_dist_thm_home = dist_thm_home;
			dist_thm_home = thmToHome.norm();
			vel_dist_home = dist_thm_home - old_dist_thm_home;


		}
		else {

			old_dist_thm_home = dist_thm_home;
			dist_thm_home = 999;
			vel_dist_home = 999;

		}

		if (allVisibleFingers) {
			indexCoords.update(markers.at(ind1).p, markers.at(ind2).p, markers.at(ind3).p);
			ind = indexCoords.getP1();

			vec_ind_thm = ind - thm;
			old_grip_aperture = grip_aperture;
			grip_aperture = vec_ind_thm.norm();
			vel_grip_change = grip_aperture - old_grip_aperture;
		}
		else {
			grip_aperture = 999;
			old_grip_aperture = grip_aperture;
			vel_grip_change = 999;
		}

		//handNearHome = (dist_thm_home < thresholdDist_near_home);
		handNearHome = (abs(thmToHome.y()) < 12);
		handSteady = (abs(vel_dist_home) < thresholdVelthm_steady);
		gripSmall = (grip_aperture < thresholdGA_small);
		gripSteady = (abs(vel_grip_change) < thresholdVelGA_steady);

	}

}



// this is run at compilation because it's titled f'main'
int main(int argc, char* argv[])
{
	//functions from cncsvision packages
	mathcommon::randomizeStart();

	// initializing glut (to use OpenGL)
	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_STEREO);

	glutGameModeString("1024x768:32@85"); //resolution  
	glutEnterGameMode();
	glutFullScreen();

	// initializes optotrak and velmex motors

	if (!testVisualStimuliOnly) {
		initOptotrak();
	}
	initMotors();

	initRendering(); // initializes the openGL parameters needed for creating the stimuli

	initStreams(); // streams as in files for writing data

	initVariables();

	// glut callback, OpenGL functions that are infinite loops to constantly run 

	glutDisplayFunc(drawGLScene); // keep drawing the stimuli

	glutKeyboardFunc(handleKeypress); // check for keypress

	glutReshapeFunc(handleResize);

	glutIdleFunc(idle);

	glutTimerFunc(TIMER_MS, update, 0);

	glutSetCursor(GLUT_CURSOR_NONE);

	//boost::thread initVariablesThread(&initVariables); 

	glutMainLoop();

	//cleanup();
	return 0;
}