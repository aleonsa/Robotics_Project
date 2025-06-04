// PhantomDlg.cpp : implementation file
//

#include "stdafx.h"
#include "Phantom.h"
#include "PhantomDlg.h"

#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>

// User includes
#include <math.h>
#include "mmsystem.h" // Multimedia timers
#include "analysis.h"

// CPhantomDlg dialog

// User definitions
#define pi 3.1415926535
#define MAX_GRAF_ROWS 300000
HHD hHD;											// OpenHaptics handler
bool initialized = false, schedulerStarted = false; // user flags
double taus[3] = {0.0, 0.0, 0.0};
double q[3] = {0.0};
const double T = 0.002; // Sample time
const int n = 3;		// Number of joints
HDSchedulerHandle servoLoopHandle;
bool iCHome = true, homeCompletedFlag = true, iCControl = true, controlCompletedFlag = true;

MMRESULT homeTimerID, controlTimerID;
const double aDH[3] = {0.0, 0.1345, 0.178}, dDH[3] = {0.0, 0.0, 0.0}, alDH[3] = {pi / 2.0, 0.0, 0.0};

double grafi[MAX_GRAF_ROWS][19] = {0.0};
const int grafSkip = 0; // Number of time intervals to discard for the output file
int indx = 0;

CPhantomDlg::CPhantomDlg(CWnd *pParent /*=NULL*/)
	: CDialog(CPhantomDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CPhantomDlg::DoDataExchange(CDataExchange *pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_EDIT1, m_statusTextBox);
	DDX_Control(pDX, IDC_READENCODERS, m_readEncoders);
	DDX_Control(pDX, IDC_ENCODER1, m_encoderBox1);
	DDX_Control(pDX, IDC_ENCODER2, m_encoderBox2);
	DDX_Control(pDX, IDC_ENCODER3, m_encoderBox3);
}

BEGIN_MESSAGE_MAP(CPhantomDlg, CDialog)
ON_WM_PAINT()
ON_WM_QUERYDRAGICON()
ON_WM_CLOSE()
//}}AFX_MSG_MAP
ON_BN_CLICKED(IDC_INITIALIZE, &CPhantomDlg::OnBnClickedInitialize)
ON_BN_CLICKED(IDC_CALIBRATION, &CPhantomDlg::OnBnClickedCalibration)
ON_BN_CLICKED(IDC_READENCODERS, &CPhantomDlg::OnBnClickedReadencoders)
ON_BN_CLICKED(IDC_HOME, &CPhantomDlg::OnBnClickedHome)
ON_BN_CLICKED(IDC_CONTROL, &CPhantomDlg::OnBnClickedControl)
END_MESSAGE_MAP()

// CPhantomDlg message handlers

BOOL CPhantomDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);	 // Set big icon
	SetIcon(m_hIcon, FALSE); // Set small icon

	// TODO: Add extra initialization here

	return TRUE; // return TRUE  unless you set the focus to a control
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CPhantomDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialog::OnPaint();
	}
}

void CPhantomDlg::OnClose()
{
	timeEndPeriod(1); // Finish Multimedia Timer resolution

	if (!iCHome)
	{
		timeKillEvent(homeTimerID);
		homeCompletedFlag = iCHome = true;
	}
	if (!iCControl)
	{
		timeKillEvent(controlTimerID);
		controlCompletedFlag = iCControl = true;
	}

	if (initialized && hdIsEnabled(HD_FORCE_OUTPUT))
		hdDisable(HD_FORCE_OUTPUT);
	hdUnschedule(servoLoopHandle);
	if (schedulerStarted)
		hdStopScheduler();
	if (initialized)
		hdDisableDevice(hHD);

	FILE *outFile;
	if (fopen_s(&outFile, "expData.m", "w") != 0)
	{
		MessageBox(_T("No se pudo crear el archivo para graficar"));
	}
	else
	{
		for (int i = 0; i < indx; i++)
		{
			fprintf(outFile, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n", grafi[i][0], grafi[i][1], grafi[i][2], grafi[i][3], grafi[i][4], grafi[i][5], grafi[i][6], grafi[i][7], grafi[i][8], grafi[i][9], grafi[i][10], grafi[i][11], grafi[i][12], grafi[i][13], grafi[i][14], grafi[i][15], grafi[i][16], grafi[i][17], grafi[i][18]);
		}
		fclose(outFile);
	}

	exit(0);
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CPhantomDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

HDCallbackCode HDCALLBACK CalibrationStatusCallback(void *pUserData)
{
	HDenum *pStatus = (HDenum *)pUserData;

	hdBeginFrame(hdGetCurrentDevice());

	int supportedCalibrationStyles;
	int calibrationStyle;
	HDErrorInfo error;

	hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);

	if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET ||
		supportedCalibrationStyles & HD_CALIBRATION_INKWELL)
	{
		hdUpdateCalibration(supportedCalibrationStyles);
	}

	hdEndFrame(hdGetCurrentDevice());

	return HD_CALLBACK_DONE;
}

HDCallbackCode HDCALLBACK DeviceAngPositionCallback(void *pUserData)
{
	HDdouble *pPosition = (HDdouble *)pUserData;

	hdBeginFrame(hdGetCurrentDevice());
	hdGetDoublev(HD_CURRENT_JOINT_ANGLES, pPosition);
	hdEndFrame(hdGetCurrentDevice());

	return HD_CALLBACK_DONE;
}

typedef struct
{
	hduVector3Dd position;
} DeviceStateStruct;

DeviceStateStruct state;

HDCallbackCode HDCALLBACK ServoLoopCallback(void *pUserData)
{
	HHD hHD = hdGetCurrentDevice();
	hdBeginFrame(hHD);
	// HDdouble pos[3];
	DeviceStateStruct *pos = static_cast<DeviceStateStruct *>(pUserData);

	hdGetDoublev(HD_CURRENT_JOINT_ANGLES, pos->position);

	HDdouble torque[3];

	for (int i = 0; i < n; i++)
	{
		if (abs(taus[i]) > 1)
		{
			torque[i] = 1500.0;
		}
		else
		{
			torque[i] = 1500.0 * taus[i];
		}
	}
	torque[0] *= -1.0;

	/*torque[0] = -taus[0] * 3000;
	torque[1] = taus[1] * 3000;
	torque[2] = taus[2] * 3000;*/

	/*q[0] = -state.position[0];
	q[1] = state.position[1];
	q[2] = state.position[2]-0.5*pi-q[1]-0.0398;*/

	q[0] = -state.position[0];
	q[1] = (state.position[1] - 3.78 * pi / 180.0) * (90.0 / 88.42);
	q[2] = (state.position[2] - state.position[1] + 6.76 * pi / 180.0 - pi / 2.0) * (45.0 / 43.54); // For 3 dof

	hdSetDoublev(HD_CURRENT_JOINT_TORQUE, torque);

	hdEndFrame(hHD);

	return HD_CALLBACK_CONTINUE;
}

void CPhantomDlg::OnBnClickedInitialize()
{
	HDErrorInfo error;
	hHD = hdInitDevice(HD_DEFAULT_DEVICE);
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		MessageBox(_T("Phantom Device not Found!"));
		return;
	}

	servoLoopHandle = hdScheduleAsynchronous(ServoLoopCallback, &state, HD_DEFAULT_SCHEDULER_PRIORITY);

	if (!hdIsEnabled(HD_FORCE_OUTPUT))
		hdEnable(HD_FORCE_OUTPUT);

	if (!schedulerStarted)
	{
		hdStartScheduler();
		schedulerStarted = true;
		Sleep(500);
	}

	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		MessageBox(_T("Servo loop initialization error"));

		hdDisableDevice(hHD);
		exit(-1);
	}
	else
	{
		initialized = true;
		m_statusTextBox.SetWindowTextW(_T("*** Phantom Robot initialized ***"));
	}

	timeBeginPeriod(1); // Multimedia timer init
}

void CPhantomDlg::OnBnClickedCalibration()
{
	if (initialized)
	{

		HDenum status;
		hdScheduleSynchronous(CalibrationStatusCallback, &status, HD_DEFAULT_SCHEDULER_PRIORITY);

		if (status == HD_CALIBRATION_NEEDS_MANUAL_INPUT)
			m_statusTextBox.SetWindowTextW(_T("*** Not calibrated yet! ***"));
		else
			m_statusTextBox.SetWindowTextW(_T("*** Calibration Done ***"));

		return;
	}
	else
	{
		MessageBox(_T(" Please initialize first the Phantom device "));
	}
}

void CPhantomDlg::OnBnClickedReadencoders()
{
	/*hduVector3Dd position;

	if(!schedulerStarted)
	{
		hdStartScheduler();
		schedulerStarted = true;
	}*/

	// hdScheduleSynchronous(DeviceAngPositionCallback, position, HD_MIN_SCHEDULER_PRIORITY);

	CString text[3];
	text[0].Format(L"%.3f", q[0] * 180.0 / pi);
	text[1].Format(L"%.3f", q[1] * 180.0 / pi);
	text[2].Format(L"%.3f", q[2] * 180.0 / pi);

	m_encoderBox1.SetWindowTextW(text[0]);
	m_encoderBox2.SetWindowTextW(text[1]);
	m_encoderBox3.SetWindowTextW(text[2]);

	return;
}

void CPhantomDlg::OnBnClickedHome()
{
	if (!iCHome)
	{
		timeKillEvent(homeTimerID);
		homeCompletedFlag = iCHome = true;
	}
	if (!iCControl)
	{
		timeKillEvent(controlTimerID);
	}

	homeTimerID = timeSetEvent(T * 1000, 0, HomeTimerProc, 0, TIME_PERIODIC);
}

void CALLBACK CPhantomDlg::HomeTimerProc(UINT uID, UINT uMsg, DWORD dwUser, DWORD dw1, DWORD dw2)
{
	static double tini = 0.0;
	static double ei[n] = {0.0}, e_1[n] = {0.0};
	static double a0[n] = {0.0}, a3[n] = {0.0}, a4[n] = {0.0}, a5[n] = {0.0};
	const double qf[n] = {0.0 * pi / 180.0, 45.0 * pi / 180.0, -90.0 * pi / 180.0}, tf = 1.0; // Posici n home
	double t = 0.0, qd[n] = {0.0}, e[n] = {0.0}, ep[n] = {0.0};
	// const double kp[n] = {2.0,2.0,2.0}, ki[n] = {0.5,0.5,0.5}, kd[n] = {0.01,0.01,0.01};
	const double kp[n] = {0.66, 0.66, 0.66}, ki[n] = {0.16, 0.16, 0.16}, kd[n] = {0.003, 0.003, 0.003};

	CPhantomDlg *pMainWnd = (CPhantomDlg *)AfxGetApp()->m_pMainWnd;

	if (iCHome)
	{
		tini = timeGetTime();
		for (int i = 0; i < n; i++)
		{
			a0[i] = q[i];
			a3[i] = 10 * (qf[i] - a0[i]) / (tf * tf * tf);
			a4[i] = -15 * (qf[i] - a0[i]) / (tf * tf * tf * tf);
			a5[i] = 6 * (qf[i] - a0[i]) / (tf * tf * tf * tf * tf);
		}

		pMainWnd->m_statusTextBox.SetWindowTextW(_T("*** Reaching home position ***"));
	}

	t = (timeGetTime() - tini) / 1000.0;

	for (int i = 0; i < n; i++)
	{
		if (t <= tf)
			qd[i] = a0[i] + a3[i] * t * t * t + a4[i] * t * t * t * t + a5[i] * t * t * t * t * t;
		else
			qd[i] = qf[i];
		e[i] = q[i] - qd[i];
		if (iCHome)
			e_1[i] = e[i];
		ep[i] = (e[i] - e_1[i]) / T;
		taus[i] = -kp[i] * e[i] - ki[i] * ei[i] - kd[i] * ep[i];
	}

	/*if(iCHome)
		indx = 0;
	grafi[indx][0] = t;
	grafi[indx][1] = q[0]*180.0/pi;
	grafi[indx][2] = q[1]*180.0/pi;
	grafi[indx][3] = q[2]*180.0/pi;
	grafi[indx][4] = qd[0]*180.0/pi;
	grafi[indx][5] = qd[1]*180.0/pi;
	grafi[indx][6] = qd[2]*180.0/pi;
	grafi[indx][7] = taus[0];
	grafi[indx][8] = taus[1];
	grafi[indx][9] = taus[2];
	indx++;
	*/

	for (int i = 0; i < n; i++)
	{
		ei[i] += e[i] * T;
		e_1[i] = e[i];
	}

	if (t > tf && homeCompletedFlag)
	{
		pMainWnd->m_statusTextBox.SetWindowTextW(_T("*** Home position ***"));
		homeCompletedFlag = false;
	}

	if (iCHome)
		iCHome = false;

	return;
}

void CPhantomDlg::OnBnClickedControl()
{

	if (!iCHome)
	{
		timeKillEvent(homeTimerID);
	}
	if (!iCControl)
	{
		timeKillEvent(controlTimerID);
		controlCompletedFlag = iCControl = true;
	}

	controlTimerID = timeSetEvent(T * 1000, 0, ControlTimerProc, 0, TIME_PERIODIC);
}

void CALLBACK CPhantomDlg::ControlTimerProc(UINT uID, UINT uMsg, DWORD dwUser, DWORD dw1, DWORD dw2)
{
	static double tini = 0.0;
	static int grafFlag = 0;
	const double g0 = 9.8;
	const int p = 8;
	double t = 0.0, g[n] = {0.0}, e[n] = {0.0}, qp[n] = {0.0}, qpfp[n] = {0.0}, qppfp[n] = {0.0}, c2 = 0.0, c3 = 0.0, c23 = 0.0, s2 = 0.0, s3 = 0.0, s23 = 0.0;

	static double th[p] = {0.002517290542366, 0.001082466154947, 0.001374082981419, 0 * 0.000768230130927, 0 * 0.035267357329092, 0 * 0.007444737668751, 0.004491588558515, 0.005345056038429}; // vector nominal de parametros

	static double Theta[p] = {0, 0, 0, 0, 0, 0, 0, 0};																																		 // vector de parametros adaptados
	static double th_anterior[p] = {0.002517290542366, 0.001082466154947, 0.001374082981419, 0.000768230130927, 0.035267357329092, 0.007444737668751, 0.004491588558515, 0.005345056038429}; // inicializa parametros para adaptacion

	// static double th[p] = { 0.00453641,0.000341864,0.004503919,0.001373760 ,0.001188021 ,0.000553657,0.004948273,0.012955526 };//NEW FOR 1500
	static double Y[3][8] = {0};

	const double kp[n] = {2.3, 2.0, 2.3}, kd[n] = {0.03, 0.03, 0.03}, ki[n] = {0.36, 0.36, 0.36}; //
	const double ks[n] = {0.12, 0.12, 0.12};
	double qd[n] = {0.0}, qdp[n] = {0.0}, qdpp[n] = {0.0};
	static double q_1[n], qpf[n], qppf[n];
	double s[n] = {0.0, 0.0, 0.0};
	static double a0[n] = {0.0}, a3[n] = {0.0}, a4[n] = {0.0}, a5[n] = {0.0};
	const double qf[n] = {50.0 * pi / 180.0, 50.0 * pi / 180.0, -60.0 * pi / 180.0}, tf = 2.0; // Posici n deseada control

	const double Tsq = 5.0, Amp[n] = {40.0, 190.0, 120.0}, Amp_n[n] = {-40.0, 40.0, 0.0};

	double W[p] = {0.0}, Wfp[p] = {0.0}, tauqp = 0.0, yep = 0.0, thp[p] = {0.0}, etau = 0.0, Wthe = 0.0;
	static double Wf[p] = {0.0}, ye = 0.0;
	// const double lamfilt = 30.0, p0 = 0.0002; //For identification
	const double lamfilt = 10.0, p0 = 0.0002;

	double qt[n] = {0.0}, qgp[n] = {0.0}, q2gp[n] = {0.0}, z1gp[n] = {0.0}, z2gp[n] = {0.0};
	static double qg[n] = {0.0}, q2g[n] = {0.0}, z1g[n] = {0.0}, z2g[n] = {0.0};
	const double pol1 = 60.0, pol2 = 60.0, pol3 = 60.0, pol4 = 60.0;
	const double lam0 = pol1 * pol2 * pol3 * pol4, lam1 = pol1 * pol2 * (pol3 + pol4) + pol3 * pol4 * (pol1 + pol2), lam2 = pol1 * (pol2 + pol3 + pol4) + pol2 * (pol3 + pol4) + pol3 * pol4, lam3 = pol1 + pol2 + pol3 + pol4;
	const static double D[n] = {th[3], th[4], th[5]};
	static double Xi[n] = {0.0}, Xip[n] = {0.0};
	CPhantomDlg *pMainWnd = (CPhantomDlg *)AfxGetApp()->m_pMainWnd;

	double H[n][n] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
	double G[n] = {0, 0, 0};
	double C[n][n] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

	// Matrices estimadas (SlotineLi Adaptable)
	double Hhat[n][n] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
	double Ghat[n] = {0, 0, 0};
	double Chat[n][n] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
	// double Dhat[n] = { 0, 0, 0 };

	if (iCControl)
	{
		tini = timeGetTime();

		// Polinomio de quinto orden para interpolaci n
		for (int i = 0; i < n; i++)
		{
			a0[i] = q[i];
			a3[i] = 10 * (qf[i] - a0[i]) / (tf * tf * tf);
			a4[i] = -15 * (qf[i] - a0[i]) / (tf * tf * tf * tf);
			a5[i] = 6 * (qf[i] - a0[i]) / (tf * tf * tf * tf * tf);
			qg[i] = q[i];
			q_1[i] = q[i];
			qpf[i] = qppf[i] = 0.0;
		}
		// pMainWnd->m_statusTextBox.SetWindowTextW(_T("*** Control in progress ***"));
	}

	t = (timeGetTime() - tini) / 1000.0;

	c2 = cos(q[1]);
	c3 = cos(q[2]);
	c23 = cos(q[1] + q[2]);

	s2 = sin(q[1]);
	s3 = sin(q[2]);
	s23 = sin(q[1] + q[2]);

	for (int i = 0; i < n; i++)
	{
		if (t <= tf)
		{
			qd[i] = a0[i] + a3[i] * t * t * t + a4[i] * t * t * t * t + a5[i] * t * t * t * t * t;
			qdp[i] = 3 * a3[i] * t * t + 4 * a4[i] * t * t * t + 5 * a5[i] * t * t * t * t;
			qdpp[i] = 6 * a3[i] * t + 12 * a4[i] * t * t + 20 * a5[i] * t * t * t;
		}
		else
		{
			qd[i] = qf[i];
			qdp[i] = 0;
			qdpp[i] = 0;
		}
		if (iCControl)
			q_1[i] = q[i];
	}

	for (int i = 0; i < n; i++)
	{
		qp[i] = (q[i] - q_1[i]) / T;			  // qp sin filtrar
		qpfp[i] = lamfilt * (qp[i] - qpf[i]);	  // velocidad filtrada
		qppfp[i] = lamfilt * (qpfp[i] - qppf[i]); // aceleracion fitrada
	}

	// for (int i = 0; i < n; i++)
	//{

	// PD
	// taus[i] = -kd[i] * (qpf[i]) - kp[i] * (q[i] - qd[i]);

	//}

	// Modelo
	H[0][0] = c2 * c2 * th[0] + c2 * c23 * th[1] + s23 * s23 * th[2];
	H[1][1] = th[0] + 2 * c3 * th[1] + th[2];
	H[1][2] = c3 * th[1] + th[2];
	H[2][1] = c3 * th[1] + th[2];
	H[2][2] = th[2];

	G[1] = g0 * (c2 * th[6] + c23 * th[7]);
	G[2] = g0 * c23 * th[7];

	// matriz C(q,qp)
	C[0][0] = -c2 * s2 * qpf[1] * th[0] - 0.5 * (c2 * s23 * (qpf[1] + qpf[2]) + s2 * c23 * qpf[1]) * th[1] + c23 * s23 * (qpf[1] + qpf[2]) * th[2];
	C[0][1] = -c2 * s2 * qpf[0] * th[0] - 0.5 * (s2 * c23 + c2 * s23) * qpf[0] * th[1] + s23 * c23 * qpf[0] * th[2];
	C[0][2] = -0.5 * c2 * s23 * qpf[0] * th[1] + c23 * s23 * qpf[0] * th[2];
	C[1][0] = -1 * C[0][1]; // c21
	C[1][1] = -s3 * qpf[2] * th[1];
	C[1][2] = -s3 * (qpf[1] + qpf[2]) * th[1];
	C[2][0] = -1 * C[0][2];
	C[2][1] = s3 * qpf[1] * th[1];

	// Par calculado perron
	// luego implementar con el Mul2D de la libreria analysis

	// Definiciones para Par calculado

	// Ganancias ParCalculado
	const double kd1 = 3;
	const double kp1 = 350;

	// Ganancias SlotineLi
	// const double kd1 = 0.1;
	// const double kp1 = 3;*/

	// Ganancias SlotineLi Adaptable
	// const double kd1 = 0.1;
	// const double kp1 = 3;

	const double Kv[n][n] = {{kd1, 0, 0}, {0, kd1, 0}, {0, 0, kd1}};
	const double Kp[n][n] = {{kp1, 0, 0}, {0, kp1 * 1.1, 0}, {0, 0, kp1}};

	// Ganancias y definiciones de SlotineLi
	// const double Lambda[n][n] = { { 0, 0, 0}, {0, 0, 0}, {0, 0, 0} }; // SlotineLi
	// const double Kvinv[n][n] = { { 0, 0, 0}, {0, 0, 0}, {0, 0, 0} }; // SlotineLi
	// double Lambdaqe[n] = { 0,0,0 };
	// double Lambdaqep[n] = { 0,0,0 };
	// double sSlotineLi[n] = { 0,0,0 };

	// Ganancias y definiciones de SlotineLi Adaptable
	double Lambda[n][n] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}; // SlotineLi
	double Kvinv[n][n] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};	 // SlotineLi
	double Lambdaqe[n] = {0, 0, 0};
	double Lambdaqep[n] = {0, 0, 0};
	double sSlotineLi[n] = {0, 0, 0};
	double Gamma[8][8] = {0};

	// Definicion de errores
	double qep[n] = {0, 0, 0};
	double qe[n] = {0, 0, 0};

	double qrp[n] = {0, 0, 0};	// SlotineLi
	double qrpp[n] = {0, 0, 0}; // SlotineLi

	for (int i = 0; i < n; i++)
	{
		qe[i] = q[i] - qd[i];
		qep[i] = qpf[i] - qdp[i];
	};

	// ########################################## Par Calculado ##########################################
	//
	double Hbar1[n] = {0, 0, 0}; // h*(qdpp)
	double Hbar2[n] = {0, 0, 0}; // h*(kv*qep)
	double Hbar3[n] = {0, 0, 0}; // h*(kp*qe)
	double HKv[n][n] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
	double HKp[n][n] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
	double Cqp[n] = {0, 0, 0};
	double Dqp[n] = {0, 0, 0};

	MatrixMul(H, Kv, 3, 3, 3, HKv);	 // kv*qerror_dot
	MatrixMul(H, Kp, 3, 3, 3, HKp);	 // kp*qerror
	MatrixMul(C, qpf, 3, 3, 1, Cqp); // kv*qerror_dot
	MatrixMul(D, qpf, 3, 3, 1, Dqp); // kp*qerror
	MatrixMul(H, qdpp, 3, 3, 1, Hbar1);
	MatrixMul(HKv, qep, 3, 3, 1, Hbar2);
	MatrixMul(HKp, qe, 3, 3, 1, Hbar3);

	// ########################################## SlotineLi ##########################################
	// InvMatrix(Kv, n, Kvinv);
	// MatrixMul(Kvinv, Kp, 3, 3, 3, Lambda);
	// MatrixMul(Lambda, qe, 3, 3, 1, Lambdaqe);
	// MatrixMul(Lambda, qep, 3, 3, 1, Lambdaqep);

	// for (int i = 0; i < n; i++)
	//{
	//	qrp[i] = qdp[i] - Lambdaqe[i]; // qr_dot
	//	qrpp[i] = qdpp[i] - Lambdaqep[i]; // qr_ddot
	//	sSlotineLi[i] = qpf[i] - qrp[i] ;// s
	// };
	// double Hqrpp[n] = { 0,0,0 }; // H*(qdpp)
	// double Kvs[n] = { 0, 0, 0 }; // ?? no debe ser de nx1 ??
	// double Cqrp[n] = { 0,0,0 };
	// double Dqrp[n] = { 0,0,0 };

	// MatrixMul(C, qrp, 3, 3, 1, Cqrp); //    C*qrp
	// MatrixMul(D, qrp, 3, 3, 1, Dqrp); //	D*qrp
	// MatrixMul(H, qrpp, 3, 3, 1, Hqrpp); // H*qrpp
	// MatrixMul(Kv, sSlotineLi, 3, 3, 1, Kvs);  //   Kv*s

	// ############################ Slotine Li Adaptable #################################################

	//		// Mecanismo de adaptacion
	// double GammaYtransp[8][3] = { 0 };
	// double Ytransp[8][3] = { 0 };
	// double theta_dot_error[8] = { 0 };

	//	Y[0][0] = c2 * c2 * qrpp[0] - 2 * c2 * s2 * qpf[0] * qpf[1];
	// Y[0][1] = c2 * c23 * qrpp[0] - (c2 * s23 * (qpf[1] + qpf[2]) + s2 * c23 * qpf[1]) * qpf[0];
	// Y[0][2] = s23 * s23 * qrpp[0] + c23 * s23 * (qpf[1] + 2 * qpf[2]) * qpf[0] + s23 * c23 * qpf[0] * qpf[1];
	// Y[0][3] = qpf[0];
	// Y[0][4] = 0.0;
	// Y[0][5] = 0.0;
	// Y[0][6] = 0.0;
	// Y[0][7] = 0.0;

	// Y[1][0] = qrpp[1] + c2 * s2 * qpf[0] * qpf[0];
	// Y[1][1] = 2 * c3 * qrpp[1] + c3 * qrpp[2] + 0.5 * (s2 * c23 + c2 * s23) * qpf[0] * qpf[0] -
	//	2 * s3 * qpf[1] * qpf[2] - s3 * qpf[2] * qpf[2];
	// Y[1][2] = qrpp[1] + qrpp[2] - s23 * c23 * qpf[0] * qpf[0];
	// Y[1][3] = 0.0;
	// Y[1][4] = qpf[1];
	// Y[1][5] = 0.0;
	// Y[1][6] = g0 * c2;
	// Y[1][7] = g0 * c23;

	// Y[2][0] = 0.0;
	// Y[2][1] = c2 * qrpp[1] + 0.5 * c2 * s23 * qpf[0] * qpf[0] + s3 * qpf[1] * qpf[1];
	// Y[2][2] = qrpp[1] + qrpp[2] - c23 * s23 * qpf[0] * qpf[0];
	// Y[2][3] = 0.0;
	// Y[2][4] = 0.0;
	// Y[2][5] = qpf[2];
	// Y[2][6] = 0.0;
	// Y[2][7] = g0 * c23;

	// Transpose(Y, 3, 8, Ytransp);
	// MatrixMul(Gamma, Ytransp, 8, 8, 3, GammaYtransp); //
	// MatrixMul(GammaYtransp, sSlotineLi, 8, 3, 1, theta_dot_error); //

	// for (int i = 0; i < 8; i++)
	//{
	//	Theta[i] = th_anterior[i] - theta_dot_error[i] * T;
	// };

	//// Modelo con parametros estimados
	// Hhat[0][0] = c2 * c2 * Theta[0] + c2 * c23 * Theta[1] + s23 * s23 * Theta[2];
	// Hhat[1][1] = Theta[0] + 2 * c3 * Theta[1] + Theta[2];
	// Hhat[1][2] = c3 * Theta[1] + Theta[2];
	// Hhat[2][1] = c3 * Theta[1] + Theta[2];
	// Hhat[2][2] = Theta[2];

	// Ghat[1] = g0 * (c2 * Theta[6] + c23 * Theta[7]);
	// Ghat[2] = g0 * c23 * Theta[7];

	////matriz C(q,qp)
	// Chat[0][0] = -c2 * s2 * qpf[1] * Theta[0] - 0.5 * (c2 * s23 * (qpf[1] + qpf[2]) + s2 * c23 * qpf[1]) * Theta[1] + c23 * s23 * (qpf[1] + qpf[2]) * Theta[2];
	// Chat[0][1] = -c2 * s2 * qpf[0] * Theta[0] - 0.5 * (s2 * c23 + c2 * s23) * qpf[0] * Theta[1] + s23 * c23 * qpf[0] * Theta[2];
	// Chat[0][2] = -0.5 * c2 * s23 * qpf[0] * Theta[1] + c23 * s23 * qpf[0] * Theta[2];
	// Chat[1][0] = -1 * Chat[0][1]; //c21
	// Chat[1][1] = -s3 * qpf[2] * Theta[1];
	// Chat[1][2] = -s3 * (qpf[1] + qpf[2]) * Theta[1];
	// Chat[2][0] = -1 * Chat[0][2];
	// Chat[2][1] = s3 * qpf[1] * Theta[1];

	// static double Dhat[n] = { Theta[3], Theta[4], Theta[5] };

	//// ley de control
	// InvMatrix(Kv, n, Kvinv);
	// MatrixMul(Kvinv, Kp, 3, 3, 3, Lambda);
	// MatrixMul(Lambda, qe, 3, 3, 1, Lambdaqe);
	// MatrixMul(Lambda, qep, 3, 3, 1, Lambdaqep);

	// for (int i = 0; i < n; i++)
	//{
	//	qrp[i] = qdp[i] - Lambdaqe[i]; // qr_dot
	//	qrpp[i] = qdpp[i] - Lambdaqep[i]; // qr_ddot
	//	sSlotineLi[i] = qpf[i] - qrp[i];// s
	// };
	// double Hhatqrpp[n] = { 0,0,0 }; // H*(qdpp)
	// double Kvs[n] = { 0, 0, 0 };
	// double Chatqrp[n] = { 0,0,0 };
	// double Dhatqrp[n] = { 0,0,0 };

	// MatrixMul(Chat, qrp, 3, 3, 1, Chatqrp); //    C*qrp
	// MatrixMul(Dhat, qrp, 3, 3, 1, Dhatqrp); //	   D*qrp
	// MatrixMul(Hhat, qrpp, 3, 3, 1, Hhatqrpp); // H*qrpp
	// MatrixMul(Kv, sSlotineLi, 3, 3, 1, Kvs);  //   Kv*s

	for (int i = 0; i < n; i++)
	{
		// Par calculado
		taus[i] = Hbar1[i] - Hbar2[i] - Hbar3[i] + Cqp[i] + Dqp[i] + G[i];
		// SlotineLi
		// taus[i] = Hqrpp[i] + Cqrp[i] + Dqrp[i] + G[i] - Kvs[i];
		// SlotineLi Adaptable
		// taus[i] = Hhatqrpp[i] + Chatqrp[i] + Dhatqrp[i] + Ghat[i] - Kvs[i];
	};

	if (t > tf && controlCompletedFlag)
	{
		pMainWnd->m_statusTextBox.SetWindowTextW(_T("*** Control completed ***"));
		controlCompletedFlag = false;
	}

	if (iCControl)
		indx = 0;
	if (grafFlag == 0)
	{
		grafi[indx][0] = t;
		grafi[indx][1] = q[0];
		grafi[indx][2] = q[1];
		grafi[indx][3] = q[2];
		grafi[indx][4] = taus[0];
		grafi[indx][5] = taus[1];
		grafi[indx][6] = taus[2];
		// grafi[indx][7] = Y[1][0];// qpf[0];
		// grafi[indx][8] = Y[1][1];//qpf[1];
		// grafi[indx][9] = Y[2][1];//qpf[2];
		grafi[indx][7] = qpf[0];
		grafi[indx][8] = qpf[1];
		grafi[indx][9] = qpf[2];
		grafi[indx][10] = qppf[0];
		grafi[indx][11] = qppf[1];
		grafi[indx][12] = qppf[2];
		grafi[indx][13] = qd[0];
		grafi[indx][14] = qd[1];
		grafi[indx][15] = qd[2];

		indx++;
		grafFlag = grafSkip + 1;
	}

	grafFlag--;

	for (int i = 0; i < n; i++)
	{
		q_1[i] = q[i];
		qpf[i] += qpfp[i] * T;
		qppf[i] += qppfp[i] * T;
		qg[i] += qgp[i] * T;
		q2g[i] += q2gp[i] * T;
		z1g[i] += z1gp[i] * T;
		z2g[i] += z2gp[i] * T;
	}

	if (iCControl)
		iCControl = false;

	return;
}

double CPhantomDlg::sign(double x)
{
	return x == 0.0 ? 0.0 : x > 0.0 ? 1.0
									: -1.0;
}

double CPhantomDlg::sat(double x, double epsilon)
{
	return abs(x) <= epsilon ? x / epsilon : x > 0.0 ? 1.0
													 : -1.0;
}