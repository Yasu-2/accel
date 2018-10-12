#include "stdafx.h"

#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
//#include <unistd.h>         //E
#include <math.h>
#include <fcntl.h>
#include <time.h>
//#include <termios.h>        //E
#include <string.h>
//#include <sys/time.h>		//E								// gettimeofday()用
//#include <pthread.h>		//E								// マルチスレッド用

//#define DEV_NAME_motor			"/dev/ttyUSB0"					// デバイスファイル名
#define BaudRate_motor 115200							// シリアル通信ボーレート(デフォルト:115.2kbps)
#define  COM_PORT_motor  L"\\\\.\\COM17"                                    // COMポート設定
//#define  SID  0x00                                           // ID設定
//#define timeout 10
#define PI					3.14159265
#define g					9.80665
#define DATA_MAX			60000
#define SERVO												// サーボモータの通信切替用


//int tio;
//int tio_old;
//struct termios	tio_old;									// 初期設定の保存用
HANDLE fd_motor;														// ファイルディスクリプタ
HDC hdc;
bool Ret; //mbedのセットアップとかで使ってる戻り値
// static double sampling_time = 0.01;							// 制御のサンプリングタイム[s]
DWORD	soft_start_time = 1000;			// サーボモータ起動時のソフトスタート時間[s, ns](デフォルト:1.0s)
DWORD	motor_period = 1.6;		// サーボモータの更新周期[s, s](デフォルト:0.5ms) (0, 1000000)(S, NS)

/* プロトタイプ宣言 */
//void end_accel();

/*----------------------制御関数------------------------------------------*/
/*==========================================================================*/

unsigned char motor_unlock[4] = { 0x80, 0x41, 0x14, 0x55 };				// モータのアンロック用
unsigned char motor_sw[4] = { 0x80, 0x41, 0x3b, 0x00 };					// モータのオン・オフ用
unsigned char motor_pgain[4] = { 0x80, 0x41, 0x32, 0x20 };				// モータのPゲイン(デフォルト:0x20 = 32)
unsigned char motor_igain[4] = { 0x80, 0x41, 0x35, 0x00 };				// モータのIゲイン(デフォルト:0x00 = 0)
unsigned char motor_dgain[4] = { 0x80, 0x41, 0x33, 0x20 };				// モータのDゲイン(デフォルト:0x20 = 32)
unsigned char motor_egain[4] = { 0x80, 0x41, 0x34, 0x04 };				// モータのEゲイン(デフォルト:0x04 = 4)
unsigned char motor_pwm_min[4] = { 0x80, 0x41, 0x3d, 0x00 };			// モータのPWM出力最小値(デフォルト:0x08 = 8)
unsigned char motor_pwm_max[4] = { 0x80, 0x41, 0x3c, 0x78 };			// モータのPWM出力最大値(デフォルト:0x78 = 120)
unsigned char motor_pos_deadband[4] = { 0x80, 0x41, 0x38, 0x00 };		// モータの角度に対する不感帯(デフォルト:0x08 = 8)
unsigned char motor_vel_deadband[4] = { 0x80, 0x41, 0x39, 0x00 };		// モータの角速度に対する不感帯(デフォルト:0x04 = 4)
unsigned char motor_cef_deadband[4] = { 0x80, 0x41, 0x3a, 0x00 };		// モータの逆起電力に対する不感帯(デフォルト:0x20 = 32)

int i, j, k;
int mode = 0;															// モード
int pre_sig = 0;														// "Preparation"フラグ

static double rom_limit[2] = { 0.0, 340.0 / 180.0 * PI };				// 関節可動域制限　なし

double pos_cmd = 0.0;													// 角度指令値[rad]
double pos_ref = 0.0;													// 負荷側角度の参照値[rad]
double pos_ref_old = 0.0;

double vel_ref = 0.0;													// 負荷側角速度の参照値[rad/s]
double vel_ref_old = 0.0;
static double vel_cut_freq = 20.0;										// 角速度算出用LPFのカットオフ角周波数[rad/s]

double vol_ref = 0.0;													// 電圧指令値[V]
double vol_ref_old = 0.0;
static double vol_cut_freq = 20.0;										// 電圧算出用LPFのカットオフ角周波数[rad/s]
double psv_ref = 0.0;													// 電源電圧参照値[V]
double pwm_out = 0.0;													// PWM出力値[%]
double cef_ref = 0.0;													// 逆起電力参照値[V]
double cef_ref_old = 0.0;
static double cef_cut_freq = 20.0;										// 逆起電力算出用LPFのカットオフ角周波数[rad/s]
double cur_cmd = 0.0;													// 電流指令値[A]

double trq_cmd = 0.0;													// 入力トルク[Nm]
double trq_f = 0.0;														// 摩擦力トルク[Nm]
double trq_g = 0.0;														// 重力トルク[Nm]
double trq_ext = 0.0;													// 反力トルク[Nm]

static double R = 5.26;													// 電機子抵抗[Ω](モータ固有値:1.78Ω)
static double k_t = 0.0104;												// トルク定数[Nm/A]
double k_e = 0.00320;													// 逆起電力定数[Vs/rad]
static double Gr = 260.0;												// 減速比

static double J_n = 0.035;												// 慣性モーメントのノミナル値[kgm^2]
static double M_robot = 0.65;											// ロボットの質量[kg]
static double L_robot = 0.135;											// 回転軸から重心までの距離[m]

double RFOB = 0.0;														// RFOB用の変数
double RFOB_old = 0.0;
static double RFOB_cut_freq = 20.0;										// RFOB用LPFのカットオフ角周波数[rad/s]

/*後から付け足した値*/
double Rev = 0.0; //各周波数
double tau_1 = 0.0;   //トルク推定器からのトルク
double tau_2 = 0.0; //反力推定オブザーバーの出力1つ手前のトルク
double pos_adm = 0.0;      //仮想インピーダンスを通り抜けた後の角度
double pos_adm_old = 0.0;
double pos_adm_old2 = 0.0;
double alpha = 0.0;     //仮想インピーダンスの計算に使う
double beta = 0.0;      //仮想インピーダンスの計算に使う
static double M_adm = 3.0;												// 仮想質量[kg]70
static double D_adm = 4.0;											// 仮想減衰係数[Ns/m]120
static double K_adm = 3.0;												// 仮想ばね定数[N/m]50


//アクセルの加速度を得るパラメータ
double pos_base;
double accel;        //アクセルの踏み込み量から出す速度
double accel_old;
double location = 0.0;					//アクセルからの角度から位置をsimulate
double  total_time = 0.01;				//計測時間
double pos_wave;

	// プロトタイプ
/*==========================================================================*/

// csvファイル用
time_t timer;												// 時刻取得用
struct tm	*local;											// 地方時変換用

/* csvファイル用 ******************************/
FILE *fp;															// ファイルポインタ
	char fn[256];															// ファイル名
char *file_header = "total_time[s],pos_cmd[deg],pos_ref[deg],vel_ref[deg/s],vol_ref[V],cef_ref[V],cur_cmd[A],trq_cmd[Nm],trq_ext[Nm],pos_adm[deg],location[m],pos_base[deg],accel[m/s]\n";
unsigned long l, csv = 0;												// csvファイルへの書き込み用
//int Mode[DATA_MAX];
double Total_time[DATA_MAX];
//double Tr_time[DATA_MAX];
double Pos_cmd[DATA_MAX];
double Pos_ref[DATA_MAX];
double Vel_ref[DATA_MAX];
double Vol_ref[DATA_MAX];
double Cef_ref[DATA_MAX];
double Cur_cmd[DATA_MAX];
double Trq_cmd[DATA_MAX];
double Trq_ext[DATA_MAX];
double Pos_adm[DATA_MAX];	//追加
double Location[DATA_MAX];	//追加
double Pos_base[DATA_MAX];
double Accel[DATA_MAX];	//追加
/**********************************************/


void serial_setup() {
	//1.ポートをオープン
	fd_motor = CreateFile(COM_PORT_motor, GENERIC_WRITE | GENERIC_READ, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	if (fd_motor == INVALID_HANDLE_VALUE)
	{
		printf("PORT OPEN ERROR\n");
	}
	else {
		printf("PORT OPEN SUCCEEDED\n");
	}
	//2.送受信バッファ初期化
	Ret = SetupComm(fd_motor, 64, 64);
	if (!Ret)
	{
		printf("SetupComm ERROR\n");
		CloseHandle(fd_motor);
	}
	else {
		printf("SetupComm SUCCEEDED\n");
	}
	Ret = PurgeComm(fd_motor, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);
	if (!Ret) {
		printf("PurgeComm ERROR\n");
		CloseHandle(fd_motor);
	}
	else {
		printf("PurgeComm SUCCEEDED\n");
	}
	//基本通信条件の設定
	DCB dcb;
	GetCommState(fd_motor, &dcb);
	dcb.DCBlength = sizeof(DCB);
	dcb.BaudRate = BaudRate_motor;
	dcb.fBinary = FALSE;
	dcb.ByteSize = 8;
	dcb.fParity = NOPARITY;
	dcb.StopBits = ONESTOPBIT;

	Ret = SetCommState(fd_motor, &dcb);

	if (!Ret) {
		printf("SetCommState ERROR\n");
		CloseHandle(fd_motor);
	}
	else {
		printf("SetCommState SUCCEEDED\r\n");
	}
}

void serial_close() {
	if (fd_motor > 0) {
		CloseHandle(fd_motor);
		printf("MOTOR CLOSED\n");
	}
}

void finish_accel(void) {
	unsigned long bytes;

	//Finalモータオフ
	motor_sw[3] = 0x00;
	WriteFile(fd_motor, motor_sw, sizeof(motor_sw), &bytes, NULL);		//Change to writeFile
	Sleep(motor_period);          //Change it!

	// シリアルポートを初期設定に戻す
	DCB dcb;
	GetCommState(fd_motor, &dcb);
	dcb.DCBlength = sizeof(DCB);
	dcb.BaudRate = 115200;
	dcb.fBinary = FALSE;
	dcb.ByteSize = 8;
	dcb.fParity = NOPARITY;
	dcb.StopBits = ONESTOPBIT;
	Ret = SetCommState(fd_motor, &dcb);

	serial_close();	//シリアルポートclose
}

/*CSVの書き込み関数*/                                                          //CSVファイルの書き込みを何度か行うので関数化
void CSV(void) {
	printf("Data write start\n");
	for (l = 0; l < csv; l++) {
		//fprintf(fp, "%d, ", Mode[l]);
		fprintf(fp, "%lf,", Total_time[l]);
		fprintf(fp, "%lf, %lf, %lf, ", Pos_cmd[l], Pos_ref[l], Vel_ref[l]);
		fprintf(fp, "%lf, %lf, %lf, ", Vol_ref[l], Cef_ref[l], Cur_cmd[l]);
		fprintf(fp, "%lf, %lf, ", Trq_cmd[l], Trq_ext[l]);
		fprintf(fp, "%lf,", Pos_adm[l]);//追加
		fprintf(fp, "%lf,", Location[l]);//追加
		fprintf(fp, "%lf,", Pos_base[l]);//追加
		fprintf(fp, "%lf,", Accel[l]);//追加
		fprintf(fp, "\n");
	}
	fclose(fp);
	printf("Data write finish\n");
}
/* LPF */
double LPF(double old_value, double input_value, double cut_freq, double sampling_time)
{
	double LPF_value;

	LPF_value = old_value + (input_value - old_value) * (1.0 - exp(-cut_freq * sampling_time));

	return LPF_value;
}


/* シリアルポートの初期化 */
/*void serial_init(int fd)
{
	struct termios	tio;		//構造体の初期化がまだ、、データは何？
	int CS8, CLOCAL, CREAD, VTIME, TCSANOW;
	

	// 初期設定の保存
	tcgetattr(fd, &tio_old);
	// 設定の初期化
	memset(&tio, 0, sizeof(tio));
	tio.c_cflag = CS8 | CLOCAL | CREAD;
	tio.c_cc[VTIME] = 100;
	// ボーレートの設定
	cfsetispeed(&tio, BAUD_RATE);
	cfsetospeed(&tio, BAUD_RATE);
	// デバイスに設定を反映
	tcsetattr(fd, TCSANOW, &tio);
}*/


/* 度数をラジアンに変換 */
double deg_to_rad(double deg) {
	return (deg * PI / 180.0);
}

/* ラジアンを度数に変換 */
double rad_to_deg(double rad) {
	return (rad * 180.0 / PI);
}

/* サーボモータ関連 ******************************/
/* 角度指令値の入力 */
void input_pos_cmd(double pos_cmd, HANDLE fd_motor)
{
	unsigned char motor[5];
	int motor_pos_cmd;								// 角度指令値[-]
	unsigned long bytes;

	motor_pos_cmd = (int)(pos_cmd * 4096 / (340.0 / 180.0 * PI));

	printf("motor_pos_cmd: %d\n", motor_pos_cmd);

	motor[0] = 0x80;
	motor[1] = 0x00 | 0x40 | 2;						// 書き込み(2byte)
	motor[2] = 0x30;								// 角度指令値のアドレス
	motor[3] = (motor_pos_cmd >> 0) & 0x7f;			// 角度指令値(low byte)
	motor[4] = (motor_pos_cmd >> 7) & 0x7f;			// 角度指令値(high byte)
	WriteFile(fd_motor, motor, sizeof(motor), &bytes, NULL);
	Sleep(motor_period);
	printf("motor_pos_cmd write\n");
}

/* 負荷側角度の参照値を取得 */
double get_pos_ref(HANDLE fd_motor)                      //サーボモータの角度
{
	unsigned char motor[5];
	unsigned char motor_pos_ref[2];		// 負荷側角度の参照値[-]
	double pos_ref;						// 負荷側角度の参照値[rad]
	unsigned long bytes;

	motor[0] = 0x80;
	motor[1] = 0x00 | 0x20 | 2;			// 読み込み(2byte)
	motor[2] = 0x20;					// 負荷側角度の参照値のアドレス
	motor[3] = 0x00;					// Readのためのダミーバイト
	motor[4] = 0x00;					// Readのためのダミーバイト
	WriteFile(fd_motor, motor, sizeof(motor), &bytes, NULL);
	Sleep(motor_period);
	ReadFile(fd_motor, motor_pos_ref, 2, &bytes, NULL);
	Sleep(motor_period);

	// 取得データをラジアンに変換
	pos_ref = (double)(((motor_pos_ref[1] << 7) + motor_pos_ref[0]) * 340.0 / 180.0 * PI / 4096.0);

	return pos_ref;
}

#ifdef SERVO
/* 電源電圧参照値の取得 */
double get_psv_ref(HANDLE fd_motor)
{
	unsigned char motor[5];
	unsigned char motor_psv_ref[2];		// 電源電圧参照値[-]
	double psv_ref;						// 電源電圧参照値[V]
	unsigned long bytes;

	motor[0] = 0x80;
	motor[1] = 0x00 | 0x20 | 2;			// 読み込み(2byte)
	motor[2] = 0x28;					// 電源電圧参照値のアドレス
	motor[3] = 0x00;					// Readのためのダミーバイト
	motor[4] = 0x00;					// Readのためのダミーバイト
	WriteFile(fd_motor, motor, sizeof(motor), &bytes, NULL);
	Sleep(motor_period);
	ReadFile(fd_motor, motor_psv_ref, 2, &bytes, NULL);
	Sleep(motor_period);

	// 取得データを電圧に変換
	psv_ref = (double)(27.5 * ((motor_psv_ref[1] << 7) + motor_psv_ref[0]) / 4096.0);

	return psv_ref;
}

/* PWM出力値の取得 */
double get_pwm_out(HANDLE fd_motor, double pwm_min, double pwm_max)
{
	unsigned char motor[5];
	unsigned char motor_pwm_out[2];		// PWM出力値[-]
	unsigned char pwm;
	double pwm_out;						// PWM出力値[%]
	int sign;							// 符号判定用
	unsigned long bytes;

	motor[0] = 0x80;
	motor[1] = 0x00 | 0x20 | 2;			// 読み込み(2byte)
	motor[2] = 0x3e;					// PWM出力値のアドレス
	motor[3] = 0x00;					// Readのためのダミーバイト
	motor[4] = 0x00;					// Readのためのダミーバイト
	WriteFile(fd_motor, motor, sizeof(motor), &bytes, NULL);
	Sleep(motor_period);
	ReadFile(fd_motor, motor_pwm_out, 2, &bytes, NULL);
	Sleep(motor_period);

	// 符号判定
	sign = (int)(motor_pwm_out[1] >> 6);

	// 取得データをパーセントに変換
	if (sign == 0) {
		pwm = (motor_pwm_out[1] << 7) + motor_pwm_out[0];

		// PWM出力値の最大値と最小値を制限
		if (pwm < pwm_min && pwm != 0x00)
			pwm = pwm_min;
		else if (pwm > pwm_max)
			pwm = pwm_max;

		pwm_out = (double)(pwm / 128.0);
	}
	else {
		pwm = ~((motor_pwm_out[1] << 7) + motor_pwm_out[0]);
		pwm += 1;

		// PWM出力値の最大値と最小値を制限
		if (pwm < pwm_min && pwm != 0x00)
			pwm = pwm_min;
		else if (pwm > pwm_max)
			pwm = pwm_max;

		pwm_out = (double)((-pwm) / 128.0);
	}

	return pwm_out;
}

/* 逆起電力参照値の取得 */
double get_cef_ref(HANDLE fd_motor)
{
	unsigned char motor[5];
	unsigned char motor_cef_ref[2];		// 逆起電力参照値[-]
	unsigned char cef;
	double cef_ref;						// 逆起電力参照値[V]
	int sign;							// 符号判定用
	unsigned long bytes;

	motor[0] = 0x80;
	motor[1] = 0x00 | 0x20 | 2;			// 読み込み(2byte)
	motor[2] = 0x24;					// 逆起電力参照値のアドレス
	motor[3] = 0x00;					// Readのためのダミーバイト
	motor[4] = 0x00;					// Readのためのダミーバイト
	WriteFile(fd_motor, motor, sizeof(motor), &bytes, NULL);
	Sleep(motor_period);
	ReadFile(fd_motor, motor_cef_ref, 2, &bytes, NULL);
	Sleep(motor_period);

	// 符号判定
	sign = (int)(motor_cef_ref[1] >> 6);
	// 取得データを電圧に変換
	if (sign == 0) {
		cef = (motor_cef_ref[1] << 7) + motor_cef_ref[0];
		cef_ref = (double)(27.5 * cef / 4096.0);
	}
	else {
		cef = ~((motor_cef_ref[1] << 7) + motor_cef_ref[0]);
		cef_ref = (double)(27.5 * (-cef + 1) / 4096.0);
	}

	return cef_ref;
}
#endif

/*************************************************/

/*accel関数の準備を行う関数*/
int ready_accel(char *argv[]){
	unsigned long bytes;
	/*********************************************/
	// デバイスファイル(シリアルポート)をオープン
	//fd = open(DEV_NAME, O_RDWR);
	// デバイスの open() に失敗した場合
	//if (fd < 0) {
		//perror(argv[1]);
		//printf("Open error\n");
		//exit(1);
	//}
	/*********************************************/
	// シリアルポートの初期化
	//serial_open(COMPORT, /*BaudRate*/CBR_115200, 10);		//シリアルポートopen
	serial_setup();
	//serial_init(fd);

	// モータのアンロック処理
	WriteFile(fd_motor, motor_unlock, sizeof(motor_unlock), &bytes, NULL);
	Sleep(motor_period);

	// 負荷側角度の初期値を取得
	pos_ref = get_pos_ref(fd_motor);
	printf("pos_ini = %f [rad]\n" , pos_ref);
	pos_ref_old = pos_ref;   //ループ1回目は現在と過去の値は同じ
	// 負荷側角度の初期値が関節可動域制限を超えていた場合
	if (rom_limit[0] >= pos_ref || pos_ref >= rom_limit[1]) {
		printf("Error < pos_ini >\n");
		return -1;
	}

	// 現在時刻の取得
	timer = time(NULL);
	// 地方時に変換
	local = localtime(&timer);

	// csvファイルの作成
	sprintf(fn, "%d_%d_%d_%d_%d.csv", local->tm_year + 1900, local->tm_mon + 1, local->tm_mday, local->tm_hour, local -> tm_min);
	fp = fopen(fn, "w");
	fprintf(fp, file_header);


	// 負荷側角度の初期値を角度指令値に設定
	pos_base = pos_ref;

	//アクセルの踏み込み角度を得る
    //accel_1 = pos_ref - pos_base;
    //accel_1_old = accel_1;
	accel = pos_ref - pos_base;
	accel_old = accel;

	// サーボモータへの角度指令
	input_pos_cmd(pos_base, fd_motor); // ここでpos_cmd = 0.0 に指令位置を出すのは正しい？

	// 過去データの保存
	pos_ref_old = pos_ref;

	/* サーボモータの設定変更 ******************************/
	// Pゲインを変更
	WriteFile(fd_motor, motor_pgain, sizeof(motor_pgain), &bytes, NULL);		//writeFile
	Sleep(motor_period);
	// Iゲインを変更RFOB_cut_RFOB_cut_freqfreq
	WriteFile(fd_motor, motor_igain, sizeof(motor_igain), &bytes, NULL);
	Sleep(motor_period);
	// Dゲインを変更K_adm
	WriteFile(fd_motor, motor_dgain, sizeof(motor_dgain), &bytes, NULL);
	Sleep(motor_period);
	// Eゲインを変更
	WriteFile(fd_motor, motor_egain, sizeof(motor_egain), &bytes, NULL);
	Sleep(motor_period);

	// PWM出力最小値を変更
	WriteFile(fd_motor, motor_pwm_min, sizeof(motor_pwm_min), &bytes, NULL);
	Sleep(motor_period);
	// PWM出力最大値を変更
	WriteFile(fd_motor, motor_pwm_max, sizeof(motor_pwm_max), &bytes, NULL);
	Sleep(motor_period);

	// 角度に対する不感帯を変更
	WriteFile(fd_motor, motor_pos_deadband, sizeof(motor_pos_deadband), &bytes, NULL);
	Sleep(motor_period);
	// 角速度に対する不感帯を変更
	WriteFile(fd_motor, motor_vel_deadband, sizeof(motor_vel_deadband), &bytes, NULL);
	Sleep(motor_period);
	// 逆起電力に対する不感帯を変更
	WriteFile(fd_motor, motor_cef_deadband, sizeof(motor_cef_deadband), &bytes, NULL);		//writeFile
	Sleep(motor_period);			//Sleep
	/*******************************************************/

	// モータオン
	motor_sw[3] = 0x01;
	WriteFile(fd_motor, motor_sw, sizeof(motor_sw), &bytes, NULL);	//writeFile
	Sleep(soft_start_time);		//Sleep
	return 0;
}

/*アクセルの踏み込み量から角度を求める関数*/
double getaccel(double sampling_time){                      //サンプリングタイムをもらうとアクセルの踏み込み角度を算出
	unsigned long bytes;
	printf("getaccel start\n");
	/* サーボモータの制御 ******************************/

	// 初期姿勢(135deg)まで回転
	if (pre_sig == 0) {
		if (0.0 <= pos_base && pos_base <= 135.0 / 180.0 * PI)
			pos_base += PI / 900.0;
		else if (135.0 / 180.0 * PI < pos_base && pos_base <= 340.0 / 180.0 * PI)                              //持っていきたい角度にゆっくり変更
			pos_base -= PI / 900.0;
		if (134.0 / 180.0 * PI <= pos_base - PI / 900.0 && pos_base - PI / 900.0 <= 136.0 / 180.0 * PI) {     //持っていきたい角度に変更
			pre_sig = 1;
		}
	}
	else
		pos_base = 135.0 / 180.0 * PI;       //pre_sig

	printf("a\n");

	// 角度指令値が関節可動域制限を超えていた場合
	if (rom_limit[0] >= pos_ref || pos_ref >= rom_limit[1]) {
		printf("Error < pos_cmd >\n");
		// モータオフ1
		motor_sw[3] = 0x00;
		WriteFile(fd_motor, motor_sw, sizeof(motor_sw), &bytes, NULL);	//Write
		Sleep(motor_period);			//Sleep
	}
	printf("b\n");
	// 負荷側角度の参照値が関節可動域制限を超えていた場合
	if (rom_limit[0] >= pos_ref || pos_ref >= rom_limit[1]) {
		printf("Error < pos_ref >\n");
		// モータオフ2
		motor_sw[3] = 0x00;
		WriteFile(fd_motor, motor_sw, sizeof(motor_sw), &bytes, NULL);	//writeFile
		Sleep(motor_period);			//Sleep
	}
	printf("c\n");
#ifdef SERVO
	// 電源電圧参照値の取得
	psv_ref = get_psv_ref(fd_motor);

	// PWM出力値の取得
	pwm_out = get_pwm_out(fd_motor, motor_pwm_min[3], motor_pwm_max[3]);

	// 逆起電力参照値の取得
	cef_ref = get_cef_ref(fd_motor);
	cef_ref = LPF(cef_ref_old, cef_ref, cef_cut_freq, sampling_time);
#endif
	printf("d\n");
	/***************************************************/

	// 負荷側角速度の参照値を算出
	vel_ref = (pos_ref - pos_ref_old) / sampling_time;
	vel_ref = LPF(vel_ref_old, vel_ref, vel_cut_freq, sampling_time);

	// 逆起電力定数の選択
	if (vel_ref >= 0)
		k_e = 0.00333;
	else
		k_e = 0.00326;

	/************************* ココから **************************/

    //反力推定オブザーバ 外乱オブザーバ
    Rev = vel_ref * RFOB_cut_freq * J_n;
    tau_1 = Rev + trq_cmd - trq_f - trq_g;
    tau_2 = LPF(tau_2, tau_1, RFOB_cut_freq, sampling_time);
    trq_ext = tau_2 - Rev;

	printf("trq_ext %f\n", trq_ext);


    //仮想インピーダンス
    trq_ext = -1 * trq_ext;     //反力と力の向きが逆
    alpha = K_adm + (D_adm / sampling_time) + (M_adm / (sampling_time * sampling_time));
    beta = trq_ext + (D_adm * pos_adm_old / sampling_time) + (M_adm * (2*pos_adm_old - pos_adm_old2) / (sampling_time * sampling_time));
    pos_adm = beta / alpha;
	// 反力に従って角度指令値を変化させる                                //元の角度司令値をPos_bace、サーボモータに送る指令値をpos_com
    pos_cmd = pos_base + pos_adm;
	//pos_cmd = pos_base;		//アドミタンス制御を無効にするとき
	


	/************************* ココまで **************************/

	// 角度指令値が関節可動域制限を超えていた場合
	if (rom_limit[0] >= pos_ref || pos_ref >= rom_limit[1]) {
		printf("Error < pos_cmd >\n");
		// モータオフ3
		motor_sw[3] = 0x00;
		WriteFile(fd_motor, motor_sw, sizeof(motor_sw), &bytes, NULL);	//change to writeFile
		Sleep(motor_period);			//change to Sleep
	}

	// サーボモータへの角度指令
	input_pos_cmd(pos_cmd, fd_motor);

#ifdef SERVO
	// 電圧指令値の推定
	vol_ref = psv_ref * pwm_out;
	vol_ref = LPF(vol_ref_old, vol_ref, vol_cut_freq, sampling_time);

	// 電流指令値の推定
	cur_cmd = (vol_ref + cef_ref) / R;
#else
	// 逆起電力参照値の推定
	cef_ref = Gr * k_e * cur_cmd;

	// 電流指令値の推定
	cur_cmd = (-motor_dgain[3] * (motor_pgain[3] * (adm_cmd - pos_ref) - vel_ref) + motor_egain[3] * cef_ref) / 27.5;
#endif

	// 入力トルクの推定
	trq_cmd = Gr * k_t * cur_cmd;

	// 摩擦力トルクの算出                                            //摩擦トルクを違う不連続摩擦モデルにて実現しよう
	if (vel_ref < -0.0175)
		trq_f = 1.243 * vel_ref - 0.352;
	else if (vel_ref > 0.0175)
		trq_f = 1.209 * vel_ref + 0.349;
	else
		trq_f = 0.0;

    // 重力トルクの算出
    trq_g = M_robot * g * L_robot * cos(pos_ref + 0.825);       //トルクの向きや固有の角度の位置を考慮する必要あり、LM、角度を測定する必要あり

	accel = pos_ref - pos_base;
    accel = LPF(accel_old, accel, vel_cut_freq, sampling_time);       //誤差を小さくするためにローパスにかける

	/************************* ココから **************************/
	// ○○○_old に ○○○ を代入
    pos_adm_old2 = pos_adm_old;
    pos_adm_old = pos_adm;
    pos_ref_old = pos_ref;
	vel_ref_old = vel_ref;
	vol_ref_old = vol_ref;
	cef_ref_old = cef_ref;
	accel_old = accel;
	RFOB_old = RFOB;

	/************************* ココまで **************************/
	// データの保存（変数[csv]に今の値を代入してcsvをインクリメント）
	if (csv < DATA_MAX) {
		
		Total_time[csv] = total_time;
		//Tr_time[csv] = Tr_time;
		Pos_cmd[csv] = pos_cmd * 180.0 / PI;
		Pos_ref[csv] = pos_ref * 180.0 / PI;
		Vel_ref[csv] = vel_ref;
		Vol_ref[csv] = vol_ref;
		Cef_ref[csv] = cef_ref;
		Cur_cmd[csv] = cur_cmd;
		Trq_cmd[csv] = trq_cmd;
		Trq_ext[csv] = trq_ext;
		Pos_adm[csv] = pos_adm * 180.0 / PI;
		Location[csv] = location;
		Pos_base[csv] = pos_base * 180.0 / PI;
		Accel[csv] = accel;
		csv++;
	}
		
	// 無限ループ終了
    return accel;
}

/*シリアルポートの関数化*/
/*
void end_accel(){
	unsigned long bytes; // WriteFile,ReadFileに使用。
    //Finalモータオフ
    motor_sw[3] = 0x00;
    WriteFile(fd_motor, motor_sw, sizeof(motor_sw), &bytes, NULL);		//Change to writeFile
    Sleep(motor_period);          //Change it!

	// シリアルポートを初期設定に戻す
	HANDLE hCom;
	DCB    stDcb;
	//tcsetattr(fd, TCSANOW, &tio_old);
	GetCommState(hCom, &stDcb);				//シリアルポート初期化する関数
	serial_close(fd_motor);						//シリアルポートclose
	//close(fd);
}
*/

/*void serial_close(HANDLE hCom)
{
	if (hCom && hCom != (HANDLE)-1) {
		CloseHandle(hCom);
	}
}*/

/*double get_location(double sampling_time, double accel) {
	double n, a = 0.0, dx, location, i;

	n = 2.0;		//分割数
	dx = (sampling_time - a) / n;

	//for (i = a; i < sampling_time; i += dx) {
		location = accel * dx;
	//}

	//printf("location %f\n", location);

	return location;
}*/

// 以下組込例(全体の制御に組み込むときは以下をコメントアウト)
int main(char *argv[])						// 引数として *argv[] を持ってくる
{
	double Ts = 0.01;						// サンプリングタイム
	double value_accel = 0.0;				// getaccelの戻り値を受け取る変数
	int loop = 0;							// 無限ループ数（蛇足）
	

	Ret = ready_accel(argv);						// 無限ループ前に引数 argv で実行
	if (Ret != -1) {
		// 無限ループ
		while (1)
		{
			value_accel = getaccel(Ts);			// 無限ループ内で引数(サンプリングタイム)で実行
			// 無限ループの脱出（蛇足）
			location += value_accel * Ts;	//位置情報を数値積分より算出
			pos_wave = pos_base * sin(0.02 *total_time);		//pos_basenに振動加える
			total_time = total_time + Ts;
			loop++;
			if (loop > 400)
				break;
		}
	}
	CSV();									// 毎ループcsvに書き込んでいると処理に時間がかかるので、最後にまとめてやる
	finish_accel();							// 無限ループ後に引数無しで実行

	return 0;
}
