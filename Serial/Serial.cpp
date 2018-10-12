// Serial.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// mbedから送られるデータの全長
#define DATA_LNG 25
#define DATA_BUFF 64

HANDLE fd_mbed;
HDC hdc;
bool Ret;
char getstr1[8], getstr2[8];
char buff1[DATA_BUFF];
char buff2[DATA_BUFF];

/* プロトタイプ宣言 */
int Getencdata(char *RecieveData);


void mbed_setup() {
	//1.ポートをオープン
	fd_mbed = CreateFile(L"\\\\.\\COM22", GENERIC_WRITE | GENERIC_READ, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	if (fd_mbed == INVALID_HANDLE_VALUE)
	{
		printf("MBEDのポートをオープンできません。\n");
	}
	else {
		printf("MBEDのポートのオープンが完了しました.\n");
	}
	//2.送受信バッファ初期化
	Ret = SetupComm(fd_mbed, 64, 64);
	if (!Ret)
	{
		printf("送受信バッファの設定に失敗しました.\n");
		CloseHandle(fd_mbed);
	}
	else {
		printf("送受信バッファの設定が完了しました.\r\n");
	}
	Ret = PurgeComm(fd_mbed, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);
	if (!Ret) {
		printf("送受信バッファの初期化に失敗しました.\n");
		CloseHandle(fd_mbed);
	}
	else {
		printf("送受信バッファの初期化が完了しました.\r\n");
	}
	//基本通信条件の設定
	DCB dcb;
	GetCommState(fd_mbed, &dcb);
	dcb.DCBlength = sizeof(DCB);
	dcb.BaudRate = 9600;
	dcb.fBinary = FALSE;
	dcb.ByteSize = 8;
	dcb.fParity = NOPARITY;
	dcb.StopBits = ONESTOPBIT;

	Ret = SetCommState(fd_mbed, &dcb);

	if (!Ret) {
		printf("MBEDの基本通信条件の設定に失敗しました.\n");
		CloseHandle(fd_mbed);
	}
	else {
		printf("MBEDの基本通信条件の設定が完了しました.\r\n");
	}
}

void mbed_close() {
	if (fd_mbed > 0) {
		printf("MBED CLOSED\n");
		CloseHandle(fd_mbed);
	}
}

// エンコーダから文字列を読み取る関数
// 戻り値：読み込みに成功したかのフラグ
int Getencdata(char *RecieveData) {
	char data[64];
	data[0] = '\0';
	char request[1] = { 's' };
	int flag = 0; //読み込みに成功したかのフラグ
	int i = 0;
	unsigned long nn;
	char *p;
	
	int n_adress;
	WriteFile(fd_mbed, request, 1, &nn, 0);
	Ret = ReadFile(fd_mbed, data, DATA_LNG, &nn, 0);
	//Ret = fscanf(fd_mbed, "%d%d", getstr1, getstr2);
	//printf("nn:%d\n", nn);
	//printf("char1:%s\n", getstr1);
	//printf("char2:%s\n", getstr2);
	if (!Ret) {
		printf("MBED READ FAILED\n");
		flag = -1;
	}
	else {
		flag = 1;
		// 受け取ったデータの表示
		for (i = 0; i < DATA_LNG+1; i++) {
			printf("data[%d] = %c(%d)\n", i, data[i], data[i]);
		}
		
		// 区切り文字の場所を検索
		p = strchr(data, '\n');
		if (p != NULL) {
			n_adress = p - data;
			if (n_adress == 24) {
				data[DATA_LNG] = '\0'; //NULL終端
				//printf("RecieveData の文字列の長さ %d\n", strlen(RecieveData));
				//printf("data の文字列の長さ %d\n", strlen(data));
				strcpy_s(RecieveData, DATA_LNG + 1, data);
			}
			else if (n_adress == 0) {
				strncpy_s(buff1, DATA_LNG - n_adress, p + 1, DATA_LNG - n_adress - 1);//区切り文字の後ろの文字列を格納
				strcpy_s(RecieveData, DATA_LNG + 1, buff1);
			}
			else {
				//printf("buff1: %s\n", buff1);
				strncpy_s(buff2, n_adress + 1, data, n_adress);//区切り文字の手前の文字列を格納
				//printf("buff2: %s\n", buff2);
				strcat_s(buff1, DATA_LNG + 1, buff2);//前回のループで取得した区切り文字の後ろの文字列と結合
				//printf("buff: %s\n", buff1);
				if (strlen(buff1) >= DATA_LNG - 1) {
					flag = 1;
					buff1[DATA_LNG] = '\0'; //NULL終端
					strcpy_s(RecieveData, DATA_LNG + 1, buff1);
				}
				else {
					flag = 0;
				}
				strncpy_s(buff1, DATA_LNG - n_adress, p + 1, DATA_LNG - n_adress - 1);//区切り文字の後ろの文字列を格納
			}
			
		}
		else {
			//printf("区切り文字は見つかりませんでした。\n");
			//printf("data: %22s\n", data);
			flag = 0;
		}
	}
	return flag;
}

// 仕様書にはvoid型で記述してあったけど、読み込み成功・失敗のフラグを返したほうが何かと便利な気がしたのでこうしてある
int Getangle(double *theta, double *dtheta, double *ddtheta) {
	char RecieveData[64] = {};
	char count_char[3][9] = { "0" };
	double count_double[3] = { 0 };
	unsigned long nn;
	int flag = 0;
	int i = 0;
	flag = Getencdata(RecieveData);
	if (flag == 1) {
		//printf("strncpy_s\n");
		//printf("count(buff):%s\n", buff);
		strncpy_s(count_char[0], 9, RecieveData, 8);
		strncpy_s(count_char[1], 9, RecieveData +8, 8);
		strncpy_s(count_char[2], 9, RecieveData +16, 8);
		for (i = 0; i < 3; i++) {
			count_double[i] = atof(count_char[i]);
		}
		*theta = count_double[0];
		*dtheta = count_double[1];
		*ddtheta = count_double[2];
	}
	return flag;
}

void risk(int risk) {
	char on[1] = { '1' };
	char off[1] = { '0' };
	unsigned long nn;
	if (risk == 1) {
		WriteFile(fd_mbed, on, 1, &nn, 0);
	}else{
		WriteFile(fd_mbed, off, 1, &nn, 0);
	}
}
	

// mbedから送られてくるエンコーダの情報を無限ループで読み続けるプログラム
void main(void) {
	char data[255];
	char RecieveData[DATA_LNG] = {"0"};
	unsigned long nn;
	int i=0;
	int flag = 0; // 読み込み成功・失敗のフラグ
	double theta = 0; // 角度
	double dtheta = 0; // 角速度
	double ddtheta = 0; // 角加速度

	buff1[0] = '\0';
	buff2[0] = '\0';

	//mbedとの通信手段の確立
	mbed_setup();

	//関数Getangleの仕様上、初回呼び出し時に必ず読み込めないので予め呼び出しておく
	flag = Getangle(&theta, &dtheta, &ddtheta);

	//Ret = ReadFile(fd_mbed, data, 255, &nn, 0);// リセット
	risk(0);
	while (1) {
		flag = Getangle(&theta, &dtheta, &ddtheta);
		printf("flag = %d: %lf,%lf,%lf\n", flag, theta, dtheta, ddtheta);
		i++;
		Sleep(100);
	}
	CloseHandle(fd_mbed);
}
