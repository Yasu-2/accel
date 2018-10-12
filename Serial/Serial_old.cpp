// Serial.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// mbedから送られるデータの全長
#define DATA_LNG 22

HANDLE fd_mbed;
HDC hdc;
bool Ret;
char getstr1[8], getstr2[8];
char buff1[DATA_LNG] = { "0" };
char buff2[DATA_LNG] = { "0" };
char buff[DATA_LNG] = { "0" };

/* プロトタイプ宣言 */
int Getencdata(char *buff);


void mbed_setup() {
	//1.ポートをオープン
	fd_mbed = CreateFile(L"\\\\.\\COM21", GENERIC_WRITE | GENERIC_READ, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	if (fd_mbed == INVALID_HANDLE_VALUE)
	{
		printf("MBEDのポートをオープンできません。\n");
	}
	else {
		printf("MBEDのポートのオープンが完了しました.\n");
	}
	//2.送受信バッファ初期化
	Ret = SetupComm(fd_mbed, DATA_LNG, DATA_LNG);
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
int Getencdata() {
	char data[DATA_LNG];
	int flag = 0; //読み込みに成功したかのフラグ
	int i = 0;
	unsigned long nn;
	char *p;
	
	int n_adress;
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
		for (i = 0; i < DATA_LNG; i++) {
			//printf("data[%d] = %c\n", i, data[i]);
		}
		if (data[DATA_LNG - 2] != 13) {
			p = strchr(data, '\n');
			if (p != NULL) {
				n_adress = p - data;
				//printf("区切り文字は文字列の%d番目にあります。\n", n_adress);
				//printf("data: %.22s\n", data);
				//printf("文字列の検索結果: %s\n", p+1);
				strncpy_s(buff2, n_adress + 1, data, n_adress);
				//printf("buff1: %s\n", buff1);
				//printf("buff2: %s\n", buff2);
				strcat_s(buff1, DATA_LNG, buff2);
				//printf("buff1: %s\n", buff1);
				//printf("文字列の長さ %d\n", strlen(buff1));
				if (strlen(buff1) == DATA_LNG-1) {
					flag = 1;
					strcpy_s(buff, DATA_LNG, buff1);
				}else{
					flag = 0;
				}
				strncpy_s(buff1, DATA_LNG - n_adress , p + 1, DATA_LNG - n_adress -1);
			}
			else {
				//printf("区切り文字は見つかりませんでした。\n");
				//printf("data: %22s\n", data);
				flag = 0;
			}
		}
	}
	return flag;
}

// 仕様書にはvoid型で記述してあったけど、読み込み成功・失敗のフラグを返したほうが何かと便利な気がしたのでこうしてある
int Getangle(double *theta, double *dtheta, double *ddtheta) {
	char RecieveData[DATA_LNG] = { "0" };
	char count_char[3][9] = { "0" };
	double count_double[3] = { 0 };
	unsigned long nn;
	int flag = 0;
	int i = 0;
	flag = Getencdata();
	if (flag == 1) {
		//printf("count(buff):%s\n", buff);
		strncpy_s(count_char[0], 8, buff, 7);
		strncpy_s(count_char[1], 8, buff+7, 7);
		strncpy_s(count_char[2], 8, buff+14, 7);
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
	char on[1] = { 1 };
	char off[1] = { 0 };
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

	//mbedとの通信手段の確立
	mbed_setup();

	//関数Getangleの仕様上、初回呼び出し時に必ず読み込めないので予め呼び出しておく
	flag = Getangle(&theta, &dtheta, &ddtheta);

	//Ret = ReadFile(fd_mbed, data, 255, &nn, 0);// リセット
	risk(0);
	while (i <= 10) {
		flag = Getangle(&theta, &dtheta, &ddtheta);
		printf("%lf,%lf,%lf\n", theta, dtheta, ddtheta);
		i++;
		Sleep(200);
	}
	CloseHandle(fd_mbed);
}
