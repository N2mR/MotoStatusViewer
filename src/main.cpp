#include <M5Core2.h>
#include <map>
#include <Kalman.h>
#include <cstdint>
#include <cmath>
#include <BluetoothSerial.h>
#include <sstream>
#include <../.pio/libdeps/m5stack-core2/ArduinoJson/ArduinoJson.h>

 enum class DisplayType
{
	Lean,
	CAN
};


// 機能切り替え
DisplayType enmDisplayType = DisplayType::Lean;
bool blneedFillBlack = false;
// IMU
//x, y, zの順
float acc[3];
float accOffset[3];
float gyro[3];
float gyroOffset[3];
float kalAngleX;
float kalAngleY;
Kalman kalmanX;
Kalman kalmanY;
long lastMs = 0;
uint16_t tick = 0;
uint8_t maxRightLean = 0;
uint8_t maxLeftLean = 0;
// Bluetooth
BluetoothSerial SerialBT;
String strMasterName = "M5Stack_master";
String strSlaveName = "M5StickC";
bool blConnect = false;
String data = "";

// GUI
bool drawAngleIndicator(float kalmanY);
bool drawMaxLean();
// Angle
void draw();
void readGyro();
void calibration();
void applyCalibration();
float getRoll();
float getPitch();

// CANData
bool drawThrottleAngle();

void setup() {
  	M5.begin();
	Serial.begin(9600);
	M5.Lcd.fillScreen(BLACK);
	// IMU
	M5.IMU.Init();
	delay(500);  
	calibration();
	readGyro();
	kalmanX.setAngle(getRoll());
	kalmanY.setAngle(getPitch());

	// バイクに取り付けるため振動に考慮
	kalmanX.setQangle(0.003);
	kalmanX.setQbias(0.005);
	kalmanX.setRmeasure(0.05);

	kalmanY.setQangle(0.003);
	kalmanY.setQbias(0.005);
	kalmanY.setRmeasure(0.05);

	lastMs = micros();
	// Bluetooth
	SerialBT.begin(strMasterName, true);
}

void loop() {
	tick++;
	// ボタンによって表示内容を切り分ける
	M5.update();
	if (M5.BtnA.wasPressed())
	{
		enmDisplayType = DisplayType::Lean;
		blneedFillBlack = true;
	} 
	else if(M5.BtnB.wasPressed())
	{
		enmDisplayType = DisplayType::CAN;
		blneedFillBlack = true;
	}
	else{
		// 何もしない
	}

	// 機能を切り替える場合、前機能の画面をリセットする必要がある
	// リセットのフラグが立っている場合、画面を黒で塗りつぶす
	if (blneedFillBlack)
	{
		M5.Lcd.fillScreen(BLACK);
		blneedFillBlack = false;
	}
	else
	{
		// 何もしない
	}

	// 機能ごとに処理を分ける
	switch (enmDisplayType)
	{
		case DisplayType::Lean:		// 傾きを表示する

			{
				//10回に1回だけ描画
				if(tick % 10 == 0){

					// 左右の傾きを取得
					readGyro();
					applyCalibration();
					float dt = (micros() - lastMs) / 1000000.0;
					lastMs = micros();
					float roll = getRoll();
					float pitch = getPitch();
					
					kalAngleX = kalmanX.getAngle(roll, gyro[0], dt);
					kalAngleY = kalmanY.getAngle(pitch, gyro[1], dt);

					// 左右の傾きをディスプレイ表示
					drawMaxLean();
					drawAngleIndicator(kalAngleY);

				}
			}
			break;
		case DisplayType::CAN:
			{
				// slave(M5StickC)へ接続
				if (!blConnect)
				{
					if (tick % 100 == 0)
					{
						blConnect = SerialBT.connect(strSlaveName);
						if (blConnect)
						{
							M5.Lcd.fillScreen(BLACK);
						}
						else
						{
							M5.Lcd.fillScreen(BLACK);
							M5.Lcd.setTextSize(2);
							M5.Lcd.setCursor(50, 20);
							M5.Lcd.printf("Not Connected.");
						}
					}
				}

				if (SerialBT.available())
				{
					std::string recv = "";
					while (SerialBT.available())
					{
						recv += (char)SerialBT.read();
					}
					
					char delm = ' ';

					const char* charRecv = recv.c_str();

					StaticJsonDocument<200> doc;
					deserializeJson(doc, charRecv);

					const char* rpm = doc["rpm"];
					const char* throttle = doc["throttle"];

					String RPM = "RPM: ";
					String Throttle = "Throttle: ";
					M5.Lcd.setTextSize(2);
					M5.Lcd.setCursor(50, 70);
					M5.Lcd.println(RPM);
					M5.Lcd.setCursor(50, 140);
					M5.Lcd.println(Throttle);

					M5.Lcd.setTextSize(3);
					M5.Lcd.setCursor(140, 65);
					M5.Lcd.fillRect(140, 65, 100, 25, BLACK);
					M5.Lcd.println(String(rpm));
					M5.Lcd.setCursor(180, 135);
					M5.Lcd.fillRect(180, 135, 100, 25, BLACK);
					M5.Lcd.println(String(throttle));

				}
			}
			break;
		default:
			break;
	}
}

/*
* GUI
*/
bool drawAngleIndicator(float kalAngleY)
{
	const uint16_t warnAngle = 30;		// 角度(注意)
	const uint16_t dangerAngle = 40;	// 角度(危険)
	const uint16_t base = 160;			// x座標の中心
	const uint16_t lineBold = 5;		// インジケータの太さ
	auto indicatorColor = GREEN;		// インジケータの色
	float magnification = 0.375;		// 角度60をMAXとしたときの倍率

	// インジケータリセット
	if (tick % 10 == 0){
		for (uint8_t i = 0; i < lineBold; i++)
		{
			M5.Lcd.drawLine(0, 200+i, 320, 200+i,  BLACK);
		}
	}

	// インジケータの色を選定
	if (static_cast<uint16_t>(dangerAngle) < std::abs(kalAngleY))		// 40度を越えた場合は赤色で表示
	{
		indicatorColor = RED;
	} else if(static_cast<uint16_t>(warnAngle) < std::abs(kalAngleY)) 	// 30度を越えた場合は黄色で表示
	{
		indicatorColor = YELLOW;
	}

	// インジケータ表示 (画面サイズ: 240*360)
	uint16_t X = std::abs(kalAngleY) / magnification;
	if (0 < kalAngleY) // 右リーン
	{
		// 範囲 160 ~ 360
		for (uint8_t i = 0; i < lineBold; i++)
		{
			// 座標の始点から終点を線で結ぶ
			M5.Lcd.drawLine(base, 200+i, base+X, 200+i,  indicatorColor); // 引数: 始点x,y 終点x,y color
		}
		// 最大角度を更新(右)
		if (maxRightLean < std::abs(kalAngleY))
		{
			maxRightLean = std::abs(kalAngleY);
		}
	}
	else // 左リーン
	{
		// 範囲 0 ~ 160
		for (uint8_t i = 0; i < lineBold; i++)
		{
			// 座標の始点から終点を線で結ぶ
			M5.Lcd.drawLine(160-X, 200+i, base, 200+i, indicatorColor); // 引数: 始点x,y 終点x,y color
		}
		// 最大角度を更新(左)
		if (maxLeftLean < std::abs(kalAngleY))
		{
			maxLeftLean = std::abs(kalAngleY);
		}
	}

	M5.Lcd.setTextSize(4);
	M5.Lcd.setCursor(140, 100);
	M5.Lcd.fillRect(130, 90, 90, 40, BLACK);
	std::string strKalAngle = std::to_string(static_cast<int16_t>(std::abs(kalAngleY)));
	M5.Lcd.printf(strKalAngle.c_str());

	return true;
}

bool drawMaxLean()
{
	M5.Lcd.setTextSize(2);
	M5.Lcd.setCursor(65, 160);
	std::string displayMaxLean = "";
	displayMaxLean += "MaxL:";
	displayMaxLean += std::to_string(maxLeftLean);
	displayMaxLean += "  ";
	displayMaxLean += "MaxR:";
	displayMaxLean += std::to_string(maxRightLean);
	M5.Lcd.printf(displayMaxLean.c_str());

	return true;
}

/*
* 角度測定
*/
void readGyro(){
	M5.IMU.getGyroData(&gyro[0], &gyro[1], &gyro[2]);
	M5.IMU.getAccelData(&acc[0], &acc[1], &acc[2]);
}

void calibration(){
	float gyroSum[3];
	float accSum[3];
	for(int i = 0; i < 500; i++){
		readGyro();
		gyroSum[0] += gyro[0];
		gyroSum[1] += gyro[1];
		gyroSum[2] += gyro[2];
		accSum[0] += acc[0];
		accSum[1] += acc[1];
		accSum[2] += acc[2];
		delay(2);
	}
	gyroOffset[0] = gyroSum[0]/500;
	gyroOffset[1] = gyroSum[1]/500;
	gyroOffset[2] = gyroSum[2]/500;
	accOffset[0] = accSum[0]/500;
	accOffset[1] = accSum[1]/500;
	accOffset[2] = accSum[2]/500 - 1.0;
}

void applyCalibration(){
	gyro[0] -= gyroOffset[0];
	gyro[1] -= gyroOffset[1];
	gyro[2] -= gyroOffset[2];
	acc[0] -= accOffset[0];
	acc[1] -= accOffset[1];
	acc[2] -= accOffset[2];
}
float getRoll(){
  	return atan2(acc[1], acc[2]) * RAD_TO_DEG;
}
float getPitch(){
  	return atan(-acc[0] / sqrt(acc[1]*acc[1] + acc[2]*acc[2])) * RAD_TO_DEG;
}


// CANData
bool drawThrottleAngle()
{
	M5.Lcd.setTextSize(2);
	M5.Lcd.setCursor(20, 60);
	std::string throttle = "Throttle:";
	M5.Lcd.printf(throttle.c_str());

	M5.Lcd.setTextSize(3);
	M5.Lcd.setCursor(140, 55);
	std::string throttleAngle = "120";
	M5.Lcd.printf(throttleAngle.c_str());
	return true;
}