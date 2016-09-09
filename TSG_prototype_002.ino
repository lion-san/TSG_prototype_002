//------------------------------------------------------------
//    姿勢制御フィルタリングプログラム
//                Arduino　IDE　1.6.11
//
//　　　Arduino　　　　　　　　LSM9DS1基板　
//　　　　3.3V　　　------　　　　3.3V
//　　　　GND       ------   　　 GND
//　　　　SCL       ------        SCL
//　　　　SDA       ------        SDA
//
//　センサーで取得した値をシリアルモニターに表示する
//
//　　　　
//----------------------------------------------------------//


#include <SPI.h>                                //SPIライブラリ
#include <Wire.h>                               //I2Cライブラリ
#include <SparkFunLSM9DS1.h>                  //LSM9DS1ライブラリ：https://github.com/sparkfun/LSM9DS1_Breakout
#include <SD.h>
#include <LSM9DS1_Registers.h>
#include <LSM9DS1_Types.h>
#include <SoftwareSerial.h>


//#define ADAddr 0x48//

#define LSM9DS1_M  0x1E                 // SPIアドレス設定 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B                // SPIアドレス設定 if SDO_AG is LOW

//#define PRINT_CALCULATED              //表示用の定義
//#define DEBUG_GYRO                    //ジャイロスコープの表示

#define DECLINATION -8.58               // Declination (degrees) in Boulder, CO.


#define RX 8                            //GPS用のソフトウェアシリアル
#define TX 9                            //GPS用のソフトウェアシリアル
#define SENTENCES_BUFLEN      82        // GPSのメッセージデータバッファの個数

//-------------------------------------------------------------------------
//[Global valiables]

LSM9DS1 imu;
int SAMPLETIME = 10;
int RECORD_INTERVAL = 100;
int WRITE_INTERVAL = 1000;

//###############################################
//MicroSD 
//const int chipSelect = 4;//Arduino UNO
const int chipSelect = 10;//Arduino Micro
//###############################################

const int tact_switch = 7;//タクトスイッチ
boolean switchIs = false;
boolean swithchOn;
boolean switchRelease;

//ジャイロセンサーの積分値
//float pitch_g = 0.0;
//float roll_g = 0.0;

//相補フィルタの保持値
float prev_pitch = 0.0;
float prev_roll = 0.0;

String motionData;

//----------------------------------------------------------------------
//=== Global for GPS ===========================================
SoftwareSerial  g_gps( RX, TX );
char head[] = "$GPRMC";
char info[] = "$GPGGA";
char buf[10];
int SentencesNum = 0;                   // GPSのセンテンス文字列個数
byte SentencesData[SENTENCES_BUFLEN] ;  // GPSのセンテンスデータバッファ
boolean isReaded;                       //GPSの読み込みが完了したかどうか
String gpsData;                         //GPSの読み込みが完了データ
//======================================================

void setup(void) {

  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  //=== SD Card Initialize ====================================
  Serial.print(F("Initializing SD card..."));

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println(F("Card failed, or not present"));
    // don't do anything more:
    return;
  }
  Serial.println(F("card initialized."));

  //=======================================================

  //タクトスイッチ
  pinMode(tact_switch, INPUT);

  //=== LSM9DS1 Initialize =====================================
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress  = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;

  if (!imu.begin())              //センサ接続エラー時の表示
  {
    Serial.println(F("Failed to communicate with LSM9DS1."));
    while (1)
      ;
  }
  //=======================================================


  //=== GPS用のソフトウェアシリアル有効化 =================
  setupSoftwareSerial();
  //=======================================================
  
}

/**
 * loop
 * ずっと繰り返される関数（何秒周期？）
 * 【概要】
 * 　10msでセンサーデータをサンプリング。
 * 　記録用に、100ms単位でデータ化。
 * 　蓄積したデータをまとめて、1000ms単位でSDカードにデータを出力する。
 * 　
 */
void loop(void) {

  switch(digitalRead(tact_switch)){

   case 0://ボタンを押した

          //離すまでは実行しない
          if(switchOn)
              return;

         switchOn = true;


          
          break;

    case 1://ボタン押していない

           if(switchOn){

            
           }



            if(!switchRelease){
               if(switchIs)//既にOnなら
                 switchIs = false;

               switchRelease = false;
            }

            switchRelease = true;
              

            break;

    default:
            break;
  }

  //スイッチの判定
  if(!switchIs) //falseなら、ループする
    return;
  

  //GPS
  //▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼
  char dt = 0 ;

  motionData = "";

    // センテンスデータが有るなら処理を行う
    if (g_gps.available()) {

        // 1バイト読み出す
        dt = g_gps.read() ;
        //Serial.write(dt);

        //Serial.write(dt);//Debug ALL
        // センテンスの開始
        if (dt == '$') SentencesNum = 0 ;
        
        if (SentencesNum >= 0) {
          
          // センテンスをバッファに溜める
          SentencesData[SentencesNum] = dt ;
          SentencesNum++ ;
             
          // センテンスの最後(LF=0x0Aで判断)
          if (dt == 0x0a || SentencesNum >= SENTENCES_BUFLEN) {
    
            SentencesData[SentencesNum] = '\0' ;
    
            //GPS情報の取得
            getGpsInfo();

            //MotionSensorの値更新
            updateMotionSensors(false);
            
            // センテンスのステータスが"有効"になるまで待つ
            if ( gpsIsReady() )
            {
               // 有効になったら書込み開始
               //Serial.print("O:");
               //Serial.print( (char *)SentencesData );

               gpsData = String((char *)SentencesData );
               
               // read three sensors and append to the string:
               //記録用のセンサー値を取得
               motionData = updateMotionSensors(true);

               //SDカードへの出力
               writeDataToSdcard();


               return;
            }
          }
        }
      }

}


/**
 * writeDataToSdcard
 */
void writeDataToSdcard()
{

  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    
    dataFile.print(gpsData);
    dataFile.print(motionData);
    
    dataFile.close();
    
    // print to the serial port too:
    Serial.println(gpsData);
    Serial.println(motionData);
    Serial.println(F("================================"));
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println(F("error opening datalog.txt"));
  }

}




/**
 * updateMotionSensors
 */
unsigned long time = millis();
String updateMotionSensors(boolean print)
{

  Serial.println( millis() - time);
  time = millis();

  
  //Read three sensors data on the memory
  readGyro();
  readAccel();
  readMag();
  
  //メモリ上の角度データの更新（前回値と今回値が考慮される）  
  return printAttitude (imu.calcGyro(imu.gx), imu.calcGyro(imu.gy), imu.calcGyro(imu.gz), imu.ax, imu.ay, imu.az, -imu.my, -imu.mx, imu.mz, print) + "\n";

}




//--------------------　Gyro DATA ------------------------------------
void readGyro()
{

  imu.readGyro();


}
//-------------------　Accel DATA ----------------------
void readAccel()
{
  // To read from the accelerometer, you must first call the
  // readAccel() function. When this exits, it'll update the
  // ax, ay, and az variables with the most current data.
  imu.readAccel();


}
//--------------　Mag DATA ------------------
void readMag()
{

  imu.readMag();



}
//---------------------------------------------------------
/**
 * printAttitude
 * 取得したデータをシリアル出力する関数
 * gx : ジャイロスコープ X値
 * gy : ジャイロスコープ Y値
 * gz : ジャイロスコープ Z値
 * ax : 加速度センサー X値
 * ay : 加速度センサー Y値
 * az : 加速度センサー Z値
 * mx : 地磁気センサー X値
 * my : 地磁気センサー Y値
 * mz : 地磁気センサー Z値
 * print : 値を返すかどうか
 */

String printAttitude(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, boolean print)
{

  String output = "";

  //重力加速度から求めた角度ををカルマンフィルタの初期値とする
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));

  float heading;
  if (my == 0)
    heading = (mx < 0) ? 180.0 : 0;
  else
    heading = atan2(mx, my);

  heading -= DECLINATION * PI / 180;

  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;

  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;


  //*** Gyro ***
  float gyro_x =  gx * SAMPLETIME / 1000;
  float gyro_y = gy * SAMPLETIME / 1000;



  //相補フィルタの出力
  prev_pitch = complementFilter( prev_pitch, gyro_x, pitch );
  prev_roll = complementFilter( prev_roll, gyro_y, roll );


  //出力が求められる場合のみ、文字列計算
  if(print){
    output = prev_pitch;
    output += ",";
    output += prev_roll;
  }



  return output;
}


/**
 * 相補フィルタ
 * prev_val : 前回のOutput
 * deg_g : ジャイロセンサで得た角度
 * deg_a : 加速度センサーで得た角度
 */
 float complementFilter(float prev_val, float deg_g, float deg_a){
    return 0.95 * (prev_val + deg_g) + 0.05 * deg_a;
 }


//===============================================
//
//      GPS系の処理
//
//===============================================

/**
 * setupSoftwareSerial
 * GPS用のソフトウェアシリアルの有効化
 */
void setupSoftwareSerial(){
  g_gps.begin(9600);
}


/**
 * getGpsInfo
 * $GPGGA　ヘッダから、衛星受信数や時刻情報を取得
 */
void getGpsInfo()
{
    int i, c;
    
    //$1ヘッダが一致
    if( strncmp((char *)SentencesData, info, 6) == 0 )
    {

      //コンマカウント初期化
      c = 1; 

      // センテンスの長さだけ繰り返す
      for (i=0 ; i<SentencesNum; i++) {
        if (SentencesData[i] == ','){
          
            c++ ; // 区切り文字を数える
    
            if ( c == 2 ) {
                 //Serial.println(F("----------------------------"));
                // Serial.println((char *)SentencesData);
                 Serial.print(F("Time:"));
                 Serial.println(readDataUntilComma(i+1));
                 continue;
            }
            else if ( c == 8 ) {
                // Serial.println((char *)SentencesData);
                 Serial.print(F("Number of Satelites:"));
                 Serial.println(readDataUntilComma(i+1));
                 continue;
            }
        }
      }
      
    }
}


/**
 * gpsIsReady
 * GPS情報が有効かどうかを判断
 * 項目3が"A"かどうかで判断
 */
boolean gpsIsReady()
{
    int i, c;
    
    //$1ヘッダが一致かつ,$3ステータスが有効＝A
    if( strncmp((char *)SentencesData, head, 6) == 0 )
    {

      //コンマカウント初期化
      c = 1; 

      // センテンスの長さだけ繰り返す
      for (i=0 ; i<SentencesNum; i++) {
        if (SentencesData[i] == ','){
              
              c++ ; // 区切り文字を数える
    
            if ( c == 3 ) {
                 //次のコンマまでのデータを呼び出し
                 if( strncmp("A", readDataUntilComma(i+1), 1) == 0 ){
                   return true;
                 }
                 else{
                   //Serial.print("X:");
                   //Serial.print( (char *)SentencesData );
                   return false;
                 }
            }
        }
      }
      
    }

    return false;
}

/**
  * readDataUntilComma
  */
char* readDataUntilComma(int s)
{
  int i, j;

  j = 0;
  //初期化
  memset(buf,0x00,sizeof(buf)) ;

  //終了条件
  //次のコンマが出現or特定文字*（チェックサム)が出現
  for (i = s; i < SentencesNum; i++)
  {
    if(( SentencesData[i] == ',') || (SentencesData[i] == '*')){
      buf[j] = '\0';
      return buf;
    }
    else{
      //バッファーのオーバフローをチェック
      if( j < 10 ) {
        buf[j] = SentencesData[i];
        j++;
      }
      else{//エラー処理
        int x;
        for(x = 0; x < sizeof(buf); x++)
          buf[x] = 'X';
          return buf;
      }
      
    }
  }
  
}

