#include "nmea.h"
#include "gpio.h"
#include "glm.h"
#include "position.h"
#include "ABLine.h"

#define NULL    ( (void *) 0)
#define nullptr ( (void *) 0)

const   double    sm_a = 6378137.0;
const   double    sm_b = 6356752.314;
        char      words[15][15];             // Двумерный массив для парсинга сообщений

        NMEA      nmea;                        // Структура, где хранятся пременные, расчитываемые из NMEA
extern  Position  position;
extern  ABline 		abline;
extern  Vehicle   vehicle;  

        double    xy[2] = {0.0, 0.0};        // Для расчета UTM-coord
const   double    UTMScaleFactor = 0.9996;
const   double    koef = 0.01666666666;          ///< Коефициент перевода

/*!
 * Проверяем наличие значений долготы и широты
*/
void  checkLatLon(char *lat, char *lon){
  nmea.latitude = NMEAtoDecimal(lat);  // Получаем десятичное значение широты
  if( strchr(lat, '.'))              // Выставляем бит коректности данных
    nmea.flags |= 0x01 << latitudeOk;
  else
    nmea.flags &= ~(0x01 << latitudeOk);

  nmea.longitude = NMEAtoDecimal(lon);  // Получаем десятичное значение долготы
  if( strchr(lon, '.'))              // Выставляем бит коректности данных
    nmea.flags |= 0x01 << longtitudeOk;
  else
    nmea.flags &= ~(0x01 << longtitudeOk);
}

/*! 
  Задаем начальные значения при создании структуры
 */
void 
createStartNMEA(void){
  nmea.fix        = (vec2){0, 0};
  nmea.fixOffset  = (vec2){0,0};
  nmea.status     = 'q';
  nmea.hemisphere = 'N';
  nmea.zone       = 0.0;
  nmea.flags      = 0x00;      
}

/*!
  Расчет длины дуги меридиана
 */
double 
ArcLengthOfMeridian(double phi){
  const double n  = (sm_a - sm_b) / (sm_a + sm_b);
  double alpha    = ((sm_a + sm_b) / 2.0) * (1.0 + (powf(n, 2.0) / 4.0)
                  + (pow(n, 4.0) / 64.0));
  double beta     = (-3.0 * n / 2.0) + (9.0 * pow(n, 3.0) * 0.0625)
                  + (-3.0 * pow(n, 5.0) / 32.0);
  double gamma    = (15.0 * pow(n, 2.0) * 0.0625) + (-15.0 * pow(n, 4.0) / 32.0);
  double delta    = (-35.0 * pow(n, 3.0) / 48.0) + (105.0 * pow(n, 5.0) / 256.0);
  double epsilon  = (315.0 * pow(n, 4.0) / 512.0);
  return    alpha * (phi + (beta * sin(2.0 * phi))
                  + (gamma   * sin(4.0 * phi))
                  + (delta   * sin(6.0 * phi))
                  + (epsilon * sin(8.0 * phi)));
}

void 
MapLatLonToXY(double phi, double lambda, double lambda0){
  double ep2    = (pow(sm_a, 2.0) - pow(sm_b, 2.0)) / pow(sm_b, 2.0);
  double nu2    = ep2 * pow(cos(phi), 2.0);
  double n      = pow(sm_a, 2.0) / (sm_b * sqrt(1 + nu2));
  double t      = tan(phi);
  double t2     = t * t;
  double l      = lambda - lambda0;
  double l3Coef = 1.0 - t2 + nu2;
  double l4Coef = 5.0 - t2 + (9 * nu2)    + (4.0 * (nu2 * nu2));
  double l5Coef = 5.0     - (18.0 * t2)   + (t2 * t2) + (14.0 * nu2)  - (58.0 * t2 * nu2);
  double l6Coef = 61.0    - (58.0 * t2)   + (t2 * t2) + (270.0 * nu2) - (330.0 * t2 * nu2);
  double l7Coef = 61.0    - (479.0 * t2)  + (179.0 * (t2 * t2))       - (t2 * t2 * t2);
  double l8Coef = 1385.0  - (3111.0 * t2) + (543.0 * (t2 * t2))       - (t2 * t2 * t2);
  
  // Calculate easting (x) 
  xy[0] = (n * cos(phi) * l)
        + (n / 6.0    * pow(cos(phi), 3.0) * l3Coef * pow(l, 3.0))
        + (n / 120.0  * pow(cos(phi), 5.0) * l5Coef * pow(l, 5.0))
        + (n / 5040.0 * pow(cos(phi), 7.0) * l7Coef * pow(l, 7.0));

  // Calculate northing (y) 
  xy[1] = ArcLengthOfMeridian(phi)
        + (t / 2.0      * n * pow(cos(phi), 2.0) * pow(l, 2.0))
        + (t / 24.0     * n * pow(cos(phi), 4.0) * l4Coef * pow(l, 4.0))
        + (t / 720.0    * n * pow(cos(phi), 6.0) * l6Coef * pow(l, 6.0))
        + (t / 40320.0  * n * pow(cos(phi), 8.0) * l8Coef * pow(l, 8.0));
}

/*!
  Переводим координаты в формат UTM
 */
void 
DecDeg2UTM(double latitude, double longitude){    //!!!!!!!!
  //only calculate the zone once!
  if ( !(nmea.flags >>  zone & 0x01)){
      nmea.zone = floor((longitude + 180.0) * 0.1666666666666) + 1;
      doubleToDisplay(nmea.zone, 0);
      nmea.flags |= 0x01 << zone;       ///Выставляем флаг получения зоны
  }

  MapLatLonToXY(latitude  * 0.01745329251994329576923690766743,
                longitude * 0.01745329251994329576923690766743,
                (-183.0 + (nmea.zone * 6.0)) 
                * 0.01745329251994329576923690766743);

  xy[0]  = (xy[0] * UTMScaleFactor) + 500000.0;
  xy[1] *= UTMScaleFactor;

  if (xy[1] < 0.0)
      xy[1] += 10000000.0;
}

/*!
  Обновляем расчитаные координаты в структуре
 */
void 
UpdateNorthingEasting(void){
  DecDeg2UTM(nmea.latitude, nmea.longitude);
  //keep a copy of actual easting and northings
  nmea.actualEasting  = xy[0];
  nmea.actualNorthing = xy[1];

  //if a field is open, the real one is subtracted from the integer
  nmea.fix.easting    = xy[0] - nmea.utmEast  + nmea.fixOffset.easting;
  nmea.fix.northing   = xy[1] - nmea.utmNorth + nmea.fixOffset.northing;

  //compensate for the fact the zones lines are a grid and the world is round
  nmea.fix.easting  = ( cos (-nmea.convergenceAngle) * nmea.fix.easting) 
                    - ( sin (-nmea.convergenceAngle) * nmea.fix.northing);
  nmea.fix.northing = ( sin (-nmea.convergenceAngle) * nmea.fix.easting) 
                    + ( cos (-nmea.convergenceAngle) * nmea.fix.northing);
}

//--------------------------------------------------------------------------------//
//----------------PARSING NMEA MESSAGES-------------------------------------------//
//--------------------------------------------------------------------------------//
/*!
  Разделяем строку по символу ","
  strtok - split string into tokens
 */
void 
splitString(char *from){
	char *pch;                                  // Указатель
	pch = strtok(from, ",");                    // Разделителем выступает запятая
	for(int i = 0; pch != NULL && i < 15; i++) {  // Разбрасываем по отдельным массивам
    strcpy(words[i], pch);
		for(int k = strlen(words[i]); k < 15; k++){
			words[i][k] = '\0';
    }
		pch = strtok(NULL, ",");
	}
}

void 
ParseNMEA(void *parameter){
  vec3 pivotAxlePos;   //position for AB_Calculations


  while (1) {
    splitString( (char*) parameter);          // Разбиваем сообщение по массивам

    if (strstr( (char*) parameter, "$GPGGA") != NULL) ParseGGA(); 
    if (strstr( (char*) parameter, "$GPVTG") != NULL) ParseVTG();
    if (strstr( (char*) parameter, "$GPRMC") != NULL) ParseRMC();
    if (strstr( (char*) parameter, "$GPGLL") != NULL) ParseGLL();

    // Ошибка в координатах, товыходим
    if( (nmea.flags >> latitudeOk   & 0x01)
    &&  (nmea.flags >> longtitudeOk & 0x01) ) {

      UpdateFixPosition();

      doubleToDisplay(nmea.latitude,  1);
      doubleToDisplay(nmea.longitude, 2);

      if (abline.flags >> ABLineSet & 0x01) {
        pivotAxlePos.easting  = nmea.fix.easting - (sin(position.pivotAxlePos.heading) 
                              * vehicle.antennaPivot);
        pivotAxlePos.northing = nmea.fix.easting - (cos(position.pivotAxlePos.heading) 
                              * vehicle.antennaPivot);
        pivotAxlePos.heading  = position.fixHeading;

        GetCurrentABLine(pivotAxlePos);
      }
    }

    // Очищаем флаги для предотвращения обработки повторных данных
    nmea.flags &= ~(0x01 << latitudeOk);
    nmea.flags &= ~(0x01 << longtitudeOk);
    vTaskSuspend(NULL);         // При завершении обработки сообщения приостанавливаем задачу
                                // Для ожидания нового сообщения для обработки
  }
}

/*!
*  Расчет полушарий
*/
void 
calcSphere(char* w1, char* w2){
  if (w1 == "S"){
    nmea.latitude  *= -1;
    nmea.hemisphere = 'S';
  }
  else{
    nmea.hemisphere = 'N';
  }
  if (w2 == "W"){
    nmea.longitude *= -1;
  }
}

/*! 
 * Переводим координаты, полученные в сообщении, из минут в 
 * десятичную систему
 */
double 
NMEAtoDecimal(char *str){
  double var = atof(str);               ///< Переменная из сообщения
  var = (var - (int)(var - (int)var % 100)) 
      * koef + (int)(var - (int)var % 100) / 100;
  return var;
}

/*!
 * Парсим GGA-сообщение
 *GGA Global Positioning System Fix Data. Time, Position and fix related data for a GPS receiver
 * $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M ,  ,*47
 *    0     1      2      3    4      5 6  7  8   9    10 11  12 13  14
 *          Time   Lat         Lon
 *         1         2    3    4     5 6  7  8   9  10 11 12 13   14 15
 *         |         |    |    |     | |  |  |   |  |  |  |  |    |  |
 *$--GGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
 *1) Time (UTC)
 *2) Latitude
 *3) N or S (North or South)
 *4) Longitude
 *5) E or W (East or West)
 *6) GPS Quality Indicator,
 *    0 - fix not available,
 *    1 - GPS fix,
 *    2 - Differential GPS fix
 *7) Number of satellites in view, 00 - 12
 *8) Horizontal Dilution of precision
 *9) Antenna Altitude above/below mean-sea-level (geoid)
 *10) Units of antenna altitude, meters
 *11) Geoidal separation, the difference between the WGS-84 earth
 *    ellipsoid and mean-sea-level (geoid), "-" means mean-sea-level below ellipsoid
 *12) Units of geoidal separation, meters
 *13) Age of differential GPS data, time in seconds since last SC104
 *    type 1 or 9 update, null field when DGPS is not used
 *14) Differential reference station ID, 0000-1023
 *15) Checksum
 */ 
void 
ParseGGA(void){
  // Мигаем светлодиодом, для индикации
  GPIOD->ODR ^= 0x4;

  checkLatLon(words[2], words[4]);

  if( (nmea.flags >> latitudeOk & 0x01)
  &&  (nmea.flags >> longtitudeOk & 0x01)) { // Если обе координаты коректны, то обрабатываем полученные данные
    UpdateNorthingEasting();
  }
  else                                    // Если нет, то не мусорим и выходим
    return;

  // Положение по полушариям
  calcSphere(words[3], words[5]);

  // Остальная информация
  nmea.fixQuality        = atoi(words[6]);
  nmea.satellitesTracked = atoi(words[7]);
  nmea.hdop              = atof(words[8]);
  nmea.altitude          = atof(words[9]);
  nmea.ageDiff           = atof(words[11]);

  strncpy(nmea.time, words[1], 6);
}


/* 
* GLL Geographic Position – Latitude/Longitude
* 000GPGLL,5026.83816,N,03036.72223,E,092645.00,A,A*60
*    0         1      2       3     4       5   6   7
*             lat            lon
*         1      2     3    4    5      6 7
*         |      |     |    |    |      | |
* $--GLL,llll.ll,a,yyyyy.yy,a,hhmmss.ss,A*hh
* 1) Latitude
* 2) N or S (North or South)
* 3) Longitude
* 4) E or W (East or West)
* 5) Time (UTC)
* 6) Status A - Data Valid, V - Data Invalid
* 7) Checksum
*/
void 
ParseGLL(void){
  GPIOD->ODR ^= 0x20;

  checkLatLon(words[1], words[3]);

  if( (nmea.flags >> latitudeOk & 0x01)
  &&  (nmea.flags >> longtitudeOk & 0x01)) { // Если обе координаты коректны, то обрабатываем полученные данные
    UpdateNorthingEasting();
  }
  else{                                    // Если нет, то не мусорим и выходим
    return;
  }

  // Положение по полушариям
  calcSphere(words[2], words[4]);
  strncpy(nmea.time, words[5], 6);
}



/*
* RMC Recommended Minimum Navigation Information
* $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
*    0      1   2     3    4      5    6    7     8     9      10
*         time      lat         lon    spidInKnots
*           1      2    3    4     5    6  7   8    9  10 11  12
*           |      |    |    |     |    |  |   |    |   |  |  | 
* $--RMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,xxxx,x.x,a*hh
* 1) Time (UTC)
* 2) Status, V = Navigation receiver warning
* 3) Latitude
* 4) N or S
* 5) Longitude
* 6) E or W
* 7) Speed over ground, knots
* 8) Track made good, degrees true
* 9) Date, ddmmyy
* 10) Magnetic Variation, degrees
* 11) E or W
* 12) Checksum
*/
void 
ParseRMC(void){
  GPIOD->ODR ^= 0x10;

  checkLatLon(words[3], words[5]);

  if( (nmea.flags >> latitudeOk & 0x01)
  &&  (nmea.flags >> longtitudeOk & 0x01)) { // Если обе координаты коректны, то обрабатываем полученные данные
    UpdateNorthingEasting();
  }
  else{                                    // Если нет, то не мусорим и выходим
    return;
  }

  // Положение по полушариям
  calcSphere(words[4], words[6]);

  nmea.speed = atof(words[7]);
  nmea.speed = round(nmea.speed * 1.852);
  nmea.headingTrue = atof(words[8]);

  strncpy(nmea.time, words[1], 6);
  strncpy(nmea.date, words[9], 6);
}

/*
* Парсим VTG-сообщение
* VTG                    Track Made Good and Ground Speed
*         1   2 3   4 5   6 7   8 9
*         |   | |   | |   | |   | |
*  $--VTG,x.x,T,x.x,M,x.x,N,x.x,K*hh
*  1) Track Degrees
*  2) T = True
*  3) Track Degrees
*  4) M = Magnetic
*  5) Speed Knots
*  6) N = Knots
*  7) Speed Kilometers Per Hour
*  8) K = Kilometres Per Hour
*  9) Checksum
*/ 
void 
ParseVTG(void){
  GPIOD->ODR ^= 0x8;
  nmea.headingTrue  = atof(words[1]);
  nmea.speed        = atof(words[5]);
  nmea.speed        = round(nmea.speed * 1.852);
}
