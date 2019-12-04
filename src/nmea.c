#include "nmea.h"
#include "gpio.h"
#include "glm.h"
#include "position.h"

#define NULL    ( (void *) 0)
#define nullptr ( (void *) 0)

const   double    sm_a = 6378137.0;
const   double    sm_b = 6356752.314;
        char      words[15][15];             // Двумерный массив для парсинга сообщений
        NMEA      pn;                        // Структура, где хранятся пременные, расчитываемые из NMEA
extern  position  pos;
        double    xy[2] = {0.0, 0.0};        // Для расчета UTM-coord


/*! 
  Задаем начальные значения при создании структуры
 */
void 
createStartNMEA(void){
  pn.fix = (vec2){0, 0};
  pn.fixOffset = (vec2){0,0};
  pn.status = 'q';
  pn.hemisphere = 'N';
  pn.zone = 0;
  pn.coordCorrect = 0;
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
  return alpha    * (phi + (beta * sin(2.0 * phi))
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
  const double UTMScaleFactor = 0.9996;
  //only calculate the zone once!
  if (!pos.isFirstFixPositionSet){
      pn.zone = floor((longitude + 180.0) * 0.1666666666666) + 1;
  }

  MapLatLonToXY(latitude  * 0.01745329251994329576923690766743,
                longitude * 0.01745329251994329576923690766743,
                (-183.0 + (pn.zone * 6.0)) 
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
  DecDeg2UTM(pn.latitude, pn.longitude);
  //keep a copy of actual easting and northings
  pn.actualEasting  = xy[0];
  pn.actualNorthing = xy[1];

  //if a field is open, the real one is subtracted from the integer
  pn.fix.easting    = xy[0] - pn.utmEast  + pn.fixOffset.easting;
  pn.fix.northing   = xy[1] - pn.utmNorth + pn.fixOffset.northing;

  //compensate for the fact the zones lines are a grid and the world is round
  pn.fix.easting  = ( cos (-pn.convergenceAngle) * pn.fix.easting) 
                  - ( sin (-pn.convergenceAngle) * pn.fix.northing);
  pn.fix.northing = ( sin (-pn.convergenceAngle) * pn.fix.easting) 
                  + ( cos (-pn.convergenceAngle) * pn.fix.northing);
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
	char *pch;
	pch = strtok(from, ",");
	for(int i = 0; pch != NULL && i<15; i++){
		sprintf(words[i], "%s", pch);
		for(int k = strlen(words[i]); k<15; k++)
			words[i][k] = '\0';
		pch = strtok(NULL, ",");
	}

/*
	char *pch;                                  // Указатель
	pch = strtok(from, ",");                    // Разделителем выступает запятая

	for(int i = 0; pch != NULL && i<15; i++) {  // Разбрасываем по отдельным массивам
		sprintf(words[i], "%s", pch);             // Копируем данные
    int k = strlen(words[i]);                 // Остаток... 
    memset(words[i] + k, '\0', 15-k);         // ...заполняем "нулями"
		pch = strtok(NULL, ",");
	}
	*/
	


}

void 
ParseNMEA(void *parameter){
  /* GPSPositionStatusBit
  bit0 - isGPSPositionInit
  bit1 - isFirstFixPositionSet
  */
  while (1) {
    splitString( (char*) parameter);          // Разбиваем сообщение по массивам

    if (strstr( (char*) parameter, "$GPGGA") != NULL) ParseGGA(); 
    //if (strstr( (char*) parameter, "$GPVTG") != NULL) ParseVTG();
    //if (strstr( (char*) parameter, "$GPRMC") != NULL) ParseRMC();
    //if (strstr( (char*) parameter, "$GPGLL") != NULL) ParseGLL();

    // Если координаты были обновлены, то работаем дальше
    if( (pn.coordCorrect & 0b11) == 0b11) 
      UpdateFixPosition();

    doubleToDisplay(pn.zone, 1);
    doubleToDisplay(pn.latitude, 2);
    doubleToDisplay(pn.longitude, 3);

    vTaskSuspend(NULL);         // При завершении обработки сообщения приостанавливаем задачу
                                // Для ожидания нового сообщения для обработки
  }
}

/*! 
 * Переводим координаты, полученные в сообщении, из минут в 
 * десятичную систему
 */
double 
NMEAtoDecimal(char *str){
  double koef = 0.01666666666;          ///< Коефициент перевода
  double var = atof(str);               ///< Переменная из сообщения
  var = (var - (int)(var - (int)var % 100)) 
      * koef + (int)(var - (int)var % 100) / 100;
  return var ;
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

  pn.latitude = NMEAtoDecimal(words[2]);  // Получаем десятичное значение широты
  if( strchr(words[2], '.'))              // Выставляем бит коректности данных
    pn.coordCorrect |= 0b1;
  else
    pn.coordCorrect &= ~0b1;   

  pn.longitude = NMEAtoDecimal(words[4]);  // Получаем десятичное значение долготы
  if( strchr(words[4], '.'))              // Выставляем бит коректности данных
    pn.coordCorrect |= 0b10;
  else
    pn.coordCorrect &= ~0b10;   


  if( (pn.coordCorrect & 0b11) == 0b11)             // Если обе координаты коректны, то обрабатываем полученные данные
    UpdateNorthingEasting();
  else                                    // Если нет, то не мусорим и выходим
    return;

  // Положение по полушариям
  if (words[3] == "S"){
    pn.latitude  *= -1;
    pn.hemisphere = 'S';
  }
  else
    pn.hemisphere = 'N';

  if (words[5] == "W")
    pn.longitude *= -1;

  // Остальная информация
  pn.fixQuality        = atoi(words[6]);
  pn.satellitesTracked = atoi(words[7]);
  pn.hdop              = atof(words[8]);
  pn.altitude          = atof(words[9]);
  pn.ageDiff           = atof(words[11]);

  strncpy(pn.time, words[1], 6);
  LCD_Send_String(0, "GGA");
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

  pn.latitude = NMEAtoDecimal(words[1]);  // Получаем десятичное значение широты
  if( strchr(words[1], '.'))              // Выставляем бит коректности данных
    pn.coordCorrect |= 0b1;
  else
    pn.coordCorrect &= ~0b1;   

  pn.longitude =NMEAtoDecimal(words[3]);  // Получаем десятичное значение долготы
  if( strchr(words[3], '.'))              // Выставляем бит коректности данных
    pn.coordCorrect |= 0b10;
  else
    pn.coordCorrect &= ~0b10;   

  if( (pn.coordCorrect & 0b11) == 0b11)   // Если обе координаты коректны, то обрабатываем полученные данные
    UpdateNorthingEasting();
  else                                    // Если нет, то не мусорим и выходим
    return;

  if (words[2] == "S"){
    pn.latitude  *= -1;
    pn.hemisphere = 'S';
  }
  else pn.hemisphere = 'N';
  if (words[4] == "W")
    pn.longitude *= -1;

  LCD_Send_String(0, "GLL");
  strncpy(pn.time, words[5], 6);
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

  pn.latitude = NMEAtoDecimal(words[3]);  // Получаем десятичное значение широты
  if( strchr(words[3], '.'))              // Выставляем бит коректности данных
    pn.coordCorrect |= 0b1;
  else
    pn.coordCorrect &= ~0b1;   

  pn.longitude = NMEAtoDecimal(words[5]);  // Получаем десятичное значение долготы
  if( strchr(words[5], '.'))              // Выставляем бит коректности данных
    pn.coordCorrect |= 0b10;
  else
    pn.coordCorrect &= ~0b10;   

  if( (pn.coordCorrect & 0b11) == 0b11)             // Если обе координаты коректны, то обрабатываем полученные данные
    UpdateNorthingEasting();
  else                                    // Если нет, то не мусорим и выходим
    return;

  // Положение по полушариям
  if (words[4] == "S"){
    pn.latitude *= -1;
    pn.hemisphere = 'S';
  }
  if (words[6] == "W")
    pn.longitude *= -1;
  else pn.hemisphere = 'N';

  pn.speed = atof(words[7]);
  pn.speed = round(pn.speed * 1.852);
  pn.headingTrue = atof(words[8]);

  LCD_Send_String(0, "RMC");
  strncpy(pn.time, words[1], 6);
  strncpy(pn.date, words[9], 6);

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
  LCD_Send_String(0, "VTG");
  pn.headingTrue = atof(words[1]);
  pn.speed = atof(words[5]);
  pn.speed = round(pn.speed * 1.852);
}
