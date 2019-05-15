#include "nmea.h"
#include "gpio.h"
#include "glm.h"
#include "position.h"

#define NULL ((void *)0)

const double sm_a = 6378137.0;
const double sm_b = 6356752.314;
const double UTMScaleFactor = 0.9996;
char words[15][15];
int8_t a = 0;

NMEA pn;
extern position pos;
double xy[2] = {0.0, 0.0};


void createStartNMEA(void){
    pn.fix = (vec2){0, 0};
    pn.fixOffset = (vec2){0,0};
    pn.status = 'q';
    pn.hemisphere = 'N';



}
double ArcLengthOfMeridian(double phi){
    const double n = (sm_a - sm_b) / (sm_a + sm_b);
    double alpha = ((sm_a + sm_b) / 2.0) * (1.0 + (powf(n, 2.0) / 4.0)
                    + (pow(n, 4.0) / 64.0));
    double beta = (-3.0 * n / 2.0) + (9.0 * pow(n, 3.0) * 0.0625)
                    + (-3.0 * pow(n, 5.0) / 32.0);
    double gamma = (15.0 * pow(n, 2.0) * 0.0625) + (-15.0 * pow(n, 4.0) / 32.0);
    double delta = (-35.0 * pow(n, 3.0) / 48.0) + (105.0 * pow(n, 5.0) / 256.0);
    double epsilon = (315.0 * pow(n, 4.0) / 512.0);
    return alpha * (phi + (beta * sin(2.0 * phi))
            + (gamma * sin(4.0 * phi))
            + (delta * sin(6.0 * phi))
            + (epsilon * sin(8.0 * phi)));
}
void MapLatLonToXY(double phi, double lambda, double lambda0){
    double ep2 = (pow(sm_a, 2.0) - pow(sm_b, 2.0)) / pow(sm_b, 2.0);
    double nu2 = ep2 * pow(cos(phi), 2.0);
    double n = pow(sm_a, 2.0) / (sm_b * sqrt(1 + nu2));
    double t = tan(phi);
    double t2 = t * t;
    double l = lambda - lambda0;
    double l3Coef = 1.0 - t2 + nu2;
    double l4Coef = 5.0 - t2 + (9 * nu2) + (4.0 * (nu2 * nu2));
    double l5Coef = 5.0 - (18.0 * t2) + (t2 * t2) + (14.0 * nu2) - (58.0 * t2 * nu2);
    double l6Coef = 61.0 - (58.0 * t2) + (t2 * t2) + (270.0 * nu2) - (330.0 * t2 * nu2);
    double l7Coef = 61.0 - (479.0 * t2) + (179.0 * (t2 * t2)) - (t2 * t2 * t2);
    double l8Coef = 1385.0 - (3111.0 * t2) + (543.0 * (t2 * t2)) - (t2 * t2 * t2);
            // Calculate easting (x) 
            
    xy[0] = (n * cos(phi) * l)
        + (n / 6.0 * pow(cos(phi), 3.0) * l3Coef * pow(l, 3.0))
        + (n / 120.0 * pow(cos(phi), 5.0) * l5Coef * pow(l, 5.0))
        + (n / 5040.0 * pow(cos(phi), 7.0) * l7Coef * pow(l, 7.0));

        // Calculate northing (y) 
    xy[1] = ArcLengthOfMeridian(phi)
        + (t / 2.0 * n * pow(cos(phi), 2.0) * pow(l, 2.0))
        + (t / 24.0 * n * pow(cos(phi), 4.0) * l4Coef * pow(l, 4.0))
        + (t / 720.0 * n * pow(cos(phi), 6.0) * l6Coef * pow(l, 6.0))
        + (t / 40320.0 * n * pow(cos(phi), 8.0) * l8Coef * pow(l, 8.0));

}
void DecDeg2UTM(double latitude, double longitude){    //!!!!!!!!
    //only calculate the zone once!
    if (!pos.isFirstFixPositionSet){
        pn.zone = floor((longitude + 180.0) * 0.1666666666666) + 1;
        a = 1;
    }
    MapLatLonToXY(latitude * 0.01745329251994329576923690766743,
        longitude * 0.01745329251994329576923690766743,
        (-183.0 + (pn.zone * 6.0)) * 0.01745329251994329576923690766743);

    xy[0] = (xy[0] * UTMScaleFactor) + 500000.0;
    xy[1] *= UTMScaleFactor;
    if (xy[1] < 0.0)
        xy[1] += 10000000.0;

}
void UpdateNorthingEasting(void){
    DecDeg2UTM(pn.latitude, pn.longitude);

    //keep a copy of actual easting and northings
    pn.actualEasting = xy[0];
    pn.actualNorthing = xy[1];

    //if a field is open, the real one is subtracted from the integer
    pn.fix.easting = xy[0] - pn.utmEast + pn.fixOffset.easting;
    pn.fix.northing = xy[1] - pn.utmNorth + pn.fixOffset.northing;

    //compensate for the fact the zones lines are a grid and the world is round
    pn.fix.easting = (cos(-pn.convergenceAngle) * 
            pn.fix.easting) - (sin(-pn.convergenceAngle) * 
            pn.fix.northing);
    pn.fix.northing = (sin(-pn.convergenceAngle) * 
            pn.fix.easting) + (cos(-pn.convergenceAngle) * 
            pn.fix.northing);
}
void splitString(char *from){
	char *pch;
	pch = strtok(from, ",");
	for(int i = 0; pch != NULL && i<15; i++){
		sprintf(words[i], "%s", pch);
		for(int k = strlen(words[i]); k<15; k++)
			words[i][k] = '\0';
		pch = strtok(NULL, ",");
	}
}
void ParseNMEA(void *parameter){
    char *str;
    str = (char*) parameter;
    /* GPSPositionStatusBit
    bit0 - isGPSPositionInit
    bit1 - isFirstFixPositionSet
    */
    while (1)
    {   
        splitString(str);
        if (strstr(str, "$GPGGA") != NULL) ParseGGA(); 
        //if (strstr(str, "$GPVTG") != NULL) ParseVTG();
        //if (strstr(str, "$GPRMC") != NULL) ParseRMC();
        //if (strstr(str, "$GPGLL") != NULL) ParseGLL();
        if(a)
        	UpdateFixPosition();
        vTaskSuspend(NULL);         //При завершении обработки сообщения приостанавливаем задачу
    }
}
double NMEAtoDecimal(char *str){
    double wgs = 0.01666666666;
    double transform = atof(str);
    transform = (transform - (int)(transform-
        (int)transform%100)) * wgs + 
        (int)(transform-(int)transform%100)/100;
    return transform;
}
void ParseGGA(void){
    //$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M ,  ,*47
    //   0     1      2      3    4      5 6  7  8   9    10 11  12 13  14
    //         Time   Lat         Lon
    GPIOD->ODR ^= 0x4;
    pn.latitude = NMEAtoDecimal(words[2]);
    if (words[3] == "S"){
        pn.latitude *= -1;
        pn.hemisphere = 'S';
    }
    else pn.hemisphere = 'N';
    pn.longitude =NMEAtoDecimal(words[4]);
    if (words[5] == "W")
        pn.longitude *= -1;
    UpdateNorthingEasting();
    LCD_Send_String(0, "GGA");
    pn.fixQuality = atoi(words[6]);
    pn.satellitesTracked = atoi(words[7]);
    pn.hdop = atof(words[8]);
    pn.altitude = atof(words[9]);
    pn.ageDiff = atof(words[11]);
    strncpy(pn.time, words[1], 6);

    
}
void ParseGLL(void){
    //000GPGLL,5026.83816,N,03036.72223,E,092645.00,A,A*60
    //   0         1      2       3     4       5   6   7
    //            lat            lon
    GPIOD->ODR ^= 0x20;
    pn.latitude = NMEAtoDecimal(words[1]);
    if (words[2] == "S"){
        pn.latitude *= -1;
        pn.hemisphere = 'S';
    }
    else pn.hemisphere = 'N';
    pn.longitude =NMEAtoDecimal(words[3]);
    if (words[4] == "W")
        pn.longitude *= -1;
    UpdateNorthingEasting();
    LCD_Send_String(0, "GLL");
    strncpy(pn.time, words[1], 5);

}
void ParseRMC(void){
    //$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
    //   0      1   2     3    4      5    6    7     8     9      10
    //        time      lat         lon    spidInKnots
    GPIOD->ODR ^= 0x10;
    pn.latitude = NMEAtoDecimal(words[3]);
    if (words[4] == "S"){
        pn.latitude *= -1;
        pn.hemisphere = 'S';
    }
    else pn.hemisphere = 'N';
    pn.longitude =NMEAtoDecimal(words[5]);
    if (words[6] == "W")
        pn.longitude *= -1;
    UpdateNorthingEasting();
    LCD_Send_String(0, "RMC");
    pn.speed = atof(words[7]);
    pn.speed = round(pn.speed * 1.852);
    pn.headingTrue = atof(words[8]);

}
void ParseVTG(void){
    GPIOD->ODR ^= 0x8;
    LCD_Send_String(0, "VTG");
    pn.headingTrue = atof(words[1]);
    pn.speed = atof(words[5]);
    pn.speed = round(pn.speed * 1.852);
}
