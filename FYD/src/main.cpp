#include <Arduino.h>
#include "driver/uart.h"

#define KNOT_TO_KMH 1.852

// What we wnat from gps---------------------------------------------------
double Time = -1 ; //hhmmss.sss
int hour = -1 ; //hh
int minute = -1 ; //mm
int second = -1 ; //ss

bool data_status = false ; // 1: valid    0: invalid

double raw_lat = -1 ; //ddmm.mmmm
double lat1 = -1 ; //dd
double lat2 = -1 ; //mm.mmmm
double latitude = -1 ; 

double raw_lon = -1 ; //dddmm.mmmm
double lon1 = -1 ; //ddd
double lon2 = -1 ; //mm.mmmm
double longitude = -1 ; 

double speed_knot = -1 ; 
double speed_kmh = -1 ; 

double course = -361 ; 

int date = -1 ; //ddmmyy
int day = -1 ; //dd
int month = -1 ; //mm
int year = -1 ; //yy
// ------------------------------------------------------------------------


// Print & Debug
static QueueHandle_t uart_debug_rxqueue;
static const uint8_t buff_len = 127;
char debug_array_int[buff_len] = {0} ; 
char debug_array_double[buff_len] = {0} ; 

// GPS Communication
static QueueHandle_t uart_gps_rxqueue;
char *uart_gps_rxbuf = (char *)malloc(10);
uart_event_t event_gps;
size_t length_gps = 0;
bool first_time = true ; 
unsigned long gps_rec_time = 0 ;
int rmc_state = 0 ; 


void uart_communication_setup() ; 
void print(const char *input) ; 
void print(char input) ; 
void print(int input) ; 
void print(double input) ; 
void print(unsigned long input) ;  
void println(const char *input) ; 
void println(char input) ; 
void println(int input) ; 
void println(double input) ; 
void println(unsigned long input) ; 
void receive_gps_message() ; 
void update_rmc_data() ; 
double decode_latitude() ; 
double decode_longitude() ; 
int decode_hour() ; 
int decode_minute() ; 
int decode_second() ; 
double decode_speed() ; 
int decode_day() ; 
int decode_month() ; 
int decode_year() ; 
void all_data_print() ; 



void setup() 
{
  uart_communication_setup() ; 
  vTaskDelay(500) ; 
}

void loop() 
{
  receive_gps_message() ; 
  update_rmc_data() ; 
   all_data_print() ; 
  vTaskDelay(1200) ; 
}




void uart_communication_setup ()
{
  uart_config_t uart_config_gps = {
    .baud_rate = 9600,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_set_pin(UART_NUM_1, 4, 2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_param_config(UART_NUM_1, &uart_config_gps);
  uart_driver_install(UART_NUM_1, 1536, 1536, 40, &uart_gps_rxqueue, 0);



  uart_config_t uart_config_debug = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_set_pin(UART_NUM_0, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_param_config(UART_NUM_0, &uart_config_debug);
  uart_driver_install(UART_NUM_0, 1536, 1536, 40, &uart_debug_rxqueue, 0);
}

void print(const char *input)
{
  uart_write_bytes(UART_NUM_0, input, strlen(input));
}

void print(char input)
{
  uart_write_bytes(UART_NUM_0, (void *)&input, 1);
}

void print(int input)
{
  snprintf(&debug_array_int[0], buff_len, "%d", input);
  uart_write_bytes(UART_NUM_0, (void *)&debug_array_int[0], strlen(debug_array_int));
}

void print(double input)
{
  snprintf(&debug_array_double[0], buff_len, "%.4f", input);
  uart_write_bytes(UART_NUM_0, (void *)&debug_array_double[0], strlen(debug_array_double));
}

void print(unsigned long input)
{
  snprintf(&debug_array_int[0], buff_len, "%d", input);
  uart_write_bytes(UART_NUM_0, (void *)&debug_array_int[0], strlen(debug_array_int));
}

void println(const char *input)
{
  uart_write_bytes(UART_NUM_0, input, strlen(input));
  const char* enter = "\n" ; 
  uart_write_bytes(UART_NUM_0, enter, 1);
}

void println(char input)
{
  uart_write_bytes(UART_NUM_0, (void *)&input, 1);
  const char* enter = "\n" ; 
  uart_write_bytes(UART_NUM_0, enter, 1);
}

void println(int input)
{
  snprintf(&debug_array_int[0], buff_len, "%d", input);
  uart_write_bytes(UART_NUM_0, (void *)&debug_array_int[0], strlen(debug_array_int));
  const char* enter = "\n" ; 
  uart_write_bytes(UART_NUM_0, enter, 1);
}

void println(unsigned long input)
{
  snprintf(&debug_array_int[0], buff_len, "%d", input);
  uart_write_bytes(UART_NUM_0, (void *)&debug_array_int[0], strlen(debug_array_int));
  const char* enter = "\n" ; 
  uart_write_bytes(UART_NUM_0, enter, 1);
}

void println(double input)
{
  snprintf(&debug_array_double[0], buff_len, "%.4f", input);
  uart_write_bytes(UART_NUM_0, (void *)&debug_array_double[0], strlen(debug_array_double));
  const char* enter = "\n" ; 
  uart_write_bytes(UART_NUM_0, enter, 1);
}

void receive_gps_message()
{
  if (xQueueReceive(uart_gps_rxqueue, (void *)&event_gps, (TickType_t)1))
  {
    if (event_gps.type == UART_DATA)
    {
      uart_get_buffered_data_len(UART_NUM_1, (size_t*)&length_gps);
      uart_read_bytes(UART_NUM_1, uart_gps_rxbuf, (uint32_t)length_gps, 2);
    }
    uart_flush(UART_NUM_1);
  }
}

void update_rmc_data() 
{
  // char* x = "woeuifiowufoiru$GNRMC,144433.00,A,3547.96002,N,05123.44189,E,0.414,,270724,,,A,V*19\n$GNGGA,144433.00,3547.96002,N,05123.44189,E,1,04,4.88,1755.6,M,-18.1,M,,*5F\n$GNGSA,A,3,07,04,09,06,,,,,,,,,9.97,4.88,8.69,1*09\n$GNGSA,A,3,,,,,,sdjlsjlkjvlkjfskv$BDGSV,1,1,01,10,,,31,0*76\n$GNTXT,1,1,01,ANTENNA OK*2Bewdfwaofjioaejrijealkjkdfjkld;;fjkldfjg;kldfjkldfsj" ; 
  for (int i=0 ; i<(int)length_gps ; i++)
  {
    if (rmc_state == 0)
    {
      if ((char)uart_gps_rxbuf[i] == '$')
      {
        rmc_state++ ; 
      }
      else 
      {
        rmc_state = 0 ; 
      }
    }
    else if (rmc_state == 1)
    {
      if ((char)uart_gps_rxbuf[i] == 'G')
      {
        rmc_state++ ; 
      }
      else 
      {
        rmc_state = 0 ; 
      }
    }
    else if (rmc_state == 2)
    {
      if ((char)uart_gps_rxbuf[i] == 'P' || (char)uart_gps_rxbuf[i] == 'N')
      {
        rmc_state++ ; 
      }
      else 
      {
        rmc_state = 0 ; 
      }
    }
    else if (rmc_state == 3)
    {
      if ((char)uart_gps_rxbuf[i] == 'R')
      {
        rmc_state++ ; 
      }
      else 
      {
        rmc_state = 0 ; 
      }
    }
    else if (rmc_state == 4)
    {
      if ((char)uart_gps_rxbuf[i] == 'M')
      {
        rmc_state++ ; 
      }
      else 
      {
        rmc_state = 0 ; 
      }
    }
    else if (rmc_state == 5)
    {
      if ((char)uart_gps_rxbuf[i] == 'C')
      {
        rmc_state++ ; 
      }
      else 
      {
        rmc_state = 0 ; 
      }
    }
    else if (rmc_state == 6)
    {
      if ((char)uart_gps_rxbuf[i] == ',')
      {
        rmc_state++ ; 
      }
      else 
      {
        rmc_state = 0 ; 
      }
    }
    else if (rmc_state == 7)
    {
      if ( isdigit((char)uart_gps_rxbuf[i]) )
      {
        rmc_state++ ; 
        std::size_t offset = 0 ; 
        Time = std::stod(&uart_gps_rxbuf[i], &offset) ; 
        i = i + (int)offset - 1 ; 
      }
      else 
      {
        rmc_state = 0 ; 
      }
    }
    else if (rmc_state == 8)
    {
      if ((char)uart_gps_rxbuf[i] == ',')
      {
        rmc_state++ ; 
      }
      else 
      {
        rmc_state = 0 ; 
      }
    }
    else if (rmc_state == 9)
    {
      if ((char)uart_gps_rxbuf[i] == 'A')
      {
        rmc_state++ ; 
        data_status = true ; 
      }
      else if ((char)uart_gps_rxbuf[i] == 'V')
      {
        rmc_state++ ; 
        data_status = false ; 
      }
      else 
      {
        rmc_state = 0 ; 
        data_status = false ; 
      }
    }
    else if (rmc_state == 10)
    {
      if ((char)uart_gps_rxbuf[i] == ',')
      {
        rmc_state++ ; 
      }
      else 
      {
        rmc_state = 0 ; 
      }
    }
    else if (rmc_state == 11)
    {
      if ( isdigit((char)uart_gps_rxbuf[i]) )
      {
        rmc_state++ ; 
        std::size_t offset = 0 ; 
        raw_lat = std::stod(&uart_gps_rxbuf[i], &offset) ; 
        i = i + (int)offset - 1 ; 
      }
      else 
      {
        rmc_state = 0 ; 
      }
    }
    else if (rmc_state == 12)
    {
      if ((char)uart_gps_rxbuf[i] == ',')
      {
        rmc_state++ ; 
      }
      else 
      {
        rmc_state = 0 ; 
      }
    }
    else if (rmc_state == 13)
    {
      if ((char)uart_gps_rxbuf[i] == 'N' || (char)uart_gps_rxbuf[i] == 'S')
      {
        rmc_state++ ; 
      }
      else 
      {
        rmc_state = 0 ; 
      }
    }
    else if (rmc_state == 14)
    {
      if ((char)uart_gps_rxbuf[i] == ',')
      {
        rmc_state++ ; 
      }
      else 
      {
        rmc_state = 0 ; 
      }
    }
    else if (rmc_state == 15)
    {
      if ( isdigit((char)uart_gps_rxbuf[i]) )
      {
        rmc_state++ ; 
        std::size_t offset = 0 ; 
        raw_lon = std::stod(&uart_gps_rxbuf[i], &offset) ; 
        i = i + (int)offset - 1 ; 
      }
      else 
      {
        rmc_state = 0 ; 
      }
    }
    else if (rmc_state == 16)
    {
      if ((char)uart_gps_rxbuf[i] == ',')
      {
        rmc_state++ ; 
      }
      else 
      {
        rmc_state = 0 ; 
      }
    }
    else if (rmc_state == 17)
    {
      if ((char)uart_gps_rxbuf[i] == 'E' || (char)uart_gps_rxbuf[i] == 'W')
      {
        rmc_state++ ; 
      }
      else 
      {
        rmc_state = 0 ; 
      }
    }
    else if (rmc_state == 18)
    {
      if ((char)uart_gps_rxbuf[i] == ',')
      {
        rmc_state++ ; 
      }
      else 
      {
        rmc_state = 0 ; 
      }
    }
    else if (rmc_state == 19)
    {
      if ( isdigit((char)uart_gps_rxbuf[i]) )
      {
        rmc_state++ ; 
        std::size_t offset = 0 ; 
        speed_knot = std::stod(&uart_gps_rxbuf[i], &offset) ; 
        i = i + (int)offset - 1 ; 
      }
      else 
      {
        // rmc_state = 0 ; 
        rmc_state++ ; 
        i-- ; 
      }
    }
    else if (rmc_state == 20)
    {
      if ((char)uart_gps_rxbuf[i] == ',')
      {
        rmc_state++ ; 
      }
      else 
      {
        rmc_state = 0 ; 
      }
    }
    else if (rmc_state == 21)
    {
      if ( isdigit((char)uart_gps_rxbuf[i]) )
      {
        rmc_state++ ; 
        std::size_t offset = 0 ; 
        course = std::stod(&uart_gps_rxbuf[i], &offset) ; 
        i = i + (int)offset - 1 ; 
      }
      else 
      {
        // rmc_state = 0 ; 
        rmc_state++ ; 
        i-- ; 
      }
    }
    else if (rmc_state == 22)
    {
      if ((char)uart_gps_rxbuf[i] == ',')
      {
        rmc_state++ ; 
      }
      else 
      {
        rmc_state = 0 ; 
      }
    }
    else if (rmc_state == 23)
    {
      if ( isdigit((char)uart_gps_rxbuf[i]) )
      {
        rmc_state++ ; 
        std::size_t offset = 0 ; 
        date = std::stoi(&uart_gps_rxbuf[i], &offset) ; 
        i = i + (int)offset - 1 ; 
      }
      else 
      {
        rmc_state = 0 ; 
      }
    }
    else if (rmc_state == 24)
    {
      rmc_state = 0 ; 
    }
    else
    {
      println("WTF!?") ; 
    }
  }



  // println(uart_gps_rxbuf) ; 
  // println("*************************************************************") ; 
  // println((char)uart_gps_rxbuf[3]) ; 
}

double decode_latitude()
{
  double temp = raw_lat * 10000 ; 
  lat1 = ((int)(temp/1000000))%100 ; //dd
  lat2 = (((int)(temp))%1000000)/10000.0 ; //mm.mmmm
  latitude = lat1 + (lat2/60.0) ; 
  return latitude ; 
}

double decode_longitude()
{
  double temp = raw_lon * 10000 ; 
  lon1 = ((int)(temp/1000000))%1000 ; //ddd
  lon2 = (((int)(temp))%1000000)/10000.0 ; //mm.mmmm
  longitude = lon1 + (lon2/60.0) ; 
  return longitude ; 
}

int decode_hour() 
{
  hour = ((int)(Time)/10000)%100 ; 
  return hour ; 
}

int decode_minute() 
{
  minute = ((int)(Time)/100)%100 ; 
  return minute ; 
}

int decode_second()
{
  second = ((int)(Time))%100 ; 
  return second ; 
}

double decode_speed()
{
  speed_kmh = speed_knot * KNOT_TO_KMH ; 
  return speed_kmh ; 
}

int decode_day()
{
  day = ((int)(date)/10000)%100 ; 
  return day ; 
}

int decode_month() 
{
  month = ((int)(date)/100)%100 ; 
  return month ;
}

int decode_year()
{
  year = ((int)(date))%100 ; 
  return year ; 
}

void all_data_print()
{
  print("Time: ") ; print(decode_hour()) ; print(":") ;  print(decode_minute()) ; print(":") ;  print(decode_second()) ;
  print("    Lat: ") ; print(decode_latitude()) ; 
  print("    Lon: ") ; print(decode_longitude()) ; 
  print("    Date: ") ; print(decode_month()) ; print("/") ; print(decode_day()) ; print("/") ; print("20") ; print(decode_year()) ; 
  print("    Status: ") ; print(data_status) ; 
  print("    Course: ") ; print(course) ; 
  print("    Speed: ") ; println(decode_speed()) ;  
}

