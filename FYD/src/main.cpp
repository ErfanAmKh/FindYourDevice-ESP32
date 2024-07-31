#include <Arduino.h>
#include "driver/uart.h"

// What we wnat from gps---------------------------------------------------
double Time = -1 ; 
int hour = -1 ; 
int minute = -1 ; 
int second = -1 ; 

bool data_status = false ; 

double raw_lat = -1 ; 
double lat1 = -1 ; 
double lat2 = -1 ; 

double raw_lon = -1 ; 
double lon1 = -1 ; 
double lon2 = -1 ; 

double speed_knot = -1 ; 
double speed_kmh = -1 ; 

double course = -361 ; 

int date = -1 ; 
int day = -1 ; 
int month = -1 ; 
int year = -1 ; 
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



void setup() 
{
  uart_communication_setup() ; 
  vTaskDelay(500) ; 
}

void loop() 
{
  receive_gps_message() ; 
  update_rmc_data() ; 
  print("Time: ") ;
  print(Time) ; 
  print("    Lat: ") ; 
  print(raw_lat) ; 
  print("    Lon: ") ;
  print(raw_lon) ; 
  print("    Date: ") ; 
  print(date) ;
  print("    Status: ") ; 
  print(data_status) ; 
  print("    Course: ") ; 
  print(course) ; 
  print("    Speed: ") ; 
  println(speed_knot) ;   
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
  char* x = "$GNRMC,144433.00,A,3547.96002,N,05123.44189,E,0.414,,270724,,,A,V*19\n$GNGGA,144433.00,3547.96002,N,05123.44189,E,1,04,4.88,1755.6,M,-18.1,M,,*5F\n$GNGSA,A,3,07,04,09,06,,,,,,,,,9.97,4.88,8.69,1*09\n$GNGSA,A,3,,,,,,,,,,,,,9.97,4.88,8.69,4*00\n$GPGSV,3,1,12,02,00,188,,03,62,157,,04,65,010,24,06,23,292,28,0*67\n$GPGSV,3,2,12,07,33,237,17,09,44,308,31,11,03,326,21,14,44,128,,0*69\n$GPGSV,3,3,12,16,41,096,09,26,26,054,08,30,11,241,09,31,15,048,,0*6C\n$BDGSV,1,1,01,10,,,31,0*76\n$GNTXT,1,1,01,ANTENNA OK*2B" ; 
  for (int i=0 ; i<(int)length_gps ; i++)
  {
    if (rmc_state == 0)
    {
      if ((char)x[i] == '$')
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
      if ((char)x[i] == 'G')
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
      if ((char)x[i] == 'P' || (char)x[i] == 'N')
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
      if ((char)x[i] == 'R')
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
      if ((char)x[i] == 'M')
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
      if ((char)x[i] == 'C')
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
      if ((char)x[i] == ',')
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
      if ( isdigit((char)x[i]) )
      {
        rmc_state++ ; 
        std::size_t offset = 0 ; 
        Time = std::stod(&x[i], &offset) ; 
        i = i + (int)offset - 1 ; 
      }
      else 
      {
        rmc_state = 0 ; 
      }
    }
    else if (rmc_state == 8)
    {
      if ((char)x[i] == ',')
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
      if ((char)x[i] == 'A')
      {
        rmc_state++ ; 
        data_status = true ; 
      }
      else if ((char)x[i] == 'V')
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
      if ((char)x[i] == ',')
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
      if ( isdigit((char)x[i]) )
      {
        rmc_state++ ; 
        std::size_t offset = 0 ; 
        raw_lat = std::stod(&x[i], &offset) ; 
        i = i + (int)offset - 1 ; 
      }
      else 
      {
        rmc_state = 0 ; 
      }
    }
    else if (rmc_state == 12)
    {
      if ((char)x[i] == ',')
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
      if ((char)x[i] == 'N' || (char)x[i] == 'S')
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
      if ((char)x[i] == ',')
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
      if ( isdigit((char)x[i]) )
      {
        rmc_state++ ; 
        std::size_t offset = 0 ; 
        raw_lon = std::stod(&x[i], &offset) ; 
        i = i + (int)offset - 1 ; 
      }
      else 
      {
        rmc_state = 0 ; 
      }
    }
    else if (rmc_state == 16)
    {
      if ((char)x[i] == ',')
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
      if ((char)x[i] == 'E' || (char)x[i] == 'W')
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
      if ((char)x[i] == ',')
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
      if ( isdigit((char)x[i]) )
      {
        rmc_state++ ; 
        std::size_t offset = 0 ; 
        speed_knot = std::stod(&x[i], &offset) ; 
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
      if ((char)x[i] == ',')
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
      if ( isdigit((char)x[i]) )
      {
        rmc_state++ ; 
        std::size_t offset = 0 ; 
        course = std::stod(&x[i], &offset) ; 
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
      if ((char)x[i] == ',')
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
      if ( isdigit((char)x[i]) )
      {
        rmc_state++ ; 
        std::size_t offset = 0 ; 
        date = std::stoi(&x[i], &offset) ; 
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



