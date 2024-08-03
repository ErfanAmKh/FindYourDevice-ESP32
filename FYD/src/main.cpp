#include <Arduino.h>
#include "driver/uart.h"

const char passcode[8] = {'A', 'B', 'C', 'D', '1', '2', '3', '4'} ; 

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
char *uart_serial_rxbuf = (char *)malloc(10);
uart_event_t event_serial;
size_t length_serial = 0;
static const uint8_t buff_len = 127;
char debug_array_int[buff_len] = {0} ; 
char debug_array_double[buff_len] = {0} ; 

// GPS Communication
static QueueHandle_t uart_gps_rxqueue;
char *uart_gps_rxbuf = (char *)malloc(200);
uart_event_t event_gps;
size_t length_gps = 0;
bool first_time = true ; 
int rmc_state = 0 ; 
unsigned long gps_time = 0 ; 

// Sim800 Communication
static QueueHandle_t uart_sim800_rxqueue;
char *uart_sim800_rxbuf = (char *)malloc(10);
uart_event_t event_sim800;
size_t length_sim800 = 0;
char sim800_array_int[buff_len] = {0} ; 
char sim800_array_double[buff_len] = {0} ; 
static const uint8_t sms_buff_len = 127;
unsigned long http_request_time = 0 ; 

// General FSM
unsigned long wake_up_time ; 
bool location_reported = false ; 
bool need_to_report = false ; 


void uart_communication_setup() ; 
void print(const char *input) ; 
void print(String input) ; 
void print(char input) ; 
void print(int input) ; 
void print(double input) ; 
void print(unsigned long input) ;  
void println(const char *input) ; 
void println(String input) ; 
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
void print_sim800(const char *input) ; 
void print_sim800(String input) ; 
void print_sim800(char input) ; 
void print_sim800(int input) ; 
void print_sim800(double input) ; 
void print_sim800(unsigned long input) ;  
void println_sim800(const char *input) ; 
void println_sim800(String input) ; 
void println_sim800(char input) ; 
void println_sim800(int input) ; 
void println_sim800(double input) ; 
void println_sim800(unsigned long input) ; 
void receive_sim800_message() ; 
void communicate_with_sim800(char* command, TickType_t del) ; 
void communicate_with_sim800(String command, TickType_t del) ; 
void communicate_with_sim800(char command, TickType_t del) ; 
void check_sim800_initialization() ; 
void send_sms(char* txt, String phone_number) ; 
void config_gprs() ; 
void post(String letter) ; 
bool check_code() ; 
bool passcode_is_match(int counter) ; 
void print_wakeup_reason() ; 



void setup() 
{
  location_reported = false ; 
  need_to_report = false ; 
  uart_communication_setup() ; 
  vTaskDelay(500) ; 
  println("Uarts initialized...") ; 
  check_sim800_initialization() ; 
  vTaskDelay(500) ; 
  println("sim800 checked...") ; 
  vTaskDelay(500) ; 
  config_gprs() ; 
  println("gprs checked...") ; 
  vTaskDelay(500) ; 
  print_wakeup_reason() ; 
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 0) ; 
  println("wake up config done...") ; 
  vTaskDelay(500) ; 
  wake_up_time = millis() ; 
}

void loop() 
{
  if (!need_to_report && millis()<(wake_up_time + 1200000))
  {
    if (check_code())
    {
      need_to_report = true ; 
    }
    else 
    {
      need_to_report = false ; 
    }
  }
  else if ( (!need_to_report || location_reported) && millis()>(wake_up_time + 1200000))
  {
    println("going to sleep...") ; 
    vTaskDelay(500) ; 
    esp_deep_sleep_start() ; 
  }
  else if (!location_reported && need_to_report)
  {
    if (data_status)
    {
      char my_txt[sms_buff_len] = {} ; 
      snprintf(&my_txt[0], sms_buff_len, "Long: %.4f   Lat: %.4f", decode_longitude(), decode_latitude()) ; 
      String PN = "+989021229753" ; 
      send_sms(my_txt, PN) ;
      vTaskDelay(500) ; 
      println("text sent.") ; 
      location_reported = true ; 
      String my_letter = "Long: " +  String(decode_longitude()) + "   Lat: " + String(decode_latitude()) ; 
      post(my_letter) ; 
      http_request_time = millis() ; 
    }
  }
  else 
  {
    if (millis() > http_request_time + 20000)
    {
      http_request_time = millis() ; 
      String my_letter = "Long: " +  String(decode_longitude()) + "   Lat: " + String(decode_latitude()) ; 
      post(my_letter) ; 
    }
  }

  if (millis() > (gps_time + 1200))
  {
    gps_time = millis() ; 
    receive_gps_message() ; 
    update_rmc_data() ; 
    // all_data_print() ; 
  }

  if (check_code())
  {
    println("YYYYYYYYYEEEEEEEEEEESSSSSSSS") ; 
    println("YYYYYYYYYEEEEEEEEEEESSSSSSSS") ; 
    println("YYYYYYYYYEEEEEEEEEEESSSSSSSS") ; 
    println("YYYYYYYYYEEEEEEEEEEESSSSSSSS") ; 
    println("YYYYYYYYYEEEEEEEEEEESSSSSSSS") ; 
    println("YYYYYYYYYEEEEEEEEEEESSSSSSSS") ; 
    println("YYYYYYYYYEEEEEEEEEEESSSSSSSS") ; 
    println("YYYYYYYYYEEEEEEEEEEESSSSSSSS") ; 
    println("YYYYYYYYYEEEEEEEEEEESSSSSSSS") ; 
    println("YYYYYYYYYEEEEEEEEEEESSSSSSSS") ; 
    println("YYYYYYYYYEEEEEEEEEEESSSSSSSS") ; 
    println("YYYYYYYYYEEEEEEEEEEESSSSSSSS") ; 
  }

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
  uart_set_pin(UART_NUM_1, 4, 23, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
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



  uart_config_t uart_config_sim800 = {
    .baud_rate = 9600,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_set_pin(UART_NUM_2, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_param_config(UART_NUM_2, &uart_config_sim800);
  uart_driver_install(UART_NUM_2, 1536, 1536, 40, &uart_sim800_rxqueue, 0);
}

void print(const char *input)
{
  uart_write_bytes(UART_NUM_0, input, strlen(input));
}

void print(String input)
{
  uart_write_bytes(UART_NUM_0, (void*)&input[0], input.length()) ; 
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

void println(String input)
{
  uart_write_bytes(UART_NUM_0, (void*)&input[0], input.length()) ; 
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
      // print(" gps:     ") ; 
      // println((int)length_gps) ; 
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

void print_sim800(const char *input)
{
  uart_write_bytes(UART_NUM_2, input, strlen(input));
}

void print_sim800(String input)
{
  uart_write_bytes(UART_NUM_2, (void*)&input[0], input.length()) ; 
}

void print_sim800(char input)
{
  uart_write_bytes(UART_NUM_2, (void *)&input, 1);
}

void print_sim800(int input)
{
  snprintf(&sim800_array_int[0], buff_len, "%d", input);
  uart_write_bytes(UART_NUM_2, (void *)&sim800_array_int[0], strlen(sim800_array_int));
}

void print_sim800(double input)
{
  snprintf(&sim800_array_double[0], buff_len, "%.4f", input);
  uart_write_bytes(UART_NUM_2, (void *)&sim800_array_double[0], strlen(sim800_array_double));
}

void print_sim800(unsigned long input)
{
  snprintf(&sim800_array_int[0], buff_len, "%d", input);
  uart_write_bytes(UART_NUM_2, (void *)&sim800_array_int[0], strlen(sim800_array_int));
}

void println_sim800(const char *input)
{
  uart_write_bytes(UART_NUM_2, input, strlen(input));
  const char* enter = "\n" ; 
  uart_write_bytes(UART_NUM_2, enter, 1);
}

void println_sim800(String input)
{
  uart_write_bytes(UART_NUM_2, (void*)&input[0], input.length()) ; 
  const char* enter = "\n" ; 
  uart_write_bytes(UART_NUM_2, enter, 1);
}

void println_sim800(char input)
{
  uart_write_bytes(UART_NUM_2, (void *)&input, 1);
  const char* enter = "\n" ; 
  uart_write_bytes(UART_NUM_2, enter, 1);
}

void println_sim800(int input)
{
  snprintf(&sim800_array_int[0], buff_len, "%d", input);
  uart_write_bytes(UART_NUM_2, (void *)&sim800_array_int[0], strlen(sim800_array_int));
  const char* enter = "\n" ; 
  uart_write_bytes(UART_NUM_2, enter, 1);
}

void println_sim800(unsigned long input)
{
  snprintf(&sim800_array_int[0], buff_len, "%d", input);
  uart_write_bytes(UART_NUM_2, (void *)&sim800_array_int[0], strlen(sim800_array_int));
  const char* enter = "\n" ; 
  uart_write_bytes(UART_NUM_2, enter, 1);
}

void println_sim800(double input)
{
  snprintf(&sim800_array_double[0], buff_len, "%.4f", input);
  uart_write_bytes(UART_NUM_2, (void *)&sim800_array_double[0], strlen(sim800_array_double));
  const char* enter = "\n" ; 
  uart_write_bytes(UART_NUM_2, enter, 1);
}

void receive_sim800_message()
{
  if (xQueueReceive(uart_sim800_rxqueue, (void *)&event_sim800, (TickType_t)1))
  {
    // println("in queue receive if") ; 
    if (event_sim800.type == UART_DATA)
    {
      // println("in event if") ; 
      uart_get_buffered_data_len(UART_NUM_2, (size_t*)&length_sim800);
      // print("after buffer length: ") ; 
      // println((int)length_sim800) ; 
      uart_read_bytes(UART_NUM_2, uart_sim800_rxbuf, (uint32_t)length_sim800, 2);
      // print("after reading...") ; 
    }
    uart_flush(UART_NUM_2);
    // print("after flush...") ; 
  }
}

void communicate_with_sim800(char* command, TickType_t del = 100)
{
  // print("The command is: ") ; 
  // println(&command[0]) ; 
  // println("before writing...") ; 
  println_sim800(&command[0]) ; 
  // println("after writing...") ; 
  vTaskDelay(del) ; 
  // println("after del...") ; 
  receive_sim800_message() ; 
  // println("after receiving...") ; 
  // println(&uart_sim800_rxbuf[0]) ; 
  // println("after printting...") ; 
}

void communicate_with_sim800(String command, TickType_t del = 100)
{
  // print("The command is: ") ; 
  // println(command) ; 
  // println("before writing...") ; 
  println_sim800(&command[0]) ; 
  // println("after writing...") ;
  vTaskDelay(del) ; 
  // println("after del...") ; 
  receive_sim800_message() ; 
  // println("after receiving...") ; 
  // println(&uart_sim800_rxbuf[0]) ; 
  // println("after printting...") ; 
}

void communicate_with_sim800(char command, TickType_t del = 100)
{
  // print("The command is: ") ; 
  // println(command) ; 
  // println("before writing...") ; 
  println_sim800(command) ; 
  // println("after writing...") ;
  vTaskDelay(del) ; 
  // println("after del...") ; 
  receive_sim800_message() ; 
  // println("after receiving...") ; 
  // println(&uart_sim800_rxbuf[0]) ; 
  // println("after printting...") ;
}

void check_sim800_initialization()
{
  communicate_with_sim800("AT") ; 
  vTaskDelay(100) ; 
  communicate_with_sim800("AT+CSQ") ; 
  vTaskDelay(100) ; 
  communicate_with_sim800("AT+CCID") ; 
  vTaskDelay(100) ; 
  communicate_with_sim800("AT+CREG?") ; 
  vTaskDelay(100) ; 
  communicate_with_sim800("ATI") ; 
  vTaskDelay(100) ; 
  communicate_with_sim800("AT+CBC") ; 
  vTaskDelay(100) ; 
  communicate_with_sim800("AT+COPS?") ; 
  vTaskDelay(100) ; 
}

void send_sms(char* txt, String phone_number)
{
  communicate_with_sim800("AT+CMGF=1") ; 
  vTaskDelay(100) ; 
  String temp = "AT+CMGS=\"" + phone_number + "\"" ; 
  communicate_with_sim800(temp) ; 
  vTaskDelay(100) ; 
  communicate_with_sim800(&txt[0]) ; 
  vTaskDelay(100) ; 
  char ctrl_Z = 26 ; 
  communicate_with_sim800(ctrl_Z) ; 
  vTaskDelay(100) ; 
}

void config_gprs()
{
  communicate_with_sim800("AT+SAPBR=3,1,Contype,GPRS", 250) ; 
  vTaskDelay(2000) ; 
  communicate_with_sim800("AT+SAPBR=3,1,APN,APN", 250) ; 
  vTaskDelay(2000) ; 
}

void post(String letter)
{
  communicate_with_sim800("AT+SAPBR=1,1", 250);
  vTaskDelay(2000) ; 
  communicate_with_sim800("AT+SAPBR=2,1", 250);
  vTaskDelay(2000) ; 
  communicate_with_sim800("AT+HTTPINIT", 250);
  vTaskDelay(2000) ; 
  communicate_with_sim800("AT+HTTPPARA=CID,1", 250);
  vTaskDelay(2000) ; 
  communicate_with_sim800("AT+HTTPPARA=URL,http://httpbin.org/get" , 250);
  vTaskDelay(2000) ; 
  communicate_with_sim800("AT+HTTPPARA=CONTENT,application/x-www-form-urlencoded", 250);
  vTaskDelay(2000) ; 
  communicate_with_sim800("AT+HTTPDATA=192,5000", 250);
  vTaskDelay(100) ; 
  communicate_with_sim800(letter, 50);
  vTaskDelay(10000) ; 
  communicate_with_sim800("AT+HTTPACTION=1", 250);
  vTaskDelay(2000) ; 
  communicate_with_sim800("AT+HTTPREAD", 15000);
  vTaskDelay(2000) ; 
  // receive_sim800_message() ; 
  // println(&uart_sim800_rxbuf[0]) ; 
  communicate_with_sim800("AT+HTTPTERM", 250);
  vTaskDelay(2000) ; 
  communicate_with_sim800("AT+SAPBR=0,1", 250);
  vTaskDelay(2000) ; 
}

bool check_code() 
{
  communicate_with_sim800("AT+CMGF=1", 250) ; 
  vTaskDelay(1000) ; 
  println_sim800("AT+CMGL=\"REC UNREAD\"") ; 
  vTaskDelay(5000) ; 
  receive_sim800_message() ; 
  // print("before for:  ") ; 
  // println((int)(length_sim800)) ; 
  // print("  Buffer:  ") ; 
  // println(&uart_sim800_rxbuf[0]) ; 
  for (int i=0 ; i<(((int)length_sim800)-8) ; i++)
  {
    if (passcode_is_match(i))
    {
      return true ; 
    }
  }
  return false ; 
}

bool passcode_is_match(int counter)
{
  if ( ((char)uart_sim800_rxbuf[counter]==passcode[0]) && ((char)uart_sim800_rxbuf[counter+1]==passcode[1]) && ((char)uart_sim800_rxbuf[counter+2]==passcode[2]) && ((char)uart_sim800_rxbuf[counter+3]==passcode[3]) && ((char)uart_sim800_rxbuf[counter+4]==passcode[4]) && ((char)uart_sim800_rxbuf[counter+5]==passcode[5] && ((char)uart_sim800_rxbuf[counter+6]==passcode[6]) && ((char)uart_sim800_rxbuf[counter+7]==passcode[7])) )
  {
    return true ; 
  }
  else 
  {
    return false ; 
  }
}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : println("Wakeup caused by ULP program"); break;
    default : println("default accured"); break;
  }
}






















// #include <Arduino.h>
// int state = 0 ; 
// int temp_time = 0 ; 


// // void IRAM_ATTR my_fcn () ; 
// void test_sim800_module() ; 
// void updateSerial() ; 
// void send_SMS() ; 


// void setup() {
//   Serial.begin(115200);
//   Serial2.begin(9600);
//   delay(3000);
//  test_sim800_module();
// //  send_SMS();
// //  pinMode(4, INPUT) ; 
//   // attachInterrupt(digitalPinToInterrupt(4), my_fcn, FALLING);
// }
// void loop() {
//   updateSerial();
// //  Serial.println(state) ; 
// }

// // void IRAM_ATTR my_fcn ()
// // {
// //   state++ ; 
// // }


// void test_sim800_module()
// {
//   Serial2.println("AT");
//   updateSerial();
//   Serial.println();
//   Serial2.println("AT+CSQ");
//   updateSerial();
//   Serial2.println("AT+CCID");
//   updateSerial();
//   Serial2.println("AT+CREG?");
//   updateSerial();
//   Serial2.println("ATI");
//   updateSerial();
//   Serial2.println("AT+CBC");
//   updateSerial();
// }


// void updateSerial()
// {
// //  delay(500);
//   while (Serial.available())
//   {
//     Serial2.write(Serial.read());//Forward what Serial received to Software Serial Port
//   }
//   while (Serial2.available())
//   {
//   Serial.write(Serial2.read());//Forward what Software Serial received to Serial Port
//   }
// }


// void send_SMS()
// {
// //  Serial2.println("AT+CSCS=\"GSM\"") ;
// //  updateSerial() ;  
//   Serial2.println("AT+CMGF=1"); 
//   updateSerial();
//   Serial2.println("AT+CMGS=\"+989021229753\"");
//   updateSerial();
//   Serial2.print("Testing SMS sim800"); 
//   updateSerial();
//   Serial.println();
//   Serial.println("Message Sent");
//   Serial2.write(26);
// }
