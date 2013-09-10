

// Для совместимости как с Arduino 1.0 так и предыдущих версий

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <Wire.h>
#include <DS1307.h>
#include <math.h>
#include <IRremote.h>
#include "DHT.h"

#define DHTTYPE DHT11 // DHT 11  модель используемого датчика
#define DHTPIN 2 
#define PIR_MOTION_SENSOR 7 //пин датчика движения 7ой
  
DHT dht(DHTPIN, DHTTYPE);
/*Коды кнопок на пульте*/

const int RED_KN=0xE12448B7; //Красная кнопка
const int SEL_KN=0xE12418E7;
 
const int KN_1=0xE12440BF; 
const int KN_4=0xE124B847;

const int KN_2=0xE12428D7;
const int KN_5=0xE124F00F;

const int KN_3=0xE1246897;
const int KN_6=0xE1249867;

const int KN_8=0xE124B04F;
const int KN_0=0xE12430CF;

const int KN_9=0xE124F807;
const int KN_DISP=0xE124C837;

const int KN_7=0xE1247887;
const int KN_RESH=0xE124E817;

const int thresholdvalue=200;    //Порог при котором свет должен включиться
 
float Rsensor; 

void configure_mb_slave(long baud, char parity, char txenpin);

/*
 * update_mb_slave(slave_id, holding_regs_array, number_of_regs)
 * 
 * Проверяем, есть ли любой корректный запрос от Modbus Master. Если есть,
 * Выполняет запрашиваемые меры
 * 
 * slave: slave id (1 to 127)
 * регистры: массив с регистров временного хранения. Они начинаются с адреса 1
 * regs_size: общее число регистров временного хранения (holding registers)
 * Возвращает: 0, если нет запроса от мастера,
 * 	NO_REPLY (-1) если ответ не посылается в ведущий
 * 	Исключение код (от 1 до 4) в случае исключения Modbus
 * 	  Количество байт, переданных в качестве ответа (> 4) если это допустимо.
 */

int update_mb_slave(unsigned char slave, int *regs,
unsigned int regs_size);


/* Modbus RTU общих параметров, Master должен использовать те же параметры */
enum {
        MB_SLAVE = 1,	/* modbus slave id */
};
/* Используемые Slave регистры */
enum {        
        MB_HR,
        MB_MIN,
        MB_SEC,
        MB_TEMP_HI,
        MB_TEMP_LO,
        MB_TEMP_HOME,
        MB_HUM,
        MB_RELAY1,
        MB_RELAY2,
        MB_RELAY3,
        MB_TIME,
        MB_FOOOO,
        MB_REGS	 	/* общее количество регистров на slave */
};

int regs[MB_REGS];	/* это slave's modbus  */

int RECV_PIN = 6; 
int K;
IRrecv irrecv(RECV_PIN);

decode_results results;

void setup() 

{
{
  Serial.begin(115200);
  irrecv.enableIRIn(); // Запуск приемника
  dht.begin();
}
   /*  Пример настройки Modbus, Master должен использовать те же параметры COM */
        /* 115200 bps, 8N1, two-device network */
        configure_mb_slave(115200, 'n', 0);
        pinMode(10, OUTPUT);   
        pinMode(11, OUTPUT); 
        pinMode(12, OUTPUT);
        pinMode(7, INPUT);    
 /*  
// Установить время и дату
  RTC.stop();
  RTC.set(DS1307_SEC,00);        //Установить секунды
  RTC.set(DS1307_MIN,2);   //Установить минуты
  RTC.set(DS1307_HR,22);       //Установить часы
  RTC.set(DS1307_DOW,5);       //Установить день
  RTC.set(DS1307_DATE,31);       //Установить дату
  RTC.set(DS1307_MTH,5);        //Установить месяц
  RTC.set(DS1307_YR,13);         //Установить год
  RTC.start();
*/
}
int TIME_ON;
int TIME_O;
float h;
float t;
float temperature;
int B=3975; 
float resistance;
int sensorValue1;
int sensorValue2;
int a;
int S=2;
int k;
int m;


void loop() 


{
       
        
        /* Это все для Modbus slave */
	update_mb_slave(MB_SLAVE, regs, MB_REGS);

	/* Здесь код*/
  
 h = dht.readHumidity();//считываем влажность
 t = dht.readTemperature();
 regs[MB_HUM] = h; //запись в регистр
 regs[MB_TEMP_HOME] = t;
 
 regs[MB_HR] = RTC.get(DS1307_HR,true);
 regs[MB_MIN] = RTC.get(DS1307_MIN,false);
 regs[MB_SEC] = RTC.get(DS1307_SEC,false);
 regs[MB_TIME] = TIME_O;

        if (regs[MB_RELAY1] != 0)     digitalWrite(10, HIGH);
        else                          digitalWrite(10, LOW);

       
        if (regs[MB_RELAY2] != 0)     digitalWrite(11, HIGH);
        else                          digitalWrite(11, LOW);
        
        if (regs[MB_RELAY3] != 0)     digitalWrite(12, HIGH);
        else                          digitalWrite(12, LOW);   
        
        
      if (irrecv.decode(&results)) {
   // Serial.println(results.value, HEX);
   K = results.value, HEX;
    irrecv.resume(); // Получим следующее значение
  }
  
  //свет 1
  if (K == KN_1)           
{   regs[MB_RELAY1] = 1;
}
  if ((K == KN_4) & (regs[MB_RELAY1] !=0)){
 regs[MB_RELAY1] = 0;
 }
 
    //свет 2
    if (K == KN_2) {          
 regs[MB_RELAY2] = 1;
}
  if ((K == KN_5) & (regs[MB_RELAY2] !=0)){
 regs[MB_RELAY2] = 0;
  }
  //свет 3
   if (K == KN_3) {          
  regs[MB_RELAY3] = 1;
}
  if ((K == KN_6) & (regs[MB_RELAY3] !=0)){
regs[MB_RELAY3] = 0;
  }
        a =analogRead(0);
        resistance=(float)(1023-a)*10000/a; 
        temperature=1/(log(resistance/10000)/B+1/298.15)-273.15;
        *(float*) &(regs[MB_TEMP_HI]) = temperature;  
        
         sensorValue1 = digitalRead(PIR_MOTION_SENSOR);
         
         sensorValue2 = analogRead(2); 
  Rsensor=(float)(1023-sensorValue2)*10/sensorValue2;
    /************************* ОТКЛЮЧЕНИЕ РЕЖИМОВ И ОБЕСТОЧИВАНИЕ *******************************/   
    if (K == RED_KN) //отключение всех режимов
    {
      S = 3;
    }
    
    if (K == SEL_KN)// обесточивание системы
   {
     S = 4;
   }
   if (S == 4)
{
  regs[MB_RELAY1] = 0;
  regs[MB_RELAY2] = 0;
  regs[MB_RELAY3] = 0;
}
    /************************* РЕЖИМ "ОТПУСК" *******************************/   




 if(K == KN_0)
 {
  S = 2;  
 }

   
 if (K==KN_8)
 S=1;
 
 if (S==1){
 if (RTC.get(DS1307_HR,true) <19)  /* Режим "Вечер" включается, если времени больше восьми часов вечера*/   
      {
 {
  m = RTC.get(DS1307_SEC,false);
 TIME_O = (m);
}
 if (TIME_O > 59) 
              TIME_O = TIME_O - 60;
               if ((TIME_O== 15) || (TIME_O == 45) && (S==1))          
              regs[MB_RELAY1] = 1;      
if  ((TIME_O== 25) || (TIME_O == 55) && (S!=2))  
   regs[MB_RELAY1] = 0;     
                 if ((TIME_O== 25) || (TIME_O == 55) && (S==1))          
              regs[MB_RELAY2] = 1;      
if  ((TIME_O== 35) || (TIME_O == 5) && (S!=2))  
   regs[MB_RELAY2] = 0;    
             if ((TIME_O== 5) || (TIME_O == 25) && (S==1))          
              regs[MB_RELAY3] = 1;      
if  ((TIME_O== 15) || (TIME_O == 35) && (S!=2))  
   regs[MB_RELAY3] = 0;    
    }

 }
 

  
  if (S == 2)
  {
       /************************* РЕЖИМ "ВЕЧЕР" *******************************/
  if (RTC.get(DS1307_HR,true) > 19)  /* Режим "Вечер" включается, если времени больше восьми часов вечера*/   
      {
       
   if(sensorValue1 == HIGH)//Есть движение?
	{
		regs[MB_RELAY2] = 1;// включаем свет в коридоре
                TIME_ON = ((RTC.get(DS1307_SEC,false)) + 20);// спрашиваем время включения

	}
    if (TIME_ON > 59)
              TIME_ON = TIME_ON - 60;
     if ((RTC.get(DS1307_SEC,false)) == TIME_ON) // если прошло 20 секунд, выключаем свет
                 regs[MB_RELAY2] = 0;
          
  if(Rsensor>thresholdvalue) //&& (regs[MB_RELAY2]!=1))
       regs[MB_RELAY3] = 1;
       
        
      }
       /************************* РЕЖИМ "НОЧЬ" *******************************/
    if (RTC.get(DS1307_HR,true) < 6)   //режим Ночь включается если времени больше полуночи, но меньше 6ти утра
    {
       if(sensorValue1 == HIGH)//Есть движение?
{
  regs[MB_RELAY2] = 1;// включаем свет в коридоре
  TIME_ON = ((RTC.get(DS1307_SEC,false)) + 20);// спрашиваем время включения
}
if (TIME_ON > 59)
              TIME_ON = TIME_ON - 60;
               if ((RTC.get(DS1307_SEC,false)) == TIME_ON) // если прошло 20 секунд, выключаем свет
                 regs[MB_RELAY2] = 0;
    }
    
/*
  Serial.print(RTC.get(DS1307_HR,true)); //считать час, а также обновить все значения, нажав на истинную
  Serial.print(":");
  Serial.print(RTC.get(DS1307_MIN,false));//читать минуты без обновления (ложь)
  Serial.print(":");
  Serial.print(RTC.get(DS1307_SEC,false));//считать секунды
  Serial.print("      ");                 // Простор для счастливой жизни
  Serial.print(RTC.get(DS1307_DATE,false));//считать дату
  Serial.print("/");
  Serial.print(RTC.get(DS1307_MTH,false));//считать месяц
  Serial.print("/");
  Serial.print(RTC.get(DS1307_YR,false)); //считать год
  Serial.println();

  delay(1000);
*/
}
  
    
}

/****************************************************************************
 * Начинаем MODBUS RTU SLAVE функции
 ****************************************************************************/

/* глобальные переменные*/
unsigned int Txenpin = 0;        /*Включаем передачу с контактов, используемые на сетях RS485  */


/* перечисление кодов поддерживаемые функции Modbus. При реализации новой, поставить свою функцию код здесь! */
enum { 
        FC_READ_REGS  = 0x03,   //Чтение непрерывного блока регистра временного хранения
        FC_WRITE_REG  = 0x06,   //Запись одного регистра (holding register)
        FC_WRITE_REGS = 0x10    //Запись блока смежных регистров (contiguous registers)
};

/* Поддерживаемых функций. При реализации новой, свою функцию кода в этом массиве! */
const unsigned char fsupported[] = { FC_READ_REGS, FC_WRITE_REG, FC_WRITE_REGS };

/* Константы */
enum { 
        MAX_READ_REGS = 0x7D, 
        MAX_WRITE_REGS = 0x7B, 
        MAX_MESSAGE_LENGTH = 256 
};


enum { 
        RESPONSE_SIZE = 6, 
        EXCEPTION_SIZE = 3, 
        CHECKSUM_SIZE = 2 
};

/* Исключения код */
enum { 
        NO_REPLY = -1, 
        EXC_FUNC_CODE = 1, 
        EXC_ADDR_RANGE = 2, 
        EXC_REGS_QUANT = 3, 
        EXC_EXECUTE = 4 
};

/* позиции внутри запроса / ответа массива */
enum { 
        SLAVE = 0, 
        FUNC, 
        START_H, 
        START_L, 
        REGS_H, 
        REGS_L, 
        BYTE_CNT 
};


/*
CRC
 
 INPUTS:
 	buf   ->  Массив, содержащий сообщение, которое будет отправлено на контроллер.            
 	start ->  Начало петлю в crc counter, обычно 0.
 	cnt   -> количество байт в сообщении послано на контроллер/
 OUTPUTS:
 	temp  ->  Ret возвращает crc байт для сообщения.
 COMMENTS:
   Эта подпрограмма вычисляет crc высоких и низких байтов сообщений.
  Заметим, что этот crc используется только для Modbus, не Modbus + и так далее.
  
 ****************************************************************************/

unsigned int crc(unsigned char *buf, unsigned char start,
unsigned char cnt) 
{
        unsigned char i, j;
        unsigned temp, temp2, flag;

        temp = 0xFFFF;

        for (i = start; i < cnt; i++) {
                temp = temp ^ buf[i];

                for (j = 1; j <= 8; j++) {
                        flag = temp & 0x0001;
                        temp = temp >> 1;
                        if (flag)
                                temp = temp ^ 0xA001;
                }
        }

        /* обратный порядок байтов */
        temp2 = temp >> 8;
        temp = (temp << 8) | temp2;
        temp &= 0xFFFF;

        return (temp);
}




/***********************************************************************
 * 
 * 	Следующие функции построят требуемый запрос в
 *                  пакет Modbus запроса. 
 * 	
 ***********************************************************************/

/* 
 * Запуск пакета read_holding_register. Ответ
 */
void build_read_packet(unsigned char slave, unsigned char function,
unsigned char count, unsigned char *packet) 
{
        packet[SLAVE] = slave;
        packet[FUNC] = function;
        packet[2] = count * 2;
} 

/* 
 * Запуск пакета preset_multiple_register. Ответ
 */
void build_write_packet(unsigned char slave, unsigned char function,
unsigned int start_addr, 
unsigned char count,
unsigned char *packet) 
{
        packet[SLAVE] = slave;
        packet[FUNC] = function;
        packet[START_H] = start_addr >> 8;
        packet[START_L] = start_addr & 0x00ff;
        packet[REGS_H] = 0x00;
        packet[REGS_L] = count;
} 

/* 
 * Запуск пакета write_single_register. Ответ
 */
void build_write_single_packet(unsigned char slave, unsigned char function,
        unsigned int write_addr, unsigned int reg_val, unsigned char* packet) 
{
        packet[SLAVE] = slave;
        packet[FUNC] = function;
        packet[START_H] = write_addr >> 8;
        packet[START_L] = write_addr & 0x00ff;
        packet[REGS_H] = reg_val >> 8;
        packet[REGS_L] = reg_val & 0x00ff;
} 


/* 
 * Запуск пакета исключения ответа
 */
void build_error_packet(unsigned char slave, unsigned char function,
unsigned char exception, unsigned char *packet) 
{
        packet[SLAVE] = slave;
        packet[FUNC] = function + 0x80;
        packet[2] = exception;
} 


/*************************************************************************
 * 
 * modbus_query ( packet, length)
 * 
 * Функция для добавления контрольной суммы в конце пакета.
 * Стоит обратитить внимание, что пакет массива должен быть не менее 2 поля длинее
 * string_length.
 **************************************************************************/

void modbus_reply(unsigned char *packet, unsigned char string_length) 
{
        int temp_crc;

        temp_crc = crc(packet, 0, string_length);
        packet[string_length] = temp_crc >> 8;
        string_length++;
        packet[string_length] = temp_crc & 0x00FF;
} 



/***********************************************************************
 * 
 * send_reply( query_string, query_length )
 * 
 * Эта функция позволяет отправить ответ на Master Modbus.
 * Возвращает: общее число отправленных символов
 ************************************************************************/

int send_reply(unsigned char *query, unsigned char string_length) 
{
        unsigned char i;

        if (Txenpin > 1) { 
                UCSR0A=UCSR0A |(1 << TXC0);
                digitalWrite( Txenpin, HIGH);
                delay(1);
        }

        modbus_reply(query, string_length);
        string_length += 2;

        for (i = 0; i < string_length; i++) {
                Serial.print(query[i], BYTE);
        }

        if (Txenpin > 1) {
                while (!(UCSR0A & (1 << TXC0)));
                digitalWrite( Txenpin, LOW);
        }

        return i; 		/*это не значит, что запись была успешной, хотя */ 
}

/***********************************************************************
 * 
 * 	receive_request( array_for_data )
 * 
 * Функция для контроля запроса от modbus master.
 * 
 * Возвращает: Общее количество полученных символов, если ОК
 * 0, если нет запроса
 * Отрицательный код ошибки в случае неудачи
 ***********************************************************************/

int receive_request(unsigned char *received_string) 
{
        int bytes_received = 0;

        /* FIXME: Есть ли Serial.available. ждать 1.5T или 3.5T перед выходом из цикла? */
        while (Serial.available()) {
                received_string[bytes_received] = Serial.read();
                bytes_received++;
                if (bytes_received >= MAX_MESSAGE_LENGTH)
                        return NO_REPLY; 	/* port error */
        }

        return (bytes_received);
}


/*********************************************************************
 * 
 * 	modbus_request(slave_id, request_data_array)
 * 
 * Функция на правильный запрос возвращается и что контрольная сумма
 * правильная.
 * 
 * Возвращает: string_length если ОК
 * 		0, если не удалось
 * 		Менее чем 0 для исключения ошибки
 * 
 * 	  Примечание: Все функции, используемые для отправки или приема данных через
 * 	      Modbus, вернуть эти значения.
 * 
 **********************************************************************/

int modbus_request(unsigned char slave, unsigned char *data) 
{
        int response_length;
        unsigned int crc_calc = 0;
        unsigned int crc_received = 0;
        unsigned char recv_crc_hi;
        unsigned char recv_crc_lo;

        response_length = receive_request(data);

        if (response_length > 0) {
                crc_calc = crc(data, 0, response_length - 2);
                recv_crc_hi = (unsigned) data[response_length - 2];
                recv_crc_lo = (unsigned) data[response_length - 1];
                crc_received = data[response_length - 2];
                crc_received = (unsigned) crc_received << 8;
                crc_received =
                        crc_received | (unsigned) data[response_length - 1];

                /*********** Проверка CRC Ответа ************/
                if (crc_calc != crc_received) {
                        return NO_REPLY;
                }

                /* Проверка для slave id */
                if (slave != data[SLAVE]) {
                        return NO_REPLY;
                }
        }
        return (response_length);
}

/*********************************************************************
  *
  * Validate_request (request_data_array, request_length, available_regs)
  *
  * Функция проверит, что запрос может быть обработан slave.
  *
  * Возвращает: 0 если OK
  * Отрицательный код исключения при ошибке
  *
 **********************************************************************/

int validate_request(unsigned char *data, unsigned char length,
unsigned int regs_size) 
{
        int i, fcnt = 0;
        unsigned int regs_num = 0;
        unsigned int start_addr = 0;
        unsigned char max_regs_num;

        /* Проверка кода фунукции */
        for (i = 0; i < sizeof(fsupported); i++) {
                if (fsupported[i] == data[FUNC]) {
                        fcnt = 1;
                        break;
                }
        }
        if (0 == fcnt)
                return EXC_FUNC_CODE;

        if (FC_WRITE_REG == data[FUNC]) {
                /* что бы написать один регистр*/
                regs_num = ((int) data[START_H] << 8) + (int) data[START_L];
                if (regs_num >= regs_size)
                        return EXC_ADDR_RANGE;
                return 0;
        }
        
        /* Для функций чтения/записи регистров, это диапазон */
        regs_num = ((int) data[REGS_H] << 8) + (int) data[REGS_L];
                
        /* check quantity of registers */
        if (FC_READ_REGS == data[FUNC])
                max_regs_num = MAX_READ_REGS;
        else if (FC_WRITE_REGS == data[FUNC])
                max_regs_num = MAX_WRITE_REGS;

        if ((regs_num < 1) || (regs_num > max_regs_num))
                return EXC_REGS_QUANT;

        /* проверка регистра диапазона, стартовый адрес это 0 */
        start_addr = ((int) data[START_H] << 8) + (int) data[START_L];
        if ((start_addr + regs_num) > regs_size)
                return EXC_ADDR_RANGE;

        return 0; 		/* OK, нет исключения */
}



/************************************************************************
  * 
  * 	write_regs(first_register, data_array, registers_array)
  *
  * Пишет в  slave's holding регистр данные в запросе,
  * Начиная с start_addr.
  *
  * Возвращает: написаное число регистров
 ************************************************************************/

int write_regs(unsigned int start_addr, unsigned char *query, int *regs) 
{
        int temp;
        unsigned int i;

        for (i = 0; i < query[REGS_L]; i++) {
                /* Сдвиг регистра hi_byte для временной  работы */
                temp = (int) query[(BYTE_CNT + 1) + i * 2] << 8;
                /* или с lo_byte  */
                temp = temp | (int) query[(BYTE_CNT + 2) + i * 2];

                regs[start_addr + i] = temp;
        } 
        return i;
}

/************************************************************************
 * 
 * 	preset_multiple_registers(slave_id, first_register, number_of_registers,
 * data_array, registers_array)
 * 
 * 	Запись данных из массива в holding registers slave. 
 * 
 *************************************************************************/

int preset_multiple_registers(unsigned char slave,
unsigned int start_addr,
unsigned char count, 
unsigned char *query,
int *regs) 
{
        unsigned char function = FC_WRITE_REGS;	/* Установка нескольких регистров */
        int status = 0;
        unsigned char packet[RESPONSE_SIZE + CHECKSUM_SIZE];

        build_write_packet(slave, function, start_addr, count, packet);

        if (write_regs(start_addr, query, regs)) {
                status = send_reply(packet, RESPONSE_SIZE);
        }

        return (status);
}


/************************************************************************
 * 
 * write_single_register(slave_id, write_addr, data_array, registers_array)
 * 
 * 
 *************************************************************************/

int write_single_register(unsigned char slave,
        unsigned int write_addr, unsigned char *query, int *regs) 
{
        unsigned char function = FC_WRITE_REG; /* Функция: Запись одного регистра */
        int status = 0;
        unsigned int reg_val;
        unsigned char packet[RESPONSE_SIZE + CHECKSUM_SIZE];

        reg_val = query[REGS_H] << 8 | query[REGS_L];
        build_write_single_packet(slave, function, write_addr, reg_val, packet);
        regs[write_addr] = (int) reg_val;
/*
        written.start_addr=write_addr;
        written.num_regs=1;
*/
        status = send_reply(packet, RESPONSE_SIZE);    

        return (status);
}


/************************************************************************
 * 
 * 	read_holding_registers(slave_id, first_register, number_of_registers,
 * registers_array)
 * 
 * читаем slave's holdings registers и отправляем его на Modbus master
 * 
 *************************************************************************/

int read_holding_registers(unsigned char slave, unsigned int start_addr,

unsigned char reg_count, int *regs) 
{
        unsigned char function = 0x03; 	/* Функция 03: Считываем Holding Registers */
        int packet_size = 3;
        int status;
        unsigned int i;
        unsigned char packet[MAX_MESSAGE_LENGTH];

        build_read_packet(slave, function, reg_count, packet);

        for (i = start_addr; i < (start_addr + (unsigned int) reg_count);
	       i++) {
                        packet[packet_size] = regs[i] >> 8;
                packet_size++;
                packet[packet_size] = regs[i] & 0x00FF;
                packet_size++;
        } 

        status = send_reply(packet, packet_size);

        return (status);
}


void configure_mb_slave(long baud, char parity, char txenpin)
{
        Serial.begin(baud);

        switch (parity) {
        case 'e': // 8E1
                UCSR0C |= ((1<<UPM01) | (1<<UCSZ01) | (1<<UCSZ00));
                //      UCSR0C &= ~((1<<UPM00) | (1<<UCSZ02) | (1<<USBS0));
                break;
        case 'o': // 8O1
                UCSR0C |= ((1<<UPM01) | (1<<UPM00) | (1<<UCSZ01) | (1<<UCSZ00));
                //      UCSR0C &= ~((1<<UCSZ02) | (1<<USBS0));
                break;
        case 'n': // 8N1
                UCSR0C |= ((1<<UCSZ01) | (1<<UCSZ00));
                //      UCSR0C &= ~((1<<UPM01) | (1<<UPM00) | (1<<UCSZ02) | (1<<USBS0));
                break;                
        default:
                break;
        }

        if (txenpin > 1) { // pin 0 & pin 1 are reserved for RX/TX
                Txenpin = txenpin; /* установим глобальную переменную */
                pinMode(Txenpin, OUTPUT);
                digitalWrite(Txenpin, LOW);
        }

        return;
}   

/*
 * update_mb_slave(slave_id, holding_regs_array, number_of_regs)
 * 
 * Проверяем, есть ли любой корректный запрос от Modbus Master. Если есть,
 * Выполняет запрашиваемое
 */

unsigned long Nowdt = 0;
unsigned int lastBytesReceived;
const unsigned long T35 = 5;

int update_mb_slave(unsigned char slave, int *regs,
unsigned int regs_size) 
{
        unsigned char query[MAX_MESSAGE_LENGTH];
        unsigned char errpacket[EXCEPTION_SIZE + CHECKSUM_SIZE];
        unsigned int start_addr;
        int exception;
        int length = Serial.available();
        unsigned long now = millis();

        if (length == 0) {
                lastBytesReceived = 0;
                return 0;
        }

        if (lastBytesReceived != length) {
                lastBytesReceived = length;
                Nowdt = now + T35;
                return 0;
        }
        if (now < Nowdt) 
                return 0;

        lastBytesReceived = 0;

        length = modbus_request(slave, query);
        if (length < 1)
                return length;
        

        exception = validate_request(query, length, regs_size);
        if (exception) {
                        build_error_packet(slave, query[FUNC], exception,
                        errpacket);
                        send_reply(errpacket, EXCEPTION_SIZE);
                        return (exception);
        } 
                
                
        start_addr = ((int) query[START_H] << 8) +
                      (int) query[START_L];
        switch (query[FUNC]) {
                case FC_READ_REGS:
                        return read_holding_registers(slave, 
                        start_addr,
                        query[REGS_L],
                        regs);
                break;
                case FC_WRITE_REGS:
                        return preset_multiple_registers(slave,
                        start_addr,
                        query[REGS_L],
                        query,
                        regs);
                break;
                case FC_WRITE_REG:
                        write_single_register(slave,
                        start_addr,
                        query,
                        regs);
                break;                                
        }
}





