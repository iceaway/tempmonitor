#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rbuf.h"

#define MAX_ARGC        8 
#define PRINTS_BUFSIZE  128
#define BUFSIZE         64

//#define LOUD

#define PREAMBLE_2      0xAB
#define STX_BYTE        0xAC

#define MAX_NO_DEVICES  16 

#define ASCII_DEL  0x7F
#define ASCII_BS   0x08

#define ERR_INVALID_CMD         1
#define ERR_INVALID_CMDLENGTH   2
#define ERR_INVALID_ADDR        3
#define ERR_DEVICE_UNAVAILABLE  4

#define ABS(x)        ((x) < 0) ? (-(x)) : (x)

void color_stack(void) __attribute__ ((naked)) \
       __attribute__ ((section (".init3")));

struct cmd {
  char const * const cmd;
  char const * const help;
  int (*callback_fn)(int argc, char *argv[]);
};

struct temprh {
  uint32_t last_update;
  uint32_t timeouts;
  uint16_t rh;
  int16_t temperature;
  uint8_t valid;
};

int cmd_time(int argc, char *argv[]);
int cmd_sens(int argc, char *argv[]);

extern uint8_t _end;
extern uint8_t __stack;
static volatile uint32_t g_ticks = 0;
static volatile uint8_t g_bitsync = 0;
static volatile uint8_t g_bytesync = 0;
static uint32_t g_freq;
static uint32_t g_overflow = 0;
static struct rbuf g_rxbuf;
static struct rbuf g_txbuf;
static struct rbuf g_rfrxbuf;
static uint8_t g_echo = 1;
static uint8_t g_shiftreg = 0;
static volatile uint16_t g_diff = 0;
static struct temprh g_temprhcache[MAX_NO_DEVICES];

const struct cmd commands[] = {
  { "time",  "Show current time", cmd_time },
  { "sens",  "Sensor information", cmd_sens },
  { NULL, NULL, NULL }
};

void color_stack(void)
{
  uint8_t *p = &_end;
  while (p <= &__stack)
    *p++ = 0xcc;
}

static void transmit(void)
{
  /* Enable the TX interrupt */
  UCSR0B |= (1 << UDRIE0);
  /* Trigger interrupt for first transmission */
  UCSR0A |= (1 << UDRE0);
}

ISR(TIMER0_OVF_vect)
{
  //PORTB ^= _BV(PORTB4);
}

#define US_TO_TICK(us)  ((us)*2)
#define TICK_TO_US(tick)  ((tick)/2)

/* T on TX end is 260 us, accept +/- 80 us */
#define LIM_LO  US_TO_TICK(180)
#define LIM_HI  US_TO_TICK(340)

ISR(TIMER1_OVF_vect)
{
  ++g_overflow;
  g_bitsync = 0;
  g_bytesync = 0;
  g_shiftreg = 0;
  //PORTB &= ~_BV(PORTB4);
}

ISR(TIMER1_CAPT_vect)
{
  uint16_t diff = ICR1;
  uint8_t bitval;
  static uint8_t t_check = 0;
  static uint8_t bitcnt = 0;
  static uint8_t rxbyte = 0;
  uint8_t rxbit = 0;
  enum state {
    IDLE,
    SYNC
  };
  static enum state s = IDLE;
  TCNT1 = 0;

  /* Change capture edge */
  TCCR1B ^= _BV(ICES1);
  //PORTB ^= _BV(PORTB4);

  if (g_overflow == 0) {

    switch (s) {
    case IDLE:
      if ((diff >= (LIM_LO*2)) && (diff <= (LIM_HI*2))) {
        s = SYNC;
        g_bitsync = 1;
        //PORTB |= _BV(PORTB4);
        bitval = PINB & (1 << PB0);
        g_shiftreg = bitval;

        /*
        if (bitval)
          PORTB |= _BV(PORTB4);
        else 
          PORTB &= ~_BV(PORTB4);
         */

      }
      break;

    case SYNC:
      if ((diff >= LIM_LO) && (diff <= LIM_HI)) {
        if (!t_check) {
          t_check = 1;
        } else {
          t_check = 0;
          bitval = PINB & (1 << PB0);
          g_shiftreg <<= 1;
          g_shiftreg |= bitval;
          rxbit = 1;
          /*
          if (bitval)
            PORTB |= _BV(PORTB4);
          else 
            PORTB &= ~_BV(PORTB4);
          */
          }

      } else if ((diff >= (LIM_LO*2)) && (diff <= (LIM_HI*2))) {
        if (t_check) {
          t_check = 0;
          s = IDLE;
          g_bitsync = 0;
          g_bytesync = 0;
          //PORTB &= ~_BV(PORTB4);
          break;
        }

        bitval = PINB & (1 << PB0);
        g_shiftreg <<= 1;
        g_shiftreg |= bitval;
        rxbit = 1;
        /*
        if (bitval)
          PORTB |= _BV(PORTB4);
        else 
          PORTB &= ~_BV(PORTB4);
         */
      } else {
        s = IDLE;
        g_shiftreg = 0;
        g_bitsync = 0;
        g_bytesync = 0;
        //PORTB &= ~_BV(PORTB4);
      }

      if (g_bytesync && rxbit) {
        if (bitcnt == 0) {
          //PORTB |= _BV(PORTB4);
          rxbyte |= bitval;
        } else if (bitcnt > 0) {
          rxbyte <<= 1;
          rxbyte |= bitval;
        }

        if (bitcnt == 7) {
          rbuf_push(&g_rfrxbuf, rxbyte);
          bitcnt = 0;
          rxbyte = 0;
          //PORTB &= ~_BV(PORTB4);
        } else {
          ++bitcnt;
        }
      }

      if ((g_shiftreg == PREAMBLE_2) && !g_bytesync) {
        g_bytesync = 1;
        bitcnt = 0;
      }

      break;
    }
  } else {
    g_overflow = 0;
    g_bitsync = 0;
    g_bytesync = 0;
    s = IDLE;
    g_shiftreg = 0;
    //PORTB &= ~_BV(PORTB4);
  }
}

ISR(USART_UDRE_vect)
{
  char tmp;
  if (rbuf_pop(&g_txbuf, &tmp)) {
    UDR0 = tmp;
  } else {
    /* Nothing more to send, disable interrupt */
    UCSR0B &= ~(1 << UDRIE0);
  }
}

ISR(USART_RX_vect)
{
  uint8_t tmp;

  tmp = UDR0;
  //UDR0 = tmp; /* Echo */
  rbuf_push(&g_rxbuf, tmp);
}

ISR(TIMER0_COMPA_vect)
{
  ++g_ticks;
  PORTB ^= _BV(PORTB4);
  if ((g_ticks % 500) == 0)
    PORTB ^= _BV(PORTB5);
}

static void print_char(char data)
{
  rbuf_push(&g_txbuf, data);
  transmit();
}

static void echo(char data)
{
  if (g_echo) {
    if (data == ASCII_DEL) /* Make backspace work */
      data = ASCII_BS;
    print_char(data);
  }
}

int print_string(char *string)
{
  while (*string) {
    while (rbuf_push(&g_txbuf, *string) != 1);
    string++;
  } 
  transmit();
  return 0;
}

int prints(const char *fmt, ...)
{
  va_list ap;
  char buf[PRINTS_BUFSIZE];
  int size;

  va_start(ap, fmt);
  size = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  print_string(buf);
  return size;
}


static int parse_args(char *buf, char *argv[])
{
  char *p;
  int argc = 0;
  p = buf;

  argv[argc++] = p;

  while (*p) {
    if ((*p == ' ') && (*(p+1) != '\0')) {
      *p = '\0';
      argv[argc++] = p+1;
    }

    if (argc >= MAX_ARGC) {
      argv[argc] = 0;
      break;
    }
    ++p;
  }

  return argc;
}

static int hex2int(char c)
{
  if ((c >= 'a') && (c <= 'f'))
    return c - 'a' + 10;
  else if ((c >= 'A') && (c <= 'F')) 
    return c - 'A' + 10;
  else if ((c >= '0') && (c <= '9'))
    return c - '0';
  else
    return -1;
}

static int parse_cmd_serial_proto(char data)
{
  /* 
   * There are three host commands:
   * rtX - Read temperature of sensor X (0-F)
   * rhX - Read relative humidity of sensor X
   * rbX - Read both tem and RH of sensor X
   * rxX - Read both temp and RH of sensor X, also send time since last
   *       time temp/rh from the current sensor was read in seconds
   * All commands are terminated with a newline. The response is sent as
   * rtX: TTT\n
   * rhX: HHH\n
   * rbX: TTT;HHH\n
   * rxX: TTT;HHH;TTT\n
   * 
   * A faulty command receives the reply e followed by a two digit error code
   * ex. e01\n
   *
   * Error codes:
   * 01 - Invalid command
   * 02 - Invalid command length
   * 03 - Invalid device address
   * 04 - Device unavailable
   */
  static char buf[8] = { 0 };
  static unsigned int idx = 0;
  int device;
  int ret = 0;

  if (data != '\n') {
    if (idx < sizeof(buf)) {
      buf[idx++] = data;
    }
  } else {
    if (idx != 3) {
      ret = -ERR_INVALID_CMDLENGTH;
      prints("e%02d;%d\n", ERR_INVALID_CMDLENGTH, idx);
      idx = 0;
      goto out;
    }

    device = hex2int(buf[2]);
    if ((device < 0) || (device > (MAX_NO_DEVICES-1))) {
      ret = -ERR_INVALID_ADDR;
      prints("e%02d\n", ERR_INVALID_ADDR);
      idx = 0;
      goto out;
    }

    if (!g_temprhcache[device].valid) {
      ret = -ERR_DEVICE_UNAVAILABLE;
      prints("e%02d%x\n", ERR_DEVICE_UNAVAILABLE, device);
      idx = 0;
      goto out;
    }

    if (buf[0] == 'r') {
      switch (buf[1]) {
      case 't':
        prints("%d.%d\n",
               g_temprhcache[device].temperature / 10,
               g_temprhcache[device].temperature < 0 ?
               (-g_temprhcache[device].temperature) % 10 :
               g_temprhcache[device].temperature % 10);

        break;

      case 'h':
        prints("%u.%u\n",
               g_temprhcache[device].rh / 10,
               g_temprhcache[device].rh % 10);
        break;

      case 'b':
        prints("%d.%d;%u.%u\n",
               g_temprhcache[device].temperature / 10,
               g_temprhcache[device].temperature < 0 ?
                 (-g_temprhcache[device].temperature) % 10 :
                 g_temprhcache[device].temperature % 10,
               g_temprhcache[device].rh / 10,
               g_temprhcache[device].rh % 10);
        break;

      case 'x':
        prints("%d.%d;%u.%u;%u\n",
               g_temprhcache[device].temperature / 10,
               g_temprhcache[device].temperature < 0 ?
                 (-g_temprhcache[device].temperature) % 10 :
                 g_temprhcache[device].temperature % 10,
               g_temprhcache[device].rh / 10,
               g_temprhcache[device].rh % 10,
               (g_ticks - g_temprhcache[device].last_update) / 500);
        break;

      default:
        ret = -ERR_INVALID_CMD;
        prints("e%02d\n", ERR_INVALID_CMD);
        break;
        
      }
    } else {
      ret = -ERR_INVALID_CMD;
      prints("e%02d\n", ERR_INVALID_CMD);
    }

    idx = 0;
  }
    
out:
  return ret;
}

static int parse_cmd(char data)
{
  static char buf[RBUF_SIZE];
  static int idx = 0;
  char *argv[MAX_ARGC+1];
  int argc = 0;
  const struct cmd *p;
  int len;
  int ret = 0;
  int process = 0;

  if (data != '\r') {
    buf[idx++] = data;
    if (idx >= (RBUF_SIZE-1)) {
      buf[idx] = '\0';
      len = idx;
      idx = 0;
      process = 1;
    }
  } else {
    process = 1;
    buf[idx] = '\0';
    len = idx;
    idx = 0;
  }

  if (process) {
    if (len != 0) {
      argc = parse_args(buf, argv);

      p = &commands[0];
      while (p->cmd) {
        if (strncmp(p->cmd, buf, sizeof(buf)) == 0) {
          p->callback_fn(argc, argv);
          break;
        }
        ++p;
      }

      if (!p->cmd)
        prints("Unknown command '%s'\r\n", buf);

    }
    ret = 1;
    memset(buf, 0, sizeof(buf));
  }

  return ret; 
}

int cmd_sens(int argc, char *argv[])
{
  unsigned long id;
  int16_t temp;
  uint16_t rh;
  
  if (argc <= 1) {
    prints("usage: sens <show/clear> <id (0-15)>\r\n");
  } else if (argc >= 3) {
    if (strncmp(argv[1], "show", 4) == 0) {
      id = strtoul(argv[2], NULL, 10);
      if (id <= 15) {
        temp = g_temprhcache[id].temperature;
        rh = g_temprhcache[id].rh;
        prints("Sensor %lu\r\n", id);
        prints("        Valid: %s\r\n", g_temprhcache[id].valid ? "Yes" : "No");
        prints("  Temperature: %d.%d\r\n", 
               temp / 10,
               temp < 0 ? (-temp) % 10 : temp % 10);
        prints("           RH: %u.%u%%\r\n",
               rh / 10, rh % 10);          
        prints("  Last update: %lu s ago\r\n",
               (g_ticks - g_temprhcache[id].last_update) / 500);  
        prints("     Timeouts: %lu\r\n", g_temprhcache[id].timeouts);
               
      } else {
        /* Show all sensor info */
      }
    } else if (strncmp(argv[1], "clear", 5)) {
    } else {
      prints("Invalid command: '%s'\r\n", argv[2]);
    }
  } else {
    prints("Not enough arguments\r\n");
  }

  return 0;
}

int cmd_time(int argc, char *argv[])
{
  (void)argc;
  (void)argv;
  prints("Current time: %lu.%02lu\r\n", g_ticks/500, g_ticks % 500);
  return 0;
}

static void init_timers(void)
{
  /* CTC mode for ICP, prescaler = clk / 8 => 0.5 us per timer step  */
  TCCR1B = (1 << CS11);

  /* Set up the ICP for manchester decoding */
  /* Enable Input Capture interrupt */
  TIMSK1 = (1 << ICIE1) | (1 << TOIE1);

  /* 
   * Set up system tick. OCR0A = 125 and prescaler /256 gives a systick of 
   * 2 ms 
   */
   
  TCNT0 = 0;
  OCR0A = 125;
  /* Enable Output Compare A interrupt */
  TIMSK0 = (1 << OCIE0A);

  /* CTC mode */
  TCCR0A = (1 << WGM01);

  /* Prescaler = f_cpu / 256 */
  TCCR0B = (1 << CS02);
}

static void init_gpio(void)
{
  /* Debug output - digital pin 12 */
  DDRB |= (1 << DDB4);

  /* Incoming data, ICP. Input and pull-up */
  DDRB &= ~(1 << DDB0);
  PORTB |= (1 << PB0);

  /*
   * PD7 = Mode, shell or serial protocol to host. Pull-up active =>
   * NC =  Serial protocol
   * GND = Shell
   * PD7 = Digital Pin 7
   */
  DDRD &= ~(1 << DDD7); /* Input */
  PORTD |= (1 << PD7);  /* Pull-up */

#if 0
  /* Set PB5 as output for LED */
  DDRB |= (1 << DDB5);

  /* Set PD2 as output for relay */
  DDRD |= (1 << DDD2);

  /* Activate pull-up for motor control input pin */
  PORTB |= (1 << PB1);

  /* Enable interrupts for PCINT [7:0] */
  PCICR |= (1 << PCIE0);

  /* Mask interrupts for PCINT1 */
  PCMSK0 |= (1 << PCINT1);
#endif
}

static void init_usart(void)
{
  /* Set baud rate to 9600 */
  UBRR0H = 0;
  UBRR0L = 103; /* for 9600, required for USART0 (USB) */

  /* Enable TX and RX */
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);

  /* Set 8N1 frame format */
  UCSR0C = (1 << USBS0) | (3 << UCSZ00);

  /* Enable RX interrupt */
  UCSR0B |= (1 << RXCIE0);

}

static uint8_t crc(uint8_t *buf, size_t buflen)
{
  uint8_t crc = 0;
  size_t i;

  for (i = 0; i < buflen; ++i)
    crc ^= buf[i];

  return crc;
}

static void handle_data(unsigned char *data, int len)
{
  int i;
  static unsigned char buf[BUFSIZE];
  static unsigned int idx = 0;
  static int datactr = 0;
  static int8_t pseqno;
  static uint8_t seqno;
  static uint8_t devaddr = 0;
  uint8_t crc8;
  int16_t temp;
  uint16_t rh;

  static enum {
    STX,
    ADDR,
    TYPE,
    DATA,
    CRC
  } state = STX;


  for (i = 0; i < len; ++i) {
    /* Make sure that we don't read more data than fits in the read buffer.
     * If too many bytes are read we reset the state machine and wait for
     * another message.
     */
    if (idx >= sizeof(buf)) {
      state = STX;
      idx = 0;
    }

    switch (state) {
    case STX:
      if (data[i] == STX_BYTE) {
        memset(&buf, 0, sizeof(buf));
        buf[idx++] = data[i];
        state = ADDR;
      }
      break;

    case ADDR:
      buf[idx++] = data[i];
      seqno = (data[i] & 0xf0) >> 4;
      devaddr = data[i] & 0x0f;
      state = TYPE;
      break;

    case TYPE:
      buf[idx++] = data[i];
      datactr = 0;
      state = DATA;
      break;

    case DATA:
      buf[idx++] = data[i];
      if (++datactr == 4)
        state = CRC;
      break;

    case CRC:
      buf[idx++] = data[i];
      crc8 = crc(buf, idx-1);
      if (crc8 == buf[idx-1]) {
        temp = (buf[3] << 8) | buf[4];
        rh = (buf[5] << 8) | buf[6];
        if (seqno != pseqno) {
#ifdef LOUD
          prints("Temperature: %d.%d degC, RH = %u.%u%%, Seqno = %u (from %u)\r\n",
                 temp / 10,
                 temp < 0 ? (-temp) % 10 : temp % 10,
                 rh / 10,
                 rh % 10,
                 seqno,
                 devaddr);
#endif
          /* Update cached temperature reading */
          if (devaddr < MAX_NO_DEVICES) {
            g_temprhcache[devaddr].temperature = temp;
            g_temprhcache[devaddr].rh = rh;
            g_temprhcache[devaddr].last_update = g_ticks;
            g_temprhcache[devaddr].valid = 1;
          }
        }
        pseqno = seqno;
       
      } else {
#ifdef LOUD
        prints("Message CRC failed\r\n");
#endif
      }
      state = STX;
      idx = 0;
      break;
    }
  }
}

static int get_mode(void)
{
  return (PIND & (1 << PIND7)) ? 0 : 1;
}

int main(void)
{
  char tmp;
  uint8_t byte = 0;
  int i;

  init_timers();
  init_gpio();
  init_usart();

  rbuf_init(&g_rxbuf);
  rbuf_init(&g_txbuf);
  rbuf_init(&g_rfrxbuf);

  memset(g_temprhcache, 0, sizeof(g_temprhcache));

  /* Enable interrupts globally */
  sei(); 

  for (;;) {
    if (rbuf_pop(&g_rxbuf, &tmp)) {
      if (get_mode()) {
        /* Shell mode */
        echo(tmp);
        if (tmp == '\r')
          echo('\n');
        if (parse_cmd(tmp)) {
          prints(">> ");
        }
      } else {
        /* Serial protocol to host mode */
        parse_cmd_serial_proto(tmp);
      }
    }

    while (rbuf_pop(&g_rfrxbuf, &byte)) {
      handle_data(&byte, 1);
    }

    /* Invalidate devices which have been silent for 10 mins or more */
    for (i = 0; i < MAX_NO_DEVICES; ++i) {
      if (((g_ticks - g_temprhcache[i].last_update) / 500) > 600) {
        g_temprhcache[i].valid = 0;
      }

      /*
      if (((g_ticks - g_temprhcache[i].last_update) / 500) > 65) {
        ++g_temprhcache[i].timeouts;
      }
      */
    }
  }
}
