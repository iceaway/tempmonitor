#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rbuf.h"

#define IS_INPUT(ddr, bit)  ((ddr) & (1 << (bit)) ? 0 : 1)
#define IS_OUTPUT(ddr, bit)  ((ddr) & (1 << (bit)) ? 1 : 0)

#define MAX_ARGC        8 
#define PRINTS_BUFSIZE  128

#define PREAMBLE_2      0xAB

#define ASCII_DEL  0x7F
#define ASCII_BS   0x08

#define PREAMBLE   0x57

void color_stack(void) __attribute__ ((naked)) \
       __attribute__ ((section (".init3")));

struct cmd {
  char const * const cmd;
  char const * const help;
  int (*callback_fn)(int argc, char *argv[]);
};

int cmd_time(int argc, char *argv[]);
int cmd_freq(int argc, char *argv[]);

extern uint8_t _end;
extern uint8_t __stack;
static uint32_t g_ticks = 0;
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

const struct cmd commands[] = {
  { "time",  "Show current time", cmd_time },
  { "freq",  "Show current frequency at pin 12", cmd_freq },
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

/* TIMER1 ISR: Tick counter. Executed every 10 ms */
ISR(TIMER1_COMPA_vect)
{
  ++g_ticks;

  if ((g_ticks % 50) == 0) {
    PORTB ^= _BV(PORTB5);
  }

  //PORTB ^= _BV(PORTB4);

  //sched_update(g_ticks);

}
#define US_TO_TICK(us)  ((us)*2)
#define TICK_TO_US(tick)  ((tick)/2)

/* T on TX end is 260 us, accept +/1 50 us */
#define LIM_LO  US_TO_TICK(210)
#define LIM_HI  US_TO_TICK(300)

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
          PORTB &= ~_BV(PORTB4);
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
        PORTB &= ~_BV(PORTB4);
      }

      if (g_bytesync && rxbit) {
        if (bitcnt == 0) {
          PORTB |= _BV(PORTB4);
          rxbyte |= bitval;
        } else if (bitcnt > 0) {
          rxbyte <<= 1;
          rxbyte |= bitval;
        }

        if (bitcnt == 7) {
          rbuf_push(&g_rfrxbuf, rxbyte);
          bitcnt = 0;
          rxbyte = 0;
          PORTB &= ~_BV(PORTB4);
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
    PORTB &= ~_BV(PORTB4);
  }
#if 0
  lastcnt = ICR1;
  g_freq = 2000000/lastcnt;
#endif
  //PORTB ^= _BV(PORTB4);
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

int cmd_freq(int argc, char *argv[])
{
  prints("Current freq: %u\r\n", g_freq);
  return 0;
}

int cmd_time(int argc, char *argv[])
{
  prints("Current time: %lu.%02lu\r\n", g_ticks/100, g_ticks % 100);
  return 0;
}

static void init_timers(void)
{
  /* CTC mode for ICP, prescaler = clk / 8 => 0.5 us per timer step  */
  TCCR1B = (1 << CS11);

  /* Set up the IPC for manchester decoding */
  /* Enable Input Capture interrupt */
  TIMSK1 = (1 << ICIE1) | (1 << TOIE1);


#if 0
  /* Enable overflow interrupt */
  TIMSK0 = (1 << TOIE0);

  TCCR0B = (1 << CS01);
#endif
}

static void init_gpio(void)
{
  /* Debug output - digital pin 12 */
  DDRB |= (1 << DDB4);

  /* Incoming data, ICP. Input and pull-up */
  DDRB &= ~(1 << DDB0);
  PORTB |= (1 << PB0);


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

int main(void)
{
  char tmp;
  uint32_t last = 0;
  uint16_t adcval;
  uint8_t first = 1;
  uint16_t lastdiff = 0;
  uint8_t byte = 0;

  init_timers();
  init_gpio();
  init_usart();

  rbuf_init(&g_rxbuf);
  rbuf_init(&g_txbuf);
  rbuf_init(&g_rfrxbuf);


  /* Enable interrupts globally */
  sei(); 

  for (;;) {
    if (rbuf_pop(&g_rxbuf, &tmp)) {
      echo(tmp);
      if (tmp == '\r')
        echo('\n');
      if (parse_cmd(tmp)) {
        prints(">> ");
      }
    }

    if (g_bitsync) {
      if (first) {
        prints("Sync!\r\n");
        first = 0;
      }

      if (g_bytesync) {
        prints("Got preamble!\r\n");
      }
    } else {
      first = 1;
    }

    if (rbuf_pop(&g_rfrxbuf, &byte)) {
      prints("Received: %02x\r\n", byte);
    }
  }
}
