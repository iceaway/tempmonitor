#include <stdio.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <time.h>
#include <signal.h>
#include <stdint.h>
#include <sys/poll.h>
#include <linux/limits.h>
#ifdef USE_DATABASE
#include <mysql/my_global.h>
#include <mysql/mysql.h>
#endif

#define BUFSIZE           128

#define STX               0xAC  /* Start Of Message */

static volatile int g_finished = 0;
#ifdef USE_DATABASE
static MYSQL *con;
#endif

static uint8_t crc(uint8_t *buf, size_t buflen)
{
  uint8_t crc = 0;
  size_t i;

  for (i = 0; i < buflen; ++i)
    crc ^= buf[i];

  return crc;
}

static int set_interface_attribs(int fd, int speed, int parity)
{
  struct termios tty;
  memset (&tty, 0, sizeof(tty));
  if (tcgetattr (fd, &tty) != 0)
  {
    printf("error %d from tcgetattr\n", errno);
    return -1;
  }

  cfsetospeed (&tty, speed);
  cfsetispeed (&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_iflag &= ~IGNBRK;
  tty.c_iflag |= PARMRK; /* Indicate framing errors */
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag |= parity;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr (fd, TCSANOW, &tty) != 0)
  {
    printf("error %d from tcsetattr\n", errno);
    return -1;
  }
  return 0;
}

void sigint_handler(int s)
{
  (void)s;
  g_finished = 1;
}

static void handle_data(unsigned char *data, ssize_t len)
{
  ssize_t i;
  static unsigned char buf[BUFSIZE];
  static unsigned int idx = 0;
  static int datactr = 0;
  static int8_t pseqno;
  static uint8_t seqno;
  uint8_t crc8;
  int16_t temp;
  uint16_t rh;
  FILE *templog = NULL;
  char filename[PATH_MAX];
#ifdef USE_DATABASE
  char query[128];
#endif
  struct tm *tmp;
  time_t t;
  char timestr[64];

  static enum {
    STX1,
    STX2,
    ADDR,
    TYPE,
    DATA,
    CRC
  } state = STX1;


  for (i = 0; i < len; ++i) {
    /* Make sure that we don't read more data than fits in the read buffer.
     * If too many bytes are read we reset the state machine and wait for
     * another message.
     */
    if (idx >= sizeof(buf)) {
      state = STX1;
      idx = 0;
    }

    switch (state) {
    case STX1:
      if (data[i] == STX) {
        memset(&buf, 0, sizeof(buf));
        buf[idx++] = data[i];
        state = STX2;
      }
      break;

    case STX2:
      if (data[i] == STX) {
        buf[idx++] = data[i];
        state = ADDR;
      }
      break;

    case ADDR:
      buf[idx++] = data[i];
      seqno = (data[i] & 0xf0) >> 4;
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
        temp = (buf[4] << 8) | buf[5];
        rh = (buf[6] << 8) | buf[7];
        //printf("Temperature: %0.1f degC, RH = %0.1f%%\nSeqno = %u\n", (float)temp/10.0f, (float)rh/10.0f, seqno);
        t = time(NULL);
        tmp = localtime(&t);
        if (tmp != NULL) {
          strftime(timestr, sizeof(timestr), "%Y-%m-%d %H:%M:%S", tmp);
        }
        /* Print the long-term log file */
        snprintf(filename, PATH_MAX, "templog_%u.csv", buf[2] & 0x0f);
        templog = fopen(filename, "a+"); 
        if (templog) {
          if (seqno != pseqno) {
            printf("%s: %02u: Temperature: %0.1f degC, RH = %0.1f%%, seqno = %u\n",
                   timestr,
                   buf[2] & 0x0f,
                   (float)temp/10.0f,
                   (float)rh/10.0f,
                   seqno);
            fprintf(templog, "%s,%02u,%02u,%0.1f,%0.1f\n",
                    timestr,
                    buf[2] & 0x0f, 
                    seqno,
                    (float)temp/10.0f,
                    (float)rh/10.0f);
            fclose(templog);
#ifdef USE_DATABASE
            snprintf(query,
                     sizeof(query),
                     "INSERT INTO templog VALUES(%u, '%s', %0.1f, %0.1f)",
                     buf[2] & 0x0f, 
                     timestr,
                     (float)temp/10.0f, 
                     (float)rh/10.0f);
            if (mysql_query(con, query))
              printf("Failed to insert into database\n");
#endif
            pseqno = seqno;
          }
        } else {
          printf("Failed to open '%s' for writing data\n", filename);
        }

        /* Print the last known value to file in a format that Domoticz understands */
        snprintf(filename, PATH_MAX, "temp_rh_current_%u.csv", buf[2] & 0x0f);
        templog = fopen(filename, "w+"); 
        if (templog) {
          fprintf(templog, "%0.1f;%0.1f\n", (float)temp/10.0f, (float)rh/10.0f);
          fclose(templog);
        } else {
          printf("Failed to open '%s' for writing data. Errno = %d\n", filename, errno);
        }

      } else {
        printf("Message CRC failed\n");
      }
      state = STX1;
      idx = 0;
      break;
    }
  }
}

int main(int argc, char *argv[])
{
  int fd;
  unsigned char buf[BUFSIZE];
  struct sigaction sa;
  struct pollfd fds;
  int rc;
  int len;
#ifdef USE_DATABASE
  con = mysql_init(NULL);

  if (con == NULL) {
    printf("Failed to init mysql\n");
    goto out;
  }

  if (mysql_real_connect(con, "localhost", "tempmonitor",
                         "temp123", "temperature", 0, NULL, 0) == NULL) {
    printf("Failed to connect to MariaDB.\n");
    mysql_close(con);
    goto out;
  }
#endif

  if (argc < 2) {
    printf("Not enough arguments. Usage: %s <tty>\n", argv[0]);
    goto out;
  }

  sa.sa_handler = sigint_handler;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;

  sigaction(SIGINT, &sa, NULL);

  fd = open (argv[1], O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
  if (fd < 0)
  {
    printf("error %d opening %s: %s\n", errno, argv[1], strerror (errno));
    goto out_not_open;
  }

  set_interface_attribs(fd, B1200, 0);

  fds.fd = fd;
  fds.events = POLLIN;

  while(!g_finished) {
    rc = poll(&fds, 1, -1);
    if (rc == 0) {
      /* Timeout, should not happen since timeout = inf */
      printf("Error: poll() timeout\n");
    } else if (rc < 0) {
      printf("Call to poll() failed: %s\n", strerror(errno));
    } else {
      /* rc > 0, we poll returned successfully */
      if (fds.revents & POLLIN) {
        len = read(fd, buf, sizeof(buf));
        handle_data(buf, len);
      }
    }
  }

out:
  close(fd);
out_not_open:
  return 0;
}
