#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>

#include <pigpio.h>
#include <sys/time.h>

/*
2014-08-20

gcc -o freq_count_1 freq_count_1.c -lpigpio -lpthread
$ sudo ./freq_count_1  4 7 8

This program uses the gpioSetAlertFunc function to request
a callback (the same one) for each gpio to be monitored.

EXAMPLES

Monitor gpio 4 (default settings)
sudo ./freq_count_1  4

Monitor gpios 4 and 8 (default settings)
sudo ./freq_count_1  4 8

Monitor gpios 4 and 8, sample rate 2 microseconds
sudo ./freq_count_1  4 8 -s2

Monitor gpios 7 and 8, sample rate 4 microseconds, report every second
sudo ./freq_count_1  7 8 -s4 -r10

Monitor gpios 4,7, 8, 9, 10, 23 24, report five times a second
sudo ./freq_count_1  4 7 8 9 10 23 24 -r2

Monitor gpios 4, 7, 8, and 9, report once a second, sample rate 1us,
generate 2us edges (4us square wave, 250000 highs per second).
sudo ./freq_count_1  4 7 8 9 -r 10 -s 1 -p 2
*/

#define MAX_GPIOS 32

#define OPT_P_MIN 1
#define OPT_P_MAX 1000
#define OPT_P_DEF 20

#define OPT_R_MIN 1
#define OPT_R_MAX 10
#define OPT_R_DEF 5

#define OPT_S_MIN 1
#define OPT_S_MAX 10
#define OPT_S_DEF 5

static volatile int g_pulse_count[MAX_GPIOS];
static uint32_t g_mask;

static int g_num_gpios;
static int g_digits[MAX_GPIOS];

// a b c d e f g DP
//static int g_segments[] = {26, 19, 13, 6, 5, 22, 27, 17};
// DP g f e d c b a
static int g_segments[] =   {17, 27, 22, 5, 6, 13, 19, 26}; //TODO: make this configurable
static int seg_patterns[10]; // 0 1 2 3 4 5 6 7 8 9
static int seg_bitpattern_digit_mask;
static int seg_bitpattern_fp_mask;
static int digit_bitpatterns[10]; // 0 1 2 3 4 5 6 7 8 9

static int g_opt_p = OPT_P_DEF;
static int g_opt_r = OPT_R_DEF;
static int g_opt_s = OPT_S_DEF;
static int g_opt_t = 0;

static char error_msgs[5][50] = {
  {""},
  {"Uninitialized"},
  {"Collapsed"},
  {"Unconfirmed"},
  {"Out-of-sync"}
};

typedef struct EightSegment {
  int is_null;
  int is_collapsed;
  int is_out_of_sync;
  int digit;
  int fp;
} s_8segment;

typedef struct SSD {
  int size;
  int gpio[8];
  int gpio_bitmask;
  s_8segment digits[8];
  float val;
  int repeat;
  int error; // 1: uninitialized, 2: collapsed, 3: unconfirmed, 4: out-of-sync
} s_ssd;

void usage()
{
   fprintf
   (stderr,
      "\n" \
      "Usage: sudo ./freq_count_1 gpio ... [OPTION] ...\n" \
      "   -p value, sets pulses every p micros, %d-%d, TESTING only\n" \
      "   -r value, sets refresh period in deciseconds, %d-%d, default %d\n" \
      "   -s value, sets sampling rate in micros, %d-%d, default %d\n" \
      "\nEXAMPLE\n" \
      "sudo ./freq_count_1 4 7 -r2 -s2\n" \
      "Monitor gpios 4 and 7.  Refresh every 0.2 seconds.  Sample rate 2 micros.\n" \
      "\n",
      OPT_P_MIN, OPT_P_MAX,
      OPT_R_MIN, OPT_R_MAX, OPT_R_DEF,
      OPT_S_MIN, OPT_S_MAX, OPT_S_DEF
   );
}

void fatal(int show_usage, char *fmt, ...)
{
   char buf[128];
   va_list ap;

   va_start(ap, fmt);
   vsnprintf(buf, sizeof(buf), fmt, ap);
   va_end(ap);

   fprintf(stderr, "%s\n", buf);

   if (show_usage) usage();

   fflush(stderr);

   exit(EXIT_FAILURE);
}

static int initOpts(int argc, char *argv[])
{
   int i, opt;

   while ((opt = getopt(argc, argv, "p:r:s:")) != -1)
   {
      i = -1;

      switch (opt)
      {
         case 'p':
            i = atoi(optarg);
            if ((i >= OPT_P_MIN) && (i <= OPT_P_MAX))
               g_opt_p = i;
            else fatal(1, "invalid -p option (%d)", i);
            g_opt_t = 1;
            break;

         case 'r':
            i = atoi(optarg);
            if ((i >= OPT_R_MIN) && (i <= OPT_R_MAX))
               g_opt_r = i;
            else fatal(1, "invalid -r option (%d)", i);
            break;

         case 's':
            i = atoi(optarg);
            if ((i >= OPT_S_MIN) && (i <= OPT_S_MAX))
               g_opt_s = i;
            else fatal(1, "invalid -s option (%d)", i);
            break;

        default: /* '?' */
           usage();
           exit(-1);
        }
    }
   return optind;
}

char* itob(char* buf, int val, int size){
  int i;

  for (i=0; i<size; i++) {
    buf[i] = "01"[(val & 1<<(size-i-1))!=0];
  }
  buf[size] = '\0';

  return buf;
}

void to_digit(unsigned int bits_0_31, int gpio, s_8segment* seg)
{
  int i;
  char buf[32];

//printf("%s\n", itob(buf, bits_0_31, 32), i);
//printf("%s\n", itob(buf, bits_0_31 & seg_bitpattern_digit_mask, 32), i);

  if (0 == (bits_0_31 & seg_bitpattern_digit_mask)) {
    seg->is_null = 1;
    seg->digit = 0;
  } else {
    seg->is_null = 0;
    seg->is_collapsed = 1;
    for (i=0; i<10; i++) {
      if (digit_bitpatterns[i] == (bits_0_31 & seg_bitpattern_digit_mask)) {
        seg->is_collapsed = 0;
        seg->digit = i;
        break;
      }
    }
  }
  seg->fp = (bits_0_31 & seg_bitpattern_fp_mask) != 0;
}

void eval_ssd(s_ssd* ssd)
{
  int i, j, digit, digits, factor;
  float next_val;

  //if (ssd->reset == 1) {
  //  //return 1;
  //}

  //ssd->reset = 1;

  //v_digits = digits[0..2]
  //v =
  //  if v_digits.flatten.any?{|val| val.nil?}
  //    nil
  //  else
  //    v_factor = 10 ** (v_digits.reverse.index{|(d, fp)| fp} || 0)
  //    v_digits.inject(0.0) {|val, (d, fp)| val*10 + (d || 0) } / v_factor
  //  end

  digits = 0;
  for (i=0; i < ssd->size; i++) {
    if (ssd->digits[i].is_out_of_sync) {
      if (ssd->error == 1) {
        ssd->error = 4;
        ssd->repeat = 0;
      } else {
        if (ssd->repeat > 0)
          ssd->repeat -= 1;
      }
      return;
    }
    if (ssd->digits[i].is_collapsed) {
      if (ssd->error == 1) {
        ssd->error = 2;
        ssd->repeat = 0;
      } else {
        if (ssd->repeat > 0)
          ssd->repeat -= 1;
      }
      return;
    }
    digit = ssd->digits[i].digit;
//printf("%d\n", ssd->gpio[i]);
//printf("%d\n", digit);
    for (j=0; j < (ssd->size-i-1); j++) {
      digit *= 10;
    }
    digits += digit;
  }
  next_val = (float) digits;
  for (i=0; i < ssd->size; i++) {
    if (ssd->digits[i].fp) {
      for (j=i; j < (ssd->size-1); j++) {
        next_val /= 10;
      }
      break;
    }
  }

  if (ssd->error == 1) {
    ssd->error = 3;
  }

  if (ssd->val == next_val) {
    if (ssd->repeat < 50)
      ssd->repeat++;

    if (ssd->error == 3 && ssd->repeat > 5) { //TODO: This TH should be configurable
      ssd->error = 0;
    }
  } else {
    ssd->repeat = 0;
    ssd->error = 3;
    ssd->val = next_val;
  }

  return;
}

void edges(int gpio, int level, uint32_t tick, void *_ssd)
{
   int i;
   char *buf;
   unsigned int bits_0_31, gpio_other_triggers;
   s_ssd *ssd = (s_ssd*)_ssd;

   // TODO: Make this configurable to support both Cathode/Anode LEDs
   /* only record high to low edges */
   if (level == 1) return;

   //if (g_reset_counts)
   //{
   //   g_reset_counts = 0;
   //   for (g=0; g<MAX_GPIOS; g++) g_pulse_count[g] = 0;
   //}

   bits_0_31 = gpioRead_Bits_0_31();
   for (i=0; i<ssd->size; i++) {
     if (ssd->gpio[i] == gpio) {

       // should be LOW
       //seg->is_out_of_sync = ((1<<gpio) & bits_0_31) != 0; // TODO: make this configurable
       gpio_other_triggers = ssd->gpio_bitmask & ~(1<<gpio);
       //printf(" %d \n", gpio_other_triggers);
       // Other gpios should be HIGH
       ssd->digits[i].is_out_of_sync = (gpio_other_triggers & ~bits_0_31) != 0;
       to_digit(bits_0_31, gpio, &ssd->digits[i]);

       break;
     }
   }

   if (gpio == ssd->gpio[ssd->size-1]) {
     eval_ssd(ssd);
   }
   //printf("[%d, %d, %d]\n", gpio, seg->digit, seg->fp);
   //buf = itob(bits_0_31, 32);
   //printf(" %s \n", buf);
}

void ssd_setup(s_ssd* ssd, int size, int* gpio)
{
  int i;
  int mode = PI_INPUT;

  ssd->gpio_bitmask = 0;
  ssd->size = size;
  for (i=0; i<size; i++) {
    ssd->gpio[i] = gpio[i];
    ssd->gpio_bitmask |= 1<<gpio[i];
    gpioSetAlertFuncEx(gpio[i], edges, ssd);
    gpioSetMode(gpio[i], mode);
    ssd->error = 1;
    ssd->repeat = 0;
  }
}

int main(int argc, char *argv[])
{
   int i, j, rest, g, wave_id, mode;
   gpioPulse_t pulse[2];
   int count[MAX_GPIOS];
   char str_seg_pattern[8];
   char str_digit_bitpattern[32];
   struct timeval my_time;
   double unix_ts;

   s_ssd display[2];

   int v_gpio[] = {21, 20, 16};
   int a_gpio[] = {25, 24, 23};

   int error;
   float val;

   /* command line parameters */

   rest = initOpts(argc, argv);

   /* get the gpios to monitor */

   g_num_gpios = 0;

   for (i=rest; i<argc; i++)
   {
      g = atoi(argv[i]);
      if ((g>=0) && (g<32))
      {
         g_digits[g_num_gpios++] = g;
         g_mask |= (1<<g);
      }
      else fatal(1, "%d is not a valid g_gpio number\n", g);
   }

   seg_patterns[0] = strtol("11111100", NULL, 2); // a b c d e f g DP
   seg_patterns[1] = strtol("01100000", NULL, 2);
   seg_patterns[2] = strtol("11011010", NULL, 2);
   seg_patterns[3] = strtol("11110010", NULL, 2);
   seg_patterns[4] = strtol("01100110", NULL, 2);
   seg_patterns[5] = strtol("10110110", NULL, 2);
   seg_patterns[6] = strtol("10111110", NULL, 2);
   seg_patterns[7] = strtol("11100000", NULL, 2);
   seg_patterns[8] = strtol("11111110", NULL, 2);
   seg_patterns[9] = strtol("11110110", NULL, 2);

   // TODO: Build segments config from argv
   seg_bitpattern_digit_mask = 0;
   for (j=7; j>0; j--) // a b c d e f g
   {
     seg_bitpattern_digit_mask = seg_bitpattern_digit_mask | (1<<(g_segments[j]));
   }
   printf("seg_bitpattern_digit_mask: %s (gpio: 0-27)\n", itob(str_digit_bitpattern, seg_bitpattern_digit_mask, 27));

   seg_bitpattern_fp_mask = 1<<(g_segments[0]);
   printf("seg_bitpattern_fp_mask:    %s (gpio: 0-27)\n", itob(str_digit_bitpattern, seg_bitpattern_fp_mask, 27));
   for (i=0; i<10; i++) // 0 1 2 3 4 5 6 7 8 9
   {
     digit_bitpatterns[i] = 0;
     for (j=7; j>=0; j--) // a b c d e f g DP
     {
       if ((seg_patterns[i] & (1<<j)) != 0)
       {
          digit_bitpatterns[i] = digit_bitpatterns[i] | (1<<(g_segments[j]));
       }
     }

     printf("[%d]", i);
     printf(" %s (abcdefg.) =>", itob(str_seg_pattern, seg_patterns[i], 8));
     printf(" %s (gpio: 0-27)\n", itob(str_digit_bitpattern, digit_bitpatterns[i], 27));
   }

   //if (!g_num_gpios) fatal(1, "At least one gpio must be specified");

   //printf("Monitoring gpios");
   //for (i=0; i<g_num_gpios; i++) printf(" %d", g_digits[i]);
   //printf("\nSample rate %d micros, refresh rate %d deciseconds\n",
   //   g_opt_s, g_opt_r);

   gpioCfgClock(g_opt_s, 1, 1);

   if (gpioInitialise()<0) return 1;

   //gpioWaveClear();

   //pulse[0].gpioOn  = g_mask;
   //pulse[0].gpioOff = 0;
   //pulse[0].usDelay = g_opt_p;

   //pulse[1].gpioOn  = 0;
   //pulse[1].gpioOff = g_mask;
   //pulse[1].usDelay = g_opt_p;

   //gpioWaveAddGeneric(2, pulse);

   //wave_id = gpioWaveCreate();

   /* monitor g_digits level changes */

   //for (i=0; i<g_num_gpios; i++) gpioSetAlertFunc(g_digits[i], edges);
   ssd_setup(&display[0], 3, v_gpio);
   ssd_setup(&display[1], 3, a_gpio);


   //mode = PI_INPUT;

   //if (g_opt_t)
   //{
   //   gpioWaveTxSend(wave_id, PI_WAVE_MODE_REPEAT);
   //   mode = PI_OUTPUT;
   //}

   //for (i=0; i<g_num_gpios; i++) gpioSetMode(g_digits[i], mode);

   while (1)
   {
      //for (i=0; i<g_num_gpios; i++) count[i] = g_pulse_count[g_digits[i]];

      //g_reset_counts = 1;

      gettimeofday(&my_time, NULL);
      unix_ts = my_time.tv_sec + my_time.tv_usec/1000000.0;

      printf("{\"time\":%f,", unix_ts);

      printf("{\"displays\":[");
      for (i=0; i<2; i++)
      {
         printf("{");
         printf("\"idx\":%d,", i);
         if (display[i].error == 0) {
           printf("\"val\":%f", display[i].val);
         } else {
           printf("\"val\":null,\"error\":%d,\"error_msg\":\"%s\"", display[i].error, error_msgs[display[i].error]);
         }
         printf("}");

         if (i!=1) printf(",");
      }
      printf("]}");

      printf("}\n");

      gpioDelay(g_opt_r * 100000);
   }

   gpioTerminate();
}

